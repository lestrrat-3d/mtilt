# mtilt design

This document states what mtilt implements, the geometric assumptions behind it, what it leaves unchecked, and
what it leaves for later. The code is the authority where the two disagree; fix whichever is wrong.

| Question | Section |
|---|---|
| What inputs are accepted? | 1, 4 |
| Which way is up, and what unit? | 2 |
| Where do epsilons come from? | 3 |
| How is an orientation chosen? | 5 |
| What counts as an overhang? | 6 |
| How are supports built and checked? | 7 |
| What does `prepare` write? | 8 |
| What bounds the work? | 9 |
| Where does each piece of code live? | 10 |
| Why these dependencies? | 11 |

## 1. Scope

mtilt runs one pipeline:

```
read mesh -> validate -> evaluate orientations -> construct supports
          -> validate the assembly -> export geometry and report
```

Implemented:

- ASCII and binary STL input; binary STL output.
- Single-material FDM with removable supports.
- One support strategy: separate vertical pillars rooted on the build plate (section 7).

Not implemented, and not stubbed: G-code, printer control, a GUI, network services, GPU work, mesh repair, model
splitting, tree supports, supports rooted on the model, soluble or resin supports, bridge detection, mechanical
simulation, slicer profiles, and 3MF output. [roadmap.md](roadmap.md) orders them.

"Best" in mtilt always means best among the evaluated candidates under the objective in section 5.4 and the
given profile. mtilt does not claim a global optimum or printability.

## 2. Coordinates, units and the transform

- Coordinates are float64 millimeters in a right-handed frame. +Z is the build direction. The build plate is the
  plane Z = 0.
- STL carries no unit. The caller names the input unit (`mm`, `cm`, `m`, `in`); mtilt multiplies every
  coordinate by the factor (1, 10, 1000, 25.4) once, before any other step, and records the factor as
  `scale_to_mm`. mtilt never infers a unit from a model's size.
- After conversion the model moves by one proper rigid transform: a rotation R (determinant +1) followed by a
  translation t. A point p of the input file lands at

  ```
  q = R * (scale_to_mm * p) + t
  ```

- `plan.json` stores the transform as `matrix`: the 4x4 homogeneous matrix `[R t; 0 0 0 1]`, row-major, acting on
  column vectors `(x, y, z, 1)` in millimeters. `inverse` is `[R^T, -R^T t; 0 0 0 1]`. To map an output point back
  to the input file, apply `inverse` and divide by `scale_to_mm`.
- mtilt never mirrors, scales (other than the unit conversion), deforms, remeshes or merges the model. Triangle
  order and winding in `model.stl` match the input. The model and the supports are written in the same frame.

## 3. Tolerances

`mesh.Tolerance` (`mesh/tolerance.go`) is the only source of numeric epsilons. All three values scale with the
model: `scale` is the largest absolute coordinate of the model's bounding box, widened by one diagonal in every
direction so placed coordinates are covered too, and at least 1 mm.

| Field | Value | Used for |
|---|---|---|
| `Length` | 1e-9 x scale | equality of positions and heights; degenerate-triangle height; contact tests |
| `Serialization` | 2^-23 x scale (one float32 ulp) | comparing files read back after writing |
| `Plate` | 4 x 2^-23 x scale | deciding that a vertex lies on the build plate |

`Plate` exists because binary STL stores float32: a face that is flat in the source arrives with heights that
differ by a few ulps once rotated. Manufacturing clearances (`top_contact_gap_mm`, `side_clearance_mm`) are
profile values and are never mixed with these.

## 4. Input validation

`mesh.FromSoup` merges two vertex positions only when all three float64 coordinates are exactly equal. It never
welds near-equal positions. `mesh.Validate` then runs these checks; each reports `passed`, `failed` or
`unchecked`.

| Check | Fails when |
|---|---|
| `triangle_count` | fewer than 4 triangles |
| `nondegenerate_triangles` | a triangle repeats a vertex, or its height over its longest edge is at most `Length` |
| `closed` | an edge belongs to one triangle |
| `edge_manifold` | an edge belongs to more than two triangles |
| `vertex_manifold` | the triangles around a vertex form more than one fan |
| `consistent_winding` | two triangles traverse a shared edge in the same direction |
| `outward_orientation` | the signed volume is not positive (unchecked if any of the three checks above failed) |
| `single_component` | the triangles form more than one edge-connected group |
| `self_intersection` | never: always `unchecked` |

Non-finite coordinates are refused earlier, by the STL reader and by `FromSoup`.

**Supported topology:** one edge-connected, closed, edge- and vertex-manifold, consistently and outwardly wound
triangle mesh with no degenerate triangles. Through-holes (genus > 0) are accepted. More than one component,
including a shell nested inside another, is refused with `ErrUnsupportedInput`; any other failed check is
`ErrInvalidMesh`. Stored STL normals are never used; the reader only counts those that disagree with the winding,
and the report carries a warning when any do.

## 5. Orientation

Generation (`internal/orient/candidates.go`), measurement (`measure.go`) and ranking (`rank.go`) are separate
functions, so a different search strategy can replace generation alone.

### 5.1 Candidates

In this order, with later duplicates (every basis component equal within 1e-9) dropped:

1. The original orientation.
2. The 24 proper rotations whose basis vectors are signed coordinate axes.
3. Up to `max_planar_faces` (default 12) "face down" rotations. Triangle normals are grouped into clusters: a
   triangle joins the first cluster whose seed normal is within 1 degree of its own, or seeds a new one.
   Triangles are visited by decreasing area, ties broken by normal components, so the result does not depend on
   triangle order. Clusters are ranked by total area, never by triangle count. Each one yields the minimal
   rotation that turns its seed normal to -Z. At most 1024 clusters are tracked.

There is no separate yaw sampling. The axis-aligned set already contains the quarter-turn yaws of each
axis-aligned pose, and face-down candidates keep the yaw their minimal rotation gives. The candidate list is cut
to `max_candidates` (default 64); the report records whether that happened.

`--keep-orientation` evaluates only the original orientation.

### 5.2 Placement

Each rotated model is translated by

```
t = (-(minX + maxX) / 2, -(minY + maxY) / 2, -minZ)
```

from its rotated bounding box, so its lowest point is at Z = 0 and its bounding box is centered on the Z axis.

### 5.3 Metrics

Per placed candidate, in mm and mm²:

| Metric | Definition |
|---|---|
| `height_mm` | maximum Z |
| `bounding_footprint_mm` | X and Y extent of the bounding box (informational only) |
| `support_demand_area_mm2` | surface area of triangles classed as demand (section 6) |
| `support_demand_projected_area_mm2` | the same triangles' area projected onto the plate |
| `bed_contact_area_mm2` | area of downward triangles whose three vertices lie within `Plate` of Z = 0 |
| `centroid_over_contact_hull` | whether the volume centroid's XY projection lies in the convex hull of the bed-contact vertices; `null` when those vertices span no area |
| `contact_hull_margin_mm` | the smallest distance from that projection to a hull edge's line, negative outside |

The bed-contact area is the area of the faces that touch the plate, not the bounding-box footprint. The centroid
metric is a geometric heuristic, not a stability simulation, and is not part of the score. mtilt computes no
support-volume estimate for candidates; the report gives the volume of each support actually built.

### 5.4 Objective

```
score = w_demand  * projected_demand_area / surface_area
      + w_height  * height / size
      - w_contact * bed_contact_area / surface_area
```

`surface_area` is the model's total surface area and `size` is twice the largest distance from its volume
centroid to a vertex. Both are the same for every orientation, so each term is a dimensionless fraction and no
two units are added. Lower is better. Default weights are 1, 0.1 and 0.5. The report lists the raw metrics, each
weighted term, the weights and both normalizers.

### 5.5 Ranking and search

Candidates are sorted by: fits the build volume (or fit unchecked) before exceeds; then score rounded to 1e-9;
then candidate ID. Rounding makes tiny float differences from triangle order count as ties.

`Prepare` walks the ranking. A candidate whose model alone exceeds the build volume is skipped. Each of the first
`max_support_attempts` (default 8) remaining candidates gets a support attempt (section 7). The first attempt
whose supports cover all demand and pass every assembly check is selected; a candidate with no demand succeeds
with no supports. If none succeeds, `Prepare` returns `ErrNoFeasibleCandidate` with the report, including each
attempted candidate's failure reason and whether the attempt limit stopped the search.

## 6. Overhang convention

The overhang angle of a downward-facing triangle is the angle between its plane and the horizontal plane,
`acos(-n.z)` for unit normal n: a downward horizontal ceiling is 0 degrees, a vertical wall is 90 degrees. This
follows PrusaSlicer's documented support threshold convention.

`overhang.Classify` puts each triangle of a placed model in one class:

| Class | Condition |
|---|---|
| bed contact | normal points down and all three vertices are within `Plate` of Z = 0 |
| demand | normal points down, not bed contact, overhang angle below `overhang_threshold_deg` |
| none | everything else |

Bridges are not detected. A horizontal span between two walls is demand like any other ceiling.

## 7. Supports

### 7.1 Geometry class

mtilt supports demand surfaces that a vertical line from the build plate meets before any other part of the
model, with room below for a pillar and its clearance zones. Demand above other model geometry (occluded),
demand too close to the plate for the shortest pillar, and demand in gaps narrower than a pillar's clearance zone
are reported as uncovered, and the candidate fails. mtilt never routes a pillar through the model, never starts
one in mid-air, and never drops demand it cannot cover.

### 7.2 Pillar geometry

Each pillar (`internal/support/pillar.go`) is a closed mesh of axis-aligned square sections centered on (x, y):

| Part | Width | Height |
|---|---|---|
| base pad | `base_width_mm` | 0 to `base_thickness_mm` |
| shaft | `pillar_width_mm` | up to `tip_height_mm` below the top (omitted when shorter than 1e-6 mm) |
| tip | `pillar_width_mm` narrowing to `contact_width_mm` | `tip_height_mm` |

The top face is `top_contact_gap_mm` plus a numeric slack (4 x `Serialization`) below the lowest point of the
model over the `contact_width_mm` square. Every section is at least `min_feature_mm` wide because the profile
requires `min_feature_mm <= contact_width_mm <= pillar_width_mm <= base_width_mm`; a profile that breaks that
order is refused rather than producing smaller features.

### 7.3 Sampling and coverage

Demand is sampled at every point of a square grid of pitch `support_spacing_mm / 4`, anchored at the origin, that
falls inside a demand triangle's XY projection (at that triangle's height), plus every demand triangle's vertices.
Samples come from a fixed grid, so retriangulating a flat region adds or removes only vertex samples inside
already-covered area.

**Coverage rule:** a sample at (x, y, z) is covered by a pillar at (px, py) that holds a surface at height sz when

```
hypot(x - px, y - py) <= support_spacing_mm  and  |z - sz| <= support_spacing_mm * max(1, tan(threshold))
```

The height condition keeps a pillar under one surface from counting for another surface stacked above it.

### 7.4 Placement passes

Samples are visited in Y, X, Z order. "Nearby positions" of a sample are the points of a `support_spacing_mm / 16`
grid within `support_spacing_mm` of it, nearest first.

1. **Edge.** A sample whose own position the model rules out (occluded, too low, or a clearance collision;
   usually beside a wall) gets the first accepted nearby position that covers it. Running this first keeps grid
   pillars from taking the only room left beside a wall.
2. **Grid.** Every node of a `support_spacing_mm` grid anchored at the origin, inside the samples' bounding box.
3. **Fill.** Each sample still uncovered tries its own position, then its nearby positions.

A position is accepted when all of these hold:

- the vertical line through it meets a demand triangle first;
- the model's lowest point over the contact square leaves room for base plus tip plus the top gap;
- no clearance zone (7.5) touches the model;
- its base square is apart from every accepted base square.

### 7.5 Clearance zones

These solids must not touch any model triangle (separating-axis test of each convex zone against each nearby
triangle; touching counts):

- the base pad, widened on every side by `side_clearance_mm`;
- the shaft, widened the same way;
- the tip, swept upward by `top_contact_gap_mm`.

Lateral clearance is not applied within the tip's height, so the narrowing tip can approach a sloped surface it
holds. Because every pillar starts at Z = 0, where the model has no interior, a pillar that touches no model
triangle lies entirely outside the model.

### 7.6 Support-to-support

Bases must not touch or overlap. The profile requires `base_width_mm < support_spacing_mm`, so grid pillars never
collide. Every other pillar is placed only when its base square is apart from every accepted one. Each pillar lies
inside the vertical prism over its base, so separate bases mean separate bodies. mtilt never exports overlapping
primitives as one body.

### 7.7 Assembly validation

After construction, `RecheckPlan` re-runs the clearance zones and the coverage rule over the finished plan.
`ValidateAssembly` then checks the meshes alone:

| Check | Requirement |
|---|---|
| `support_topology` | each support passes every `mesh.Validate` check that mtilt implements |
| `support_on_plate` | each support's lowest Z is 0 |
| `support_model_contact` | no support triangle touches a model triangle |
| `support_top_gap` | the model's lowest point over each top face is at least the gap above it |
| `support_separation` | support XY bounding boxes are pairwise disjoint |

`assembly_build_volume_fit` checks model plus supports against the build volume, and `proper_rigid_transform`
checks the determinant. The CLI writes the files, reads them back, and runs `ValidateSerialized`: the model's
coordinates must match within `Serialization`, topology checks run again, and `ValidateAssembly` runs again on
the re-read meshes with `2 x Serialization` allowance.

### 7.8 What is not verified

- Removal: mtilt does not check that a support can be reached or broken away. Bed-rooted pillars are not
  evidence of easy removal.
- Physical behavior: no support has been printed, sliced or tested by mtilt. A slicer can change the effective
  gap, merge thin features, or add its own interfaces.
- Stability under print forces, bridges, and input self-intersection.

The report lists these under `unchecked`.

## 8. Output

The CLI writes an assembly directory (`model.stl`, `supports/support-NNNN.stl`, `plan.json`) in millimeters, all
in one frame. `supports` in `plan.json` is an empty list when no supports are needed; no placeholder file is
written. Supports are numbered by Y, then X, of their center.

`prepare` refuses an existing `--out`. It writes into a hidden staging directory beside `--out`, validates the
written files, writes `plan.json` last, checks again that `--out` does not exist, and renames the staging
directory to `--out`. On any failure it removes the staging directory. An interrupted run can leave a hidden
`.<name>.partial-*` directory, never a directory named `--out`.

`plan.json` (schema version 1) holds no timing. Two runs with the same input, options and build give
byte-identical reports. Different CPU architectures may differ in the last bits of floating-point values.

## 9. Limits

| Limit | Default | Exceeded |
|---|---|---|
| STL bytes | 512 MiB | `stl.ErrLimit` |
| STL triangles | 5,000,000 | `stl.ErrLimit` |
| `max_triangles` | 2,000,000 | `ErrLimit` |
| `max_candidates` | 64 | list cut, `candidate_limit_reached` |
| `max_planar_faces` | 12 | fewer face-down candidates |
| `max_support_attempts` | 8 | search stops, `support_attempt_limit_reached` |
| `max_supports` | 5,000 | attempt fails |
| `max_samples` | 500,000 | attempt fails |

A binary STL's declared triangle count never sizes an allocation until the file's length confirms it. Every long
loop checks its `context.Context`.

## 10. Package layout

| Path | Contents |
|---|---|
| `mtilt.go` | `Prepare`, `Analyze`, `ValidateSerialized`, the candidate loop |
| `options.go` | `Unit`, `Profile` and its validation, `Options`, `Limits`, weights, the embedded example profile |
| `result.go` | `Result`, `Report` and its parts, errors, transform reporting |
| `doc.go` | package documentation |
| `mesh/` | `Mesh`, `FromSoup`, measurements, `Tolerance`, `Validate` |
| `stl/` | bounded reader, binary writer |
| `internal/overhang/` | overhang angle and triangle classes |
| `internal/orient/` | candidate generation, placement, metrics, scoring, ranking |
| `internal/support/` | XY index, separating-axis tests, pillar geometry, construction, assembly checks |
| `internal/cli/` | argument parsing, file reading, staged output |
| `internal/fixture/` | builders for the synthetic test shapes and the files in `testdata/` |
| `cmd/mtilt/` | executable entry point |
| `profiles/example-fdm.json` | illustrative, uncalibrated profile |
| `examples/` | executable examples |
| `testdata/` | fixtures generated by `internal/fixture` |

## 11. Dependencies

| Module | Use | Reason |
|---|---|---|
| `github.com/lestrrat-3d/r3` | `Vec`, `Transform` | the lestrrat-3d vector and rigid-transform library. It rejects non-finite values, keeps rotations orthonormal, and reports reflections, which mtilt's rigid-transform rule needs. It pulls in `lestrrat-3d/units`. |
| `github.com/lestrrat-3d/units` | `internal/fixture` only | the angle value `r3.Rotation` takes, to rotate the oblique test cuboid |
| `github.com/stretchr/testify` | tests only | assertions |

`lestrrat-go/stl` was not used: it parses ASCII coordinates as float32, detects binary files by keyword instead of
by length, and has no byte or triangle limits. No other geometry library is used. No dependency needs cgo, a
network, Python, or a GPU.
