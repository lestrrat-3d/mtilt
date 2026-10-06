# mtilt design

This document states what mtilt implements, the geometric assumptions behind it, what it leaves unchecked, and
what it leaves for later. The code is the authority where the two disagree; fix whichever is wrong.

| Question | Section |
|---|---|
| What inputs are accepted? | 1, 4 |
| Which way is up, and how is the model moved? | 2 |
| Where do epsilons come from? | 3 |
| How is an orientation chosen? Why is strength first? | 5 |
| What counts as an overhang? | 6 |
| How are supports planned, built and checked? | 7 |
| What does Prepare add to the document? | 8 |
| What bounds the work? | 9 |
| Where does each piece of code live? | 10 |
| Why these dependencies? | 11 |

## 1. Scope

mtilt runs one pipeline on a decad body:

```
body -> check and tessellate -> evaluate orientations -> plan supports on the mesh
     -> build model copy and pillar bodies -> decad Verify + mesh re-checks -> report
```

Implemented:

- Input: one solid `*decad.Body` with one lump.
- Single-material FDM with removable supports.
- One support strategy: separate round pillars rooted on the build plate, each a decad body (section 7).

Not implemented, and not stubbed: file input or output (decad's exporters write files; STL import into decad is
planned in decad), G-code, printer control, a GUI, network services, GPU work, mesh repair, model splitting, tree
supports, supports rooted on the model, soluble or resin supports, bridge detection, load or layer-adhesion
simulation, slicer profiles. [roadmap.md](roadmap.md) orders them.

"Best" in mtilt always means best among the evaluated orientations under the rules in section 5 and the given
profile. mtilt does not claim a global optimum or printability.

## 2. Coordinates and the transform

- Coordinates are float64 millimeters, decad's convention, in a right-handed frame. +Z is the build direction.
  The build plate is the plane Z = 0.
- The model moves by one proper rigid transform: a rotation R (determinant +1) followed by a translation t. A
  point p of the input body lands at `q = R * p + t`.
- The report stores the transform as `matrix`: the 4x4 homogeneous matrix `[R t; 0 0 0 1]`, row-major, acting on
  column vectors `(x, y, z, 1)` in millimeters. `inverse` is `[R^T, -R^T t; 0 0 0 1]`.
- The output model is `body.PlacedCopy(transform)`. mtilt never mirrors, scales, deforms or remeshes the model,
  and never merges a support into it. The model and the supports are in the same frame.

## 3. Tolerances

Planning runs on decad's tessellation of the body (`mesh.FromBody`), requested with chord tolerance
`ChordToleranceMM` (default 0.01 mm) and decad's boundary proof. decad reports a **tessellation bound**: no point
of the true surface lies farther than that from the mesh. Planar faces triangulate exactly, so a body with only
planar faces has a bound of 0 up to rounding from its placements. mtilt widens the top gap and the side clearance
by the bound while planning (section 7.5), and widens the build-volume check by it.

`mesh.Tolerance` (`internal/mesh/tolerance.go`) is the only source of numeric epsilons. All three values scale
with the model: `scale` is the largest absolute coordinate of the mesh's bounding box, widened by one diagonal in
every direction so placed coordinates are covered too, and at least 1 mm.

| Field | Value | Used for |
|---|---|---|
| `Length` | 1e-9 x scale | equality of positions and heights; degenerate-triangle height; contact tests |
| `Serialization` | 2^-23 x scale (one float32 ulp) | the numeric slack under each pillar top (section 7.2) |
| `Plate` | 4 x 2^-23 x scale | deciding that a vertex lies on the build plate |

Manufacturing clearances (`top_contact_gap_mm`, `side_clearance_mm`, `plate_anchor_height_mm`) are profile values
and are never mixed with these.

## 4. Input acceptance

| Check | Fails with |
|---|---|
| `body.IsSolid()` | `ErrInvalidBody` |
| exactly one lump | `ErrUnsupportedInput` |
| decad volume, area and bounds readable | `ErrInvalidBody` |
| tessellation succeeds with the boundary proof | `ErrInvalidBody` |
| tessellation passes every `mesh.Validate` check (closed, manifold, consistent and outward winding, nondegenerate, one component) | `ErrInvalidBody`, or `ErrUnsupportedInput` for more than one component |
| tessellation within `max_triangles` | `ErrLimit` |

The tessellation checks are reported with a `tessellation_` prefix. `self_intersection` is always `unchecked`.

## 5. Orientation

**Strength comes first, then minimal support.** For a part with a long axis, no orientation that tilts the long
axis more than `MaxLongAxisTiltDeg` (default 15 degrees) from the plate is chosen, however little support it
needs. Inside that limit mtilt looks for the orientation with the least support.

An orientation is described, for support, by its **down direction** d: the input-frame unit vector that ends up
facing the plate. Turning the part about the vertical axis afterwards changes no overhang and no support, so the
search runs over directions on the sphere. Each direction's rotation is the minimal one that turns d to -Z.

The long axis (`internal/orient/principal.go`), the estimate and the search (`search.go`, `candidates.go`),
measurement (`measure.go`) and ranking (`rank.go`) are separate.

### 5.1 The long axis

The inertia tensor of the tessellated solid (unit density, about its centroid; `Mesh.Inertia`) is diagonalized by
Jacobi rotations. Its principal axes, sorted by ascending moment, give the **long axis** (smallest moment) and the
**elongation** `1 - I1 / I2`: 0 for a cube, a sphere or a square plate, and about 0.99 for an 8 x 8 x 100 mm stick.
The tilt limit applies when the elongation is at least 0.1. The long axis's **elevation** in direction d is
`asin(|a . d|)` for long axis a: 0 lying flat, 90 standing up.

The limit encodes one rule of FDM printing: a part is weakest across its layer lines, so a slender part standing
up breaks between layers under a bending load. mtilt runs no load or layer-adhesion analysis; the report lists
`layer_strength` as unchecked.

### 5.2 Support cost

Support cost has two parts, each made dimensionless before they are added:

```
cost = w_volume * support_volume / (surface_area * size) + w_contact * support_contact / surface_area
```

`surface_area` is the mesh's surface area and `size` twice the largest distance from its centroid to a vertex;
both are the same for every orientation. Default weights are 1 and 1 (`CostWeights`). Lower is better.

The **estimate** (`Estimator.Estimate`) takes one pass over the triangles. For direction d, a triangle needs
support when it faces down at an overhang angle below the threshold (`n . d > cos(threshold)`) and is neither on
the plate nor entirely within the plate-anchor height. Each such triangle adds its projected area `A (n . d)` to
the contact estimate, and that area times its mean height above the plate to the volume estimate: the column under
it down to the plate. A triangle with any vertex lower than the shortest pillar can reach (`MinHeight + gap`, plus
the tessellation bound) adds to `too_low_mm2`; no pillar fits there. The column ignores anything of the model
between the triangle and the plate, so occluded demand is found only by planning.

The **planned cost** is measured on the pillars the planner places: their summed analytic volume and their summed
round contact areas, priced with the same formula.

### 5.3 First layer

The **first-layer area** of direction d is the model's cross-section one layer height above its lowest point
(`Mesh.SectionArea`): what the first layer prints, and so what holds the part to the plate. A direction under the
profile's `min_first_layer_area_mm2` (example: 20 mm²) is not allowed. Without this floor, a stick balanced on an
edge, with every face at exactly 45 degrees, needs no support and wins while touching the plate along a line.

### 5.4 Search

1. **Seeds:** the given orientation (d = -Z), the six coordinate directions, and the normals of the
   `max_planar_faces` (default 12) largest planar face clusters, so a flat face can sit exactly on the plate.
   Clusters group triangle normals within 1 degree of a seed normal, visiting triangles by decreasing area (ties by
   normal components), and rank by total area. At most 1024 clusters are tracked.
2. **Sweep:** `sweep_directions` (default 2000) directions spread evenly over the sphere (a Fibonacci lattice),
   about 4.5 degrees apart.
3. **Filter and rank:** a seed or sweep direction is kept when it is within the tilt limit, meets the first-layer
   floor, and has no `too_low_mm2`. Kept directions are ranked by estimated cost (rounded to 1e-9), then by lower
   elevation; the best `max_support_attempts` (default 8) that are at least 1 degree apart become finalists.
4. **Refine:** each finalist tries 8 directions around it at the sweep spacing; it moves to the best one that
   passes the same filter and is strictly cheaper, and the step halves whenever none is, down to 0.25 degrees. A
   refined finalist is never worse than where it started.
5. **Plan:** every finalist gets a support plan (section 7). Among the plans that succeed, the lowest planned cost
   wins; ties go to the larger first-layer area, then the lower height, then the lower elevation, then the lower ID.

The result is the best orientation found at this resolution, not a proven global optimum. The report lists the
given orientation and every finalist with its estimate, planned cost, metrics and outcome. `KeepOrientation`
skips the search and plans only the given orientation; the tilt limit does not apply to it.

### 5.5 Placement and metrics

Each rotated mesh is translated by `t = (-(minX + maxX) / 2, -(minY + maxY) / 2, -minZ)` from its rotated bounding
box. Per candidate, in mm, mm² and degrees:

| Metric | Definition |
|---|---|
| `height_mm` | maximum Z |
| `bounding_footprint_mm` | X and Y extent of the bounding box (informational only) |
| `first_layer_area_mm2` | cross-section at Z = layer height |
| `support_demand_area_mm2` | surface area of triangles classed as demand (section 6) |
| `support_demand_projected_area_mm2` | the same triangles' area projected onto the plate |
| `anchored_area_mm2` | area of overhang triangles within the plate-anchor height |
| `bed_contact_area_mm2` | area of downward triangles whose three vertices lie within `Plate` of Z = 0 |
| `centroid_over_contact_hull`, `contact_hull_margin_mm` | whether the centroid's XY projection lies in the convex hull of the bed-contact vertices, and its distance to the nearest hull edge line; `null` when those vertices span no area |
| `long_axis_elevation_deg` | the long axis's elevation |

## 6. Overhang convention

The overhang angle of a downward-facing triangle is the angle between its plane and the horizontal plane,
`acos(-n.z)` for unit normal n: a downward horizontal ceiling is 0 degrees, a vertical wall is 90 degrees. This
follows PrusaSlicer's documented support threshold convention.

| Class | Condition |
|---|---|
| bed contact | normal points down and all three vertices are within `Plate` of Z = 0 |
| anchored | normal points down, overhang angle below the threshold, all three vertices at or below `plate_anchor_height_mm` |
| demand | normal points down, overhang angle below the threshold, not bed contact, not anchored |
| none | everything else |

Support sampling also drops demand samples at or below `plate_anchor_height_mm`. The anchor exists because the
shortest pillar (base plus tip plus gap, 2.8 mm in the example profile) cannot fit under a surface just above the
plate, such as the underside of a rod lying on its side, which the first layers print from the plate. The example
profile's 1.2 mm is a modelling assumption, not a measured value. Bridges are not detected: a horizontal span
between two walls is demand like any other ceiling.

## 7. Supports

### 7.1 Geometry class

mtilt supports demand surfaces that a vertical line from the build plate meets before any other part of the
model, with room below for a pillar and its clearance zones. Demand above other model geometry (occluded), demand
too close to the plate for the shortest pillar, and demand in gaps narrower than a pillar's clearance zone are
reported as uncovered, and the candidate fails. mtilt never routes a pillar through the model, never starts one in
mid-air, and never drops demand it cannot cover.

### 7.2 Pillar geometry

Each pillar is one decad body: a stepped outline revolved a full turn about the pillar's vertical axis
(`support.Params.Body`). From the plate up:

| Part | Diameter | Height |
|---|---|---|
| base disc | `base_width_mm` | 0 to `base_thickness_mm` |
| shaft | `pillar_width_mm` | up to `tip_height_mm` below the top (left out when shorter than 1e-6 mm) |
| tip | `pillar_width_mm` narrowing to `contact_width_mm` | `tip_height_mm` |

The top face is `top_contact_gap_mm`, plus the tessellation bound, plus a numeric slack (4 x `Serialization`),
below the lowest point of the meshed model over the square enclosing the contact. The profile requires
`min_feature_mm <= contact_width_mm <= pillar_width_mm <= base_width_mm`, so every section is at least
`min_feature_mm` across; a profile that breaks the order is refused.

Planning works with the square that encloses each round section, so every check made on the squares holds for the
round pillar.

### 7.3 Sampling and coverage

Demand is sampled at every point of a square grid of pitch `support_spacing_mm / 4`, anchored at the origin, that
falls inside a demand triangle's XY projection (at that triangle's height), plus every demand triangle's vertices,
leaving out samples at or below the anchor height.

**Coverage rule:** a sample at (x, y, z) is covered by a pillar at (px, py) that holds a surface at height sz when

```
hypot(x - px, y - py) <= support_spacing_mm  and  |z - sz| <= support_spacing_mm * max(1, tan(threshold))
```

### 7.4 Placement passes

Samples are visited in Y, X, Z order. "Nearby positions" of a sample are the points of a `support_spacing_mm / 16`
grid within `support_spacing_mm` of it, nearest first.

1. **Edge.** A sample whose own position the model rules out (occluded, too low, or a clearance collision;
   usually beside a wall) gets the first accepted nearby position that covers it.
2. **Grid.** Every node of a `support_spacing_mm` grid anchored at the origin, inside the samples' bounding box.
3. **Fill.** Each sample still uncovered tries its own position, then its nearby positions.

A position is accepted when the vertical line through it meets a demand triangle first; the model's lowest point
over the contact square leaves room for base, tip and gap; no clearance zone touches the model; and its base square
is apart from every accepted base square.

### 7.5 Clearance zones

These solids must not touch any mesh triangle (separating-axis test; touching counts):

- the base, widened on every side by `side_clearance_mm` plus the tessellation bound;
- the shaft, widened the same way;
- the tip, swept upward by `top_contact_gap_mm` plus the tessellation bound.

Lateral clearance is not applied within the tip's height, so the narrowing tip can approach a sloped surface it
holds. Because every pillar starts at Z = 0, where the model has no interior, a pillar that touches no model
triangle lies entirely outside the model.

### 7.6 Support-to-support

Bases must not touch or overlap. The profile requires `base_width_mm < support_spacing_mm`, so grid pillars never
collide; every other pillar is placed only when its base square is apart from every accepted one.

### 7.7 Verification

On the plan, before any body is built: `RecheckPlan` re-runs the clearance zones and the coverage rule, and the
model plus pillar footprints are checked against the build volume.

After the bodies are built, for the selected candidate only:

| Check | Requirement |
|---|---|
| `decad_body_validity` | decad's `Verify` reports every assembly body (model and pillars) as a valid solid |
| `decad_interference` | `Verify` proves no two assembly bodies overlap |
| `tessellated_support_topology` | each pillar's tessellation passes `mesh.Validate` |
| `tessellated_support_on_plate` | each pillar's lowest Z is 0 |
| `tessellated_support_model_contact` | no pillar triangle touches a model triangle |
| `tessellated_support_top_gap` | the model's lowest point over each top face is at least the gap above it |
| `tessellated_support_separation` | pillar XY bounding boxes are pairwise disjoint |
| `proper_rigid_transform` | the transform does not mirror |

The tessellated checks allow the model's and the pillars' tessellation bounds. `Verify` runs over the whole
document; mtilt keeps only the rows for the assembly's own bodies. It runs without `WithClearances`, which
measured about 9.7 s against 0.33 s for a bracket and 35 pillars, because it measures every pair; the gaps and
clearances come from the mesh checks above.

### 7.8 What is not verified

- Removal: mtilt does not check that a support can be reached or broken away.
- Physical behavior: no support has been printed, sliced or tested by mtilt.
- Stability under print forces, layer strength under load, bridges, and self-intersection.

The report lists these under `unchecked`.

## 8. Result and the document

`Prepare` returns the moved model, the support bodies (numbered by Y, then X, of their center), the transform and
the report. The input body stays live and unchanged. decad has no operation that removes a body from a document,
so mtilt adds bodies only after a plan has passed every planning check:

- `ErrNoFeasibleCandidate`, `ErrInvalidBody`, `ErrUnsupportedInput`, `ErrLimit`: nothing is added.
- `ErrAssembly`: the model copy and pillars were built and failed decad's verification. They stay live in the
  document, and `FailureError.Result` holds them.

The report holds no timing. Two runs with the same body, options and build give byte-identical reports.

## 9. Limits

| Limit | Default | Exceeded |
|---|---|---|
| `max_triangles` | 2,000,000 | `ErrLimit` |
| `sweep_directions` | 2,000 | the sweep's resolution |
| `max_planar_faces` | 12 | fewer face-down seeds |
| `max_support_attempts` | 8 | the number of finalists planned |
| `max_supports` | 5,000 | attempt fails |
| `max_samples` | 500,000 | attempt fails |

Every long loop checks its `context.Context`. decad's `Verify` time grows with the number of bodies in the
document. As a dated measurement (2026-10-06, 24-core machine), `Prepare` took about 4 s for the bracket's 41
pillars, and about 5 s for the nail fixture, which plans 8 finalists and builds 49 pillars; the estimate sweep
itself took milliseconds.

## 10. Package layout

| Path | Contents |
|---|---|
| `mtilt.go` | `Prepare`, `Analyze`, the candidate loop, body building and verification |
| `options.go` | `Profile` and its validation, `Options`, `Limits`, weights, the embedded example profile |
| `result.go` | `Result`, `Report` and its parts, errors, transform reporting |
| `doc.go` | package documentation |
| `internal/mesh/` | `Mesh`, `FromBody` (decad tessellation), measurements, inertia, `Tolerance`, `Validate` |
| `internal/overhang/` | overhang angle and triangle classes |
| `internal/orient/` | long axis, support-cost estimate, direction search, placement, metrics, ranking |
| `internal/support/` | XY index, separating-axis tests, pillar geometry and bodies, planning, plan and mesh checks |
| `internal/fixture/` | test shapes as triangle soups and as decad bodies |
| `profiles/example-fdm.json` | illustrative, uncalibrated profile |
| `examples/` | executable examples |
| `_gallery/` | the README images: a separate module (solidlens, golang.org/x/image) that prepares fixtures with mtilt and renders them |

## 11. Dependencies

| Module | Use | Reason |
|---|---|---|
| `github.com/lestrrat-3d/decad` | input bodies, tessellation, pillar bodies, `Verify`, `export` in examples | the lestrrat-3d CAD engine mtilt prepares bodies from |
| `github.com/lestrrat-3d/sketch` | pillar outlines and fixture profiles | decad builds bodies from sketch profiles |
| `github.com/lestrrat-3d/r3` | `Vec`, `Transform` | the lestrrat-3d vector and rigid-transform library; it rejects non-finite values, keeps rotations orthonormal, and reports reflections |
| `github.com/lestrrat-3d/units` | lengths passed to and read from decad | decad's quantities are `units.Value` |
| `github.com/stretchr/testify` | tests only | assertions |

`_gallery/` is its own module, so its renderer (`github.com/lestrrat-3d/solidlens`) and font
(`golang.org/x/image`) never enter the library's `go.mod`. No dependency needs cgo, a network, Python, or a GPU.
