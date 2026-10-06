# mtilt validation

This page lists what is checked automatically, how to check an assembly by hand, and the record of manual checks
actually performed.

## Automated checks

`go test ./...` runs all of these. CI runs them on Linux (with the race detector), macOS and Windows.

| Area | Where | What a test asserts |
|---|---|---|
| STL reading | `stl/stl_test.go` | ASCII float64 precision; binary with a `solid` header read as binary; truncated, malformed, non-finite, oversized and multi-solid input refused; triangle and byte limits; stored normals that disagree are counted |
| STL fuzzing | `stl.FuzzDecode` | no panic or NaN on arbitrary bytes; seeds are every file in `testdata/` |
| Mesh validation | `mesh/validate_test.go` | each check fails on a mesh built to break it; exact-only vertex merging; volume, area and centroid of known shapes; rigid motion keeps volume and area and inverts exactly |
| Overhang angle | `internal/overhang` | ceiling 0 degrees, wall 90 degrees, threshold on downward faces only, faces on the plate excluded |
| Orientation | `internal/orient` | 24 distinct proper axis rotations; face-down rotations map the face normal to -Z; candidate limit; placement; bed contact is real contact area, not the bounding box; ranking tie-breaks; triangle order does not change candidates |
| Supports | `internal/support` | pillar meshes are closed with the analytic volume; bracket and bridge demand fully covered; clearance zones and coverage re-checked; bases apart; occluded, too-low and narrow-slot demand reported with reasons; triangle order and retriangulation do not change pillars; support and sample limits; cancellation |
| Assembly checks | `internal/support` | a pillar pushed into the model, a pillar inside the gap, overlapping pillars, a floating pillar and an open pillar mesh each fail their check |
| Pipeline | `prepare_test.go` | cube needs no supports; oblique cuboid turns onto its 800 mm² face; fixed bracket and bridge get validated supports; occluded part fails with `ErrNoFeasibleCandidate`; output model equals input under the reported transform; unit conversion; build-volume fit and overflow; limits; determinism; transform matrix and inverse |
| Analysis fuzzing | `mtilt.FuzzAnalyze` | no panic or NaN score on small arbitrary meshes |
| CLI | `internal/cli/cli_test.go` | the built binary's exit codes, output layout, `plan.json` file list, read-back validation, overwrite refusal, nothing written on failure, input unchanged |
| Examples | `examples/` | printed output matches |
| Fixtures | `internal/fixture` | `testdata/` matches the builders byte for byte |

None of these tests slices or prints anything.

## Manual check of an assembly in a slicer

Record every run in the log below, including failures. Steps:

1. `mtilt prepare --units mm --profile <profile> --keep-orientation --out bracket-result testdata/bracket.stl`
2. Note the slicer name and exact version.
3. Import `bracket-result/model.stl` and every `bracket-result/supports/*.stl` as parts of one object, so their
   relative positions are kept.
4. Confirm the pillars stand under the arm, and that the slicer did not move, center or drop any part.
5. Turn off the slicer's own supports for the object.
6. Slice. In the layer preview, confirm:
   - each pillar's top layer stops below the arm with a visible gap;
   - the pillar tips print as solid features and are not dropped as too thin;
   - the bases print as separate pads and do not merge with the post.
7. Record which of these held, with screenshots if possible.

## Support-removal calibration print

Use only after the slicer check above passes for the same slicer and profile.

1. Print `bracket-result` with the intended printer, material and slicer settings.
2. Record printer, nozzle, material, slicer and version, layer height, and the profile's
   `top_contact_gap_mm`, `side_clearance_mm` and `contact_width_mm`.
3. Remove the supports by hand. Record whether each came off without tools, with pliers, or not at all, and any
   damage to the arm's underside.
4. Repeat with `top_contact_gap_mm` one layer smaller and one layer larger.
5. Record the results below. A profile may set `"calibrated": true` only after this procedure passed with it.

## Log of manual checks performed

| Date | Who | mtilt version | Slicer / printer / material | Procedure | Result |
|---|---|---|---|---|---|

No manual slicer check or print has been performed yet.
