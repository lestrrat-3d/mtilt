# mtilt validation

This page lists what is checked automatically, how to check an assembly by hand, and the record of manual checks
actually performed.

## Automated checks

`go test ./...` runs all of these. CI runs them on Linux (with the race detector), macOS and Windows.

| Area | Where | What a test asserts |
|---|---|---|
| Mesh checks | `internal/mesh/validate_test.go` | each check fails on a mesh built to break it; exact-only vertex merging; volume, area, centroid, inertia and cross-section area of known shapes; rigid motion keeps them and inverts exactly |
| Overhang classes | `internal/overhang` | ceiling 0 degrees, wall 90 degrees, threshold on downward faces only, plate contact and plate-anchor height |
| Long axis | `internal/orient` | sorted principal axes of known boxes; a cube has no long axis; rotated tensors give rotated axes; `FuzzPrincipal` checks A v = lambda v on arbitrary tensors |
| Orientation | `internal/orient` | support-cost estimate of known shapes (column volume, contact, too-low area) and its independence from the frame; the search keeps finalists within the tilt limit, sorted and at least 1 degree apart, turns the oblique cuboid's face down, and does not depend on triangle order; placement; real bed-contact area; ranking tie-breaks |
| Supports | `internal/support` | pillar bodies are decad solids with the analytic volume and closed tessellations; bracket and bridge demand fully covered; clearance zones and coverage re-checked; bases apart; occluded, too-low and narrow-slot demand reported with reasons; triangle order and retriangulation do not change pillars; limits; cancellation |
| Bridges | `internal/support` | a 3 mm slot is bridged and its wall tops held, and fails without bridges; a 30 mm span is not a 10 mm bridge but is a 35 mm one; a cantilever is never a bridge; a ceiling sloped over the bridge tilt limit is not a bridge |
| Branches | `internal/support` | the occluded part's overhang is covered by branches whose feet are off the base and whose leans stay within 40 degrees; no branches when the lean limit is 0; a branch body is one solid with a clean tessellation and close to its frustum volume |
| Assembly checks | `internal/support` | a pillar pushed into the model, a pillar inside the gap, overlapping pillars, a floating pillar and an open pillar mesh each fail their check |
| Pipeline | `prepare_test.go` | cube needs no supports; oblique cuboid turns onto its 800 mm² face; fixed bracket and bridge get pillar bodies that pass decad's interference and validity checks; occluded part gets branches with feet beside the base, and fails without branches, adding no bodies; a standing stick and rod are laid down; the nail is tilted under the 15 degree limit to its cheapest planned supports and stands when the limit is off; every finalist meets the tilt limit and the first-layer floor; input errors; build volume; limits; determinism; transform matrix and inverse; Analyze adds no bodies |
| Profile | `options_test.go` | the example profile is valid and uncalibrated; missing and unknown fields and each broken rule are refused |
| Examples | `examples/` | printed output matches |

None of these tests slices or prints anything.

## Manual check of an assembly in a slicer

Record every run in the log below, including failures. Steps:

1. Run `Example_mtilt_prepare` with its temporary directory replaced by a kept one, or write the same program:
   prepare the bracket with `KeepOrientation` and export the model and every support with `decad/export.STL`.
2. Note the slicer name and exact version.
3. Import the model and every support file as parts of one object, so their relative positions are kept.
4. Confirm the pillars stand under the arm, and that the slicer did not move, center or drop any part.
5. Turn off the slicer's own supports for the object.
6. Slice. In the layer preview, confirm:
   - each pillar's top layer stops below the arm with a visible gap;
   - the pillar tips print as solid features and are not dropped as too thin;
   - the bases print as separate pads and do not merge with the post.
7. Record which of these held, with screenshots if possible.

## Support-removal calibration print

Use only after the slicer check above passes for the same slicer and profile.

1. Print the exported bracket assembly with the intended printer, material and slicer settings.
2. Record printer, nozzle, material, slicer and version, layer height, and the profile's
   `top_contact_gap_mm`, `side_clearance_mm` and `contact_width_mm`.
3. Remove the supports by hand. Record whether each came off without tools, with pliers, or not at all, and any
   damage to the arm's underside.
4. Repeat with `top_contact_gap_mm` one layer smaller and one layer larger.
5. To check `max_bridge_mm`, print the slot fixture (`fixture.Bodies.Slot`) upright with no supports and widen the
   slot until its ceiling sags; record the widest clean span.
6. To check `plate_anchor_height_mm`, print the round rod from `Example_mtilt_strength` lying down without
   supports, and record whether its underside printed cleanly.
7. Record the results below. A profile may set `"calibrated": true` only after this procedure passed with it.

## Log of manual checks performed

| Date | Who | mtilt version | Slicer / printer / material | Procedure | Result |
|---|---|---|---|---|---|

No manual slicer check or print has been performed yet.
