# mtilt roadmap

Milestones after the decad-input rewrite, in the intended order. None of them exists in the code yet. Each one
ships with tests that check a computed result, and with updates to [design.md](design.md).

## 1. Print-validated defaults

Run the calibration procedure in [validation.md](validation.md) on at least one printer, material and slicer.
Record the results there, and either calibrate `profiles/example-fdm.json` (including `plate_anchor_height_mm`
and the strength weight) or add a calibrated profile beside it with `"calibrated": true`.

## 2. Assembly export

Write the model and supports as separate objects in one 3MF package through decad's exporter. The 3MF core
specification lets a consumer ignore or replace objects typed as `support`, so test each target slicer before
choosing between `support` and `model` object types.

## 3. Better orientation search

Make the support estimate see occlusion (cast the column under each overhang against the model) so fewer
finalists fail planning, and turn the part about the vertical axis to fit rectangular build volumes. Both change
`internal/orient/search.go` only.

## 4. Removal accessibility

Check that each support can be reached from outside the part's convex hull, and report trapped supports.

## 5. Tree supports: merged trunks

Branching supports that share trunks. This needs milestones 5 and 6 first.

## Out of scope until requested

Supports rooted on the model (decided against: supports stand on the build plate only), G-code generation, printer control, a GUI, network services, GPU acceleration, automatic mesh repair, model
splitting, load or layer-adhesion simulation, soluble or resin supports, and a slicer-profile database.
