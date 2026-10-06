# mtilt roadmap

Milestones after the decad-input rewrite, in the intended order. None of them exists in the code yet. Each one
ships with tests that check a computed result, and with updates to [design.md](design.md).

## 1. Print-validated defaults

Run the calibration procedure in [validation.md](validation.md) on at least one printer, material and slicer.
Record the results there, and either calibrate `profiles/example-fdm.json` (including `plate_anchor_height_mm`
and the strength weight) or add a calibrated profile beside it with `"calibrated": true`.

## 2. STL input through decad

Mesh import belongs in decad. Once decad can turn an STL file into a body, mtilt accepts such bodies through the
same `Prepare` call.

## 3. Assembly export

Write the model and supports as separate objects in one 3MF package through decad's exporter. The 3MF core
specification lets a consumer ignore or replace objects typed as `support`, so test each target slicer before
choosing between `support` and `model` object types.

## 4. Bridge analysis

Detect downward spans anchored at both ends and treat short ones as printable bridges instead of support demand.

## 5. Better orientation search

Make the support estimate see occlusion (cast the column under each overhang against the model) so fewer
finalists fail planning, and turn the part about the vertical axis to fit rectangular build volumes. Both change
`internal/orient/search.go` only.

## 6. Removal accessibility

Check that each support can be reached from outside the part's convex hull, and report trapped supports.

## 7. Supports rooted on the model

Allow pillars that stand on an upward-facing model surface, with their own contact gap at the base. This would
cover the occluded case that fails today.

## 8. Tree supports and support optimization

Branching supports that share trunks. This needs milestones 6 and 7 first.

## Out of scope until requested

G-code generation, printer control, a GUI, network services, GPU acceleration, automatic mesh repair, model
splitting, load or layer-adhesion simulation, soluble or resin supports, and a slicer-profile database.
