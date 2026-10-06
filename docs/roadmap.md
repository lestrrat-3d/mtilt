# mtilt roadmap

Milestones after the bootstrap, in the intended order. None of them exists in the code yet. Each one ships with
tests that check a computed result, and with updates to [design.md](design.md).

## 1. Print-validated defaults

Run the calibration procedure in [validation.md](validation.md) on at least one printer, material and slicer.
Record the results there, and either calibrate `profiles/example-fdm.json` or add a calibrated profile beside
it with `"calibrated": true`. Until then every profile is uncalibrated.

## 2. Multi-object 3MF export

Write the model and supports as separate mesh objects in one 3MF package, with millimeter units. The 3MF core
specification lets a consumer ignore or replace objects typed as `support`, so the object type alone does not
guarantee a slicer keeps them. Before choosing between typing supports as `support` or as `model` objects, import
test files into each target slicer and record the outcome in validation.md.

## 3. Bridge analysis

Detect downward spans anchored at both ends and treat short ones as printable bridges instead of support demand.
Until then every such span is demand, which over-supports but never under-supports.

## 4. Better candidate search

Add yaw sampling for rectangular build volumes and local refinement around the best candidates. Generation is
already separate from measurement and ranking, so this changes `internal/orient/candidates.go` only.

## 5. Removal accessibility

Check that each support can be reached from outside the part's convex hull, and report trapped supports. Today
removal is listed as unchecked.

## 6. Supports rooted on the model

Allow pillars that stand on an upward-facing model surface, with their own contact gap at the base. This would
cover the occluded case that fails today.

## 7. Tree supports and support optimization

Branching supports that share trunks. This needs milestones 5 and 6 first.

## Out of scope until requested

G-code generation, printer control, a GUI, network services, GPU acceleration, automatic mesh repair, model
splitting, mechanical simulation, soluble or resin supports, and a slicer-profile database.
