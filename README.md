# mtilt

Automatic orientation and breakaway support generation for FDM printing of [decad](https://github.com/lestrrat-3d/decad)
bodies.

mtilt (mesh-tilt) takes a solid `*decad.Body`, picks an orientation for printing on a single-material FDM printer,
and builds separate support pillars as decad bodies in the same document: round pillars standing on the build
plate under the part's overhangs. You export the moved model and the supports with decad's exporters and import
them into a slicer.

> **Prototype.** The API, the report layout and the algorithms will change. Every support layout mtilt builds has
> passed mtilt's and decad's geometric checks and nothing else. No support from mtilt has been printed, sliced or
> tested for breakaway behavior. The example profile is illustrative and uncalibrated.

## What it does now

- Takes one connected solid decad body (millimeters, +Z up, plate at Z = 0). It refuses a body that is not a
  solid or has more than one lump.
- Tessellates the body with decad (chord tolerance 0.01 mm by default) and plans on that mesh. The planning adds
  decad's proven tessellation bound to every gap and clearance it checks.
- Evaluates candidate orientations: the given one, the 24 axis-aligned rotations, up to 12 rotations that put a
  large flat face down, and, for a long part, rotations that lay its long axis flat, plus versions tilted 5, 10, 20
  and 30 degrees.
- Ranks them by a weighted score of support demand, build height, bed contact and layer strength. The strength
  term penalizes a long part whose long axis points up, because FDM parts are weakest across layer lines: a stick
  standing up puts every layer line across its length.
- Plans supports for the best-ranked candidates in turn until one gets a complete plan. Each pillar is a base
  disc, a shaft and a tip that narrows to a small contact, and stops `top_contact_gap_mm` below the surface it
  holds.
- For the selected candidate only, adds the moved model (`PlacedCopy`; the input stays live) and one revolved
  pillar body per support to the input's document. It then runs decad's `Verify`, requires that none of these
  bodies interfere and that each is a valid solid, and re-checks the pillars on their tessellations.
- Fails with a reason, and adds no bodies, when no candidate can be supported. An overhang above another part of
  the model is one such case, because no pillar rooted on the plate can reach it.

The full scope, algorithms and limits are in [docs/design.md](docs/design.md). Planned work is in
[docs/roadmap.md](docs/roadmap.md).

## Usage

```
go get github.com/lestrrat-3d/mtilt
```

mtilt needs Go 1.26.8 or later, and no C compiler.

The executable examples are the usage documentation; `go test ./examples/` runs them and checks their output.

- [`examples/mtilt_prepare_example_test.go`](examples/mtilt_prepare_example_test.go) builds a Γ-shaped bracket,
  keeps it upright so its arm needs 41 pillars, and writes the model and every pillar as STL with
  `decad/export`.
- [`examples/mtilt_strength_example_test.go`](examples/mtilt_strength_example_test.go) gives mtilt a round rod
  standing on its end and shows it laid down.

`mtilt.Prepare` returns a `Result` with the moved model, the support bodies, the rigid transform, and a `Report`
that encodes to JSON. The report records the input's readings, the transform and its inverse, the effective
profile, every candidate's metrics, score terms and support outcome, the selected candidate, each support's
position, every validation check, the properties mtilt did not check, warnings, and whether a search limit was
reached. `mtilt.Analyze` runs the same ranking without building supports or adding bodies.

## Importing into a slicer

mtilt has not been tested with any slicer. To keep the parts where mtilt put them:

1. Export the model and every support (for example with `decad/export.STL` or `decad/export.ThreeMF`) and import
   them together, as parts of one object, so the slicer keeps their relative positions instead of centering or
   dropping each file on the plate by itself. How to do that differs between slicers.
2. Check that the pillars sit under the overhangs, with a visible gap between each pillar's top and the part.
3. Turn off the slicer's own support generation for this object.
4. Inspect the sliced preview layer by layer: the top gap, the pillar tips and the bases.

[docs/validation.md](docs/validation.md) has the full procedure, including a small calibration print.

## Development

```
go test ./...                          # unit, pipeline and example tests
go vet ./...
golangci-lint run                      # v2.12.2, config in .golangci.yml
```

## License

[PolyForm Noncommercial 1.0.0](LICENSE).
