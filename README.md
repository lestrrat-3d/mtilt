# mtilt

Automatic mesh orientation and breakaway support generation for FDM printing.

mtilt (mesh-tilt) reads a triangle mesh, picks an orientation for printing on a single-material FDM printer, and
builds separate support bodies: closed meshes standing on the build plate under the part's overhangs. It writes the
oriented model and each support as its own STL file, plus a JSON report, for you to import into a slicer.

> **Prototype.** The library API, the report layout and the algorithms will change. Every support layout mtilt
> emits has passed mtilt's own geometric checks and nothing else. No support from mtilt has been printed, sliced
> or tested for breakaway behavior. The example profile is illustrative and uncalibrated.

## What it does now

- Reads ASCII and binary STL. The file's length decides the format, so a binary file whose header starts with
  `solid` is read as binary. Coordinates are float64. NaN and infinite values, truncated files and files over the
  size limits are errors.
- Requires the input unit (`mm`, `cm`, `m` or `in`) and converts to millimeters once. It never guesses a scale
  from the model's size.
- Validates the mesh: triangle count, degenerate triangles, open edges, edges shared by more than two triangles,
  vertices where separate fans touch, inconsistent winding, inward winding, and more than one connected
  component. It does not test for self-intersection and reports that check as `unchecked`. It never repairs or
  welds a mesh. A mesh failing any check is refused.
- Evaluates candidate orientations: the input orientation, the 24 axis-aligned rotations, and up to 12 rotations
  that put a large flat face down. It measures each one (support demand area, real bed-contact area, build height)
  and ranks them by a weighted score. The report lists every metric, term and weight.
- Builds supports for the best-ranked candidates in turn until one gets a complete, validated set. A support is a
  square pillar: a base pad, a shaft, and a tip that narrows to a small contact square. Each pillar stands on the
  plate and stops `top_contact_gap_mm` below the surface it holds.
- Fails with a reason, and writes nothing, when no candidate can be supported. An overhang above another part of
  the model is one such case, because no pillar rooted on the plate can reach it.

The full scope, algorithms and limits are in [docs/design.md](docs/design.md). Planned work is in
[docs/roadmap.md](docs/roadmap.md).

## Install

mtilt needs Go 1.26.8 or later, and no C compiler.

```
go install github.com/lestrrat-3d/mtilt/cmd/mtilt@latest
```

Or build from a checkout with `go build -o mtilt ./cmd/mtilt`.

## Command line

Every command below runs from a checkout on the fixtures in `testdata/`; `internal/cli/cli_test.go` runs the same
commands on the built binary.

Inspect a mesh and rank orientations without building supports:

```
mtilt analyze --units mm testdata/oblique-cuboid.stl
mtilt analyze --units mm --json testdata/oblique-cuboid.stl
```

`analyze` prints the validation checks, the model's size, volume and surface area, and the candidates best
first. Without `--profile` it takes the overhang threshold from the built-in example profile and says so on
stderr.

Prepare a model, letting mtilt choose the orientation:

```
mtilt prepare --units mm --profile profiles/example-fdm.json --out result testdata/oblique-cuboid.stl
```

The cuboid is rotated onto its largest face and needs no supports, so `result/supports/` is empty.

Prepare a model in the orientation it was given:

```
mtilt prepare --units mm --profile profiles/example-fdm.json --keep-orientation --out fixed-result testdata/bracket.stl
```

The bracket's arm hangs 30 mm above the plate, and `fixed-result/supports/` holds 41 pillars under it.
`--keep-orientation` still moves the model onto the plate; `plan.json` records that translation.

This one fails, because the arm hangs over the part's own base:

```
mtilt prepare --units mm --profile profiles/example-fdm.json --keep-orientation --out occluded-result testdata/occluded.stl
```

`prepare` refuses to write into an existing path. It writes into a hidden staging directory next to `--out`, reads
every file back and checks it again, and renames the staging directory to `--out` only when everything passed.

### Output

```
result/
  model.stl              the oriented model, binary STL, millimeters
  supports/
    support-0001.stl     one closed mesh per support, same frame as model.stl
    ...
  plan.json              the report
```

`plan.json` records the input's SHA-256 digest, unit and conversion factor, the rigid transform and its inverse,
the effective profile, every candidate's metrics, score and support outcome, the selected candidate, each
support's position and file, every validation check, the properties mtilt did not check, warnings, and whether a
search limit was reached. It holds no timing data; `prepare` prints the elapsed time on stderr.

### Exit codes

| Code | Meaning |
|---|---|
| 0 | Success |
| 1 | No candidate could be supported, or the output could not be written |
| 2 | Invalid command, flag or profile, or `--out` already exists |
| 3 | The input file is missing, malformed, over a limit, or not a supported mesh |
| 130 | Interrupted |

## Importing into a slicer

mtilt has not been tested with any slicer. To keep the parts where mtilt put them:

1. Import `model.stl` and every file in `supports/` together, as parts of one object, so the slicer keeps their
   relative positions instead of centering or dropping each file on the plate by itself. How to do that differs
   between slicers; mtilt has not been checked against any of them.
2. Check that the pillars sit under the overhangs, with a visible gap between each pillar's top and the part.
3. Turn off the slicer's own support generation for this object.
4. Inspect the sliced preview layer by layer: the top gap, the pillar tips and the bases. A slicer can merge thin
   features or change the gap, depending on its settings.

[docs/validation.md](docs/validation.md) has the full procedure, including a small calibration print.

## Library

The root package `github.com/lestrrat-3d/mtilt` works on meshes, not files: `mtilt.Prepare` and `mtilt.Analyze`
take a `mesh.Mesh` and options and return a result with a report. Package `stl` reads and writes files. The
executable examples in [examples/](examples) prepare the bracket, show the occluded failure, rank the cuboid's
orientations, and read and validate an STL file. `go test ./examples/` runs them and checks their output.

## Development

```
go test ./...                          # unit, fixture and CLI end-to-end tests
go vet ./...
golangci-lint run                      # v2.12.2, config in .golangci.yml
go test ./internal/fixture -update     # regenerate testdata/ after changing a fixture
```

## License

[PolyForm Noncommercial 1.0.0](LICENSE).
