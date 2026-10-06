# CLAUDE.md

Guidance for agents working in this repository. Read before structural changes. Update when a decision changes.

## What this is

**mtilt** (mesh-tilt): Go library + thin CLI. Reads a triangle mesh, picks an FDM print orientation, builds
separate bed-rooted pillar supports, writes model + supports as STL plus `plan.json`. Pure Go, CPU only.

`docs/design.md` is the contract: scope, units, transform convention, tolerances, algorithms, limits, package
layout. `docs/roadmap.md` lists unbuilt work. `docs/validation.md` lists automated checks and the manual
procedure + log.

## Read before you write

| Before touching | Read |
|---|---|
| Any file | `docs/design.md` section 10 (package layout) |
| Mesh validation, tolerances | `docs/design.md` sections 3, 4 |
| Orientation code (`internal/orient/`) | `docs/design.md` section 5 |
| Overhang classification | `docs/design.md` section 6 |
| Support code (`internal/support/`) | `docs/design.md` section 7 |
| CLI, output, report fields | `docs/design.md` section 8; `README.md` |
| Limits | `docs/design.md` section 9 |
| Tests, fixtures | `docs/validation.md`; `internal/fixture/` |

## Commands

```
go test ./...                          # MUST pass; includes CLI end-to-end tests (builds cmd/mtilt)
go test -race ./...                    # MUST pass
go vet ./...                           # MUST pass
golangci-lint run                      # v2.12.2, config .golangci.yml (copied from kinetograph)
CGO_ENABLED=0 go build ./...           # MUST pass
go test ./internal/fixture -update     # regenerate testdata/ after changing internal/fixture
go test -run '^$' -fuzz FuzzDecode -fuzztime 30s ./stl
go test -run '^$' -fuzz FuzzAnalyze -fuzztime 30s .
```

Local `golangci-lint` version MUST match CI's before trusting a clean local run.

## Hard rules

- **NEVER claim validation that did not happen.** No sentence in code, docs, reports or commit messages may say
  a support was printed, sliced, removed cleanly, or works in a named slicer unless `docs/validation.md`'s log
  records that run. Geometric checks passing → say "passed mtilt's geometric checks", nothing more.
- **NEVER report an unchecked property as passed.** Use `mesh.StatusUnchecked` and add an `Unchecked` entry.
  Unavailable metric → `null`/omitted, NEVER 0.
- **NEVER return success-shaped failures.** Uncovered demand, collision, build-volume overflow or limit hit →
  candidate fails with a reason. No candidate succeeds → `ErrNoFeasibleCandidate` + report. NEVER drop demand
  because no pillar fits.
- **Model moves by one proper rigid transform only.** Unit conversion happens once (`Mesh.Scaled`). NEVER mirror,
  rescale, remesh, weld, repair, or merge supports into the model. Model + supports share one frame.
- **Units:** float64 mm internally, +Z up, plate at Z = 0. Input unit is always explicit. NEVER infer scale from
  size. Profile JSON fields carry their unit in the key (`_mm`, `_deg`).
- **Epsilons come from `mesh.Tolerance` only.** NEVER add a bare epsilon constant for geometry comparison. Use
  `Length`, `Serialization` or `Plate`. Manufacturing clearances stay profile values.
- **Normals come from winding.** NEVER read stored STL normals for geometry.
- **Determinism:** no map iteration into output order, no `time.Now` or randomness in the library, no timing in
  `plan.json`. Sort with explicit tie-breaks. Same input + options → byte-identical report.
- **Bounded work:** every loop over triangles, samples or candidates checks `ctx` and respects `Limits`. NEVER
  size an allocation from a file-declared count before the file length confirms it.
- **Root API takes meshes, not paths.** File handling lives in `stl/` and `internal/cli/`.
- **Keep generation, measurement, ranking and feasibility separate** (`internal/orient/` files).
- **NEVER add a no-op flag, placeholder support, fake score or stub package.** Unbuilt features go in
  `docs/roadmap.md`.
- **NEVER add a `go.mod` dependency without recording it in `docs/design.md` section 11.** Approved:
  `github.com/lestrrat-3d/r3`, `github.com/lestrrat-3d/units` (`internal/fixture` only),
  `github.com/stretchr/testify` (tests only).
  No cgo, network, Python, external slicer or GPU in library or tests.

## Supported scope (summary; design.md owns it)

- Input: one closed, edge/vertex-manifold, consistently outward-wound, non-degenerate component. Multiple
  components / nested shells → `ErrUnsupportedInput`. Self-intersection unchecked.
- Supports: vertical square pillars from the plate to demand surfaces reachable straight from below. Occluded,
  too-low, or too-narrow demand → uncovered → candidate fails. Bridges are demand. Removal unverified.

## Conventions

- Go style, tests, file layout: `~/.claude/docs/go.md`. Tests use `testify/require`, external `_test` packages,
  `t.Context()`, `t.TempDir()`.
- Test shapes come from `internal/fixture`; files in `testdata/` are generated from it, never hand-edited.
- Every capability has a test asserting a computed value (area, position, status, exit code), never "it ran".
- User-facing usage → executable examples in `examples/` with `// Output:`. NEVER README-only snippets that no
  test runs; README CLI commands are mirrored by `internal/cli/cli_test.go`.
- Docs state current behavior only; no changelogs.
- Prose a human reads follows `~/.claude/docs/prose.md`.
