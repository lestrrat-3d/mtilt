# CLAUDE.md

Guidance for agents working in this repository. Read before structural changes. Update when a decision changes.

## What this is

**mtilt** (mesh-tilt): Go library. Takes a solid `*decad.Body`, picks an FDM print orientation (strength first:
long axis within a tilt limit; then least support, found by a sphere sweep + refinement + real planning), plans
bed-rooted pillar supports on decad's tessellation, builds the moved
model + one revolved pillar body per support in the body's decad document, verifies them with decad. Pure Go, CPU
only. No file I/O; examples export with `decad/export`.

`docs/design.md` is the contract: scope, transform convention, tolerances, algorithms, document effects, limits,
package layout. `docs/roadmap.md` lists unbuilt work. `docs/validation.md` lists automated checks and the manual
procedure + log.

## Read before you write

| Before touching | Read |
|---|---|
| Any file | `docs/design.md` section 10 (package layout) |
| Tessellation, tolerances, input checks | `docs/design.md` sections 3, 4 |
| Orientation code (`internal/orient/`) | `docs/design.md` section 5 |
| Overhang classification, plate anchor, bridges | `docs/design.md` section 6 |
| Support code (`internal/support/`) | `docs/design.md` section 7 |
| Anything that adds bodies to a document | `docs/design.md` section 8 |
| Limits | `docs/design.md` section 9 |
| decad API | decad's `docs/api-design.md`, `docs/layout.md`, and its doc comments |

## Commands

```
go test ./...            # MUST pass; ~30 s, most of it decad Verify
go test -race ./...      # MUST pass
go vet ./...             # MUST pass
golangci-lint run        # v2.12.2, config .golangci.yml (copied from kinetograph)
CGO_ENABLED=0 go build ./...
go test -run '^$' -fuzz FuzzPrincipal -fuzztime 30s ./internal/orient
cd _gallery && go run .  # regenerate docs/images after changing orientation or supports
```

Local `golangci-lint` version MUST match CI's before trusting a clean local run.

## Hard rules

- **NEVER claim validation that did not happen.** No sentence in code, docs, reports or commit messages may say
  a support was printed, sliced, removed cleanly, works in a named slicer, or makes a part stronger unless
  `docs/validation.md`'s log records that run. Checks passing → say "passed mtilt's and decad's geometric checks".
- **NEVER report an unchecked property as passed.** Use `mesh.StatusUnchecked` and add an `Unchecked` entry.
  Unavailable metric → `null`/omitted, NEVER 0.
- **NEVER return success-shaped failures.** Uncovered demand, collision, build-volume overflow, failed decad
  verification or limit hit → fail with a reason. NEVER drop demand because no pillar fits.
- **Add bodies to the caller's document only after the plan passed every planning check.** decad cannot remove a
  body. Planning failures add nothing; only `ErrAssembly` leaves bodies behind, and returns them.
- **NEVER consume or modify the input body.** Use `PlacedCopy`, never `Placed`, on it. Temporary bodies mtilt
  builds itself may be `Placed` (that retires them).
- **Model moves by one proper rigid transform only.** NEVER mirror, scale, remesh or merge supports into it.
- **Plan on the tessellation, widened by its proven bound.** Every gap/clearance check on a mesh adds
  `mesh.FromBody`'s bound. Coordinates are mm, +Z up, plate at Z = 0.
- **Epsilons come from `mesh.Tolerance` only.** NEVER add a bare epsilon for geometry comparison. Manufacturing
  clearances stay profile values.
- **decad pairwise clearance (`WithClearances`) is too slow for pillars** (9.7 s vs 0.33 s for 36 bodies). Use
  `Verify` for interference + validity; check gaps on meshes.
- **decad booleans reject face-touching operands.** Build multi-part solids as one revolve/extrude, or overlap
  the operands.
- **Determinism:** no map iteration into output order, no `time.Now` or randomness in the library, no timing in
  the report. Sort with explicit tie-breaks.
- **Bounded work:** every loop over triangles, samples or candidates checks `ctx` and respects `Limits`.
- **Strength outranks support.** NEVER let a support saving pick an orientation over `MaxLongAxisTiltDeg`. Change
  the limit, never trade it in a weighted sum.
- **Keep long axis, estimate/search, measurement and ranking separate** (`internal/orient/` files).
- **NEVER add a no-op option, placeholder support, fake score or stub package.** Unbuilt features go in
  `docs/roadmap.md`.
- **README images come from `_gallery/` (own module).** Regenerate and commit `docs/images/*.png` whenever a
  change alters an orientation or a support layout. NEVER import solidlens into the root module.
- **NEVER add a `go.mod` dependency without recording it in `docs/design.md` section 11.** Approved:
  `github.com/lestrrat-3d/decad`, `github.com/lestrrat-3d/sketch`, `github.com/lestrrat-3d/r3`,
  `github.com/lestrrat-3d/units`, `github.com/stretchr/testify` (tests only). No cgo, network, Python, external
  slicer or GPU.

## Supported scope (summary; design.md owns it)

- Input: one solid decad body with one lump whose tessellation passes every mesh check. Self-intersection
  unchecked.
- Supports: round pillars from the plate to demand surfaces reachable straight from below. Occluded, too-low or
  too-narrow demand → uncovered → candidate fails. Surfaces within `plate_anchor_height_mm` of the plate need no
  support. Spans up to `max_bridge_mm` between two walls, and points on top of a wall, need no support. Removal
  and layer strength unverified.

## Conventions

- Go style, tests, file layout: `~/.claude/docs/go.md`. Tests use `testify/require`, external `_test` packages,
  `t.Context()`, `t.TempDir()`.
- Test shapes come from `internal/fixture`: triangle soups for mesh-level packages, decad bodies
  (`fixture.Bodies`) for the pipeline.
- Every capability has a test asserting a computed value, never "it ran".
- User-facing usage → executable examples in `examples/` with `// Output:`. NEVER README-only snippets.
- Docs state current behavior only; no changelogs.
- Prose a human reads follows `~/.claude/docs/prose.md`.
