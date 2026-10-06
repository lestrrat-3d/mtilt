package support_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var params = support.Params{
	ThresholdDeg:    45,
	SpacingMM:       4,
	ContactWidthMM:  1,
	PillarWidthMM:   2,
	TipHeightMM:     2,
	BaseWidthMM:     3,
	BaseThicknessMM: 0.6,
	TopGapMM:        0.2,
	SideClearanceMM: 0.5,
}

var limits = support.Limits{MaxSupports: 1000, MaxSamples: 100_000}

// placedAsIs puts soup on the plate without rotating it.
func placedAsIs(t *testing.T, soup [][3]r3.Vec) (*mesh.Mesh, mesh.Tolerance) {
	t.Helper()
	m, err := mesh.FromSoup(soup)
	require.NoError(t, err)
	placed, _, err := orient.Place(m, r3.Identity())
	require.NoError(t, err)
	b := placed.Bounds()
	d := b.Diagonal()
	return placed, mesh.ToleranceFor(b.Union(mesh.Box{Min: r3.NewVec(-d, -d, -d), Max: r3.NewVec(d, d, d)}))
}

func build(t *testing.T, soup [][3]r3.Vec) (*mesh.Mesh, *support.Plan, mesh.Tolerance) {
	t.Helper()
	placed, tol := placedAsIs(t, soup)
	plan, err := support.Build(t.Context(), placed, params, limits, tol)
	require.NoError(t, err)
	return placed, plan, tol
}

// tessellationTolMM is the chord tolerance pillar bodies are tessellated
// with in these tests.
const tessellationTolMM = 0.01

// pillarMesh builds pl as a decad body and returns its tessellation and the
// tessellation's bound.
func pillarMesh(t *testing.T, p support.Params, pl support.Pillar) (*mesh.Mesh, float64) {
	t.Helper()
	body, err := p.Body(t.Context(), sketch.NewWorld(), decad.New(), pl)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	m, bound, err := mesh.FromBody(t.Context(), body, tessellationTolMM)
	require.NoError(t, err)
	return m, bound
}

func meshes(t *testing.T, plan *support.Plan) []*mesh.Mesh {
	t.Helper()
	out := make([]*mesh.Mesh, len(plan.Pillars))
	for i, pl := range plan.Pillars {
		out[i], _ = pillarMesh(t, params, pl)
	}
	return out
}

func requireAllPassed(t *testing.T, checks []mesh.Check) {
	t.Helper()
	for _, c := range checks {
		require.Equal(t, mesh.StatusPassed, c.Status, "%s: %s", c.Name, c.Detail)
	}
}

func TestPillarBody(t *testing.T) {
	cases := map[string]support.Params{
		"default": params,
		"base as wide as shaft": func() support.Params {
			p := params
			p.BaseWidthMM = p.PillarWidthMM
			return p
		}(),
		"straight tip": func() support.Params {
			p := params
			p.ContactWidthMM = p.PillarWidthMM
			return p
		}(),
	}
	for name, p := range cases {
		for _, top := range []float64{p.MinHeight(), 10} {
			pl := support.Pillar{X: 3, Y: -4, SurfaceZ: top + 0.2, TopZ: top}
			body, err := p.Body(t.Context(), sketch.NewWorld(), decad.New(), pl)
			require.NoError(t, err, "%s top %g", name, top)
			require.True(t, body.IsSolid())
			require.Len(t, body.Lumps(), 1)

			b, err := body.Bounds()
			require.NoError(t, err)
			require.InDelta(t, 0, b.Min.Z, 1e-9)
			require.InDelta(t, top, b.Max.Z, 1e-9)
			require.InDelta(t, 3, (b.Min.X+b.Max.X)/2, 1e-9)
			require.InDelta(t, -4, (b.Min.Y+b.Max.Y)/2, 1e-9)
			require.InDelta(t, p.BaseWidthMM, b.Max.X-b.Min.X, 1e-6)

			// Volume: base disc, shaft cylinder, and the tip's cone
			// frustum pi h/3 (r1^2 + r1 r2 + r2^2).
			rb, rs, rc := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
			shaftH := top - p.TipHeightMM - p.BaseThicknessMM
			want := math.Pi*rb*rb*p.BaseThicknessMM + math.Pi*rs*rs*shaftH +
				math.Pi*p.TipHeightMM/3*(rs*rs+rs*rc+rc*rc)
			vol, err := body.Volume()
			require.NoError(t, err)
			got, err := vol.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.InDelta(t, want, got, 1e-6, "%s top %g", name, top)

			m, _ := pillarMesh(t, p, pl)
			rep, err := mesh.Validate(t.Context(), m, mesh.ToleranceFor(m.Bounds()))
			require.NoError(t, err)
			require.Empty(t, rep.Failed(), "%s top %g", name, top)
		}
	}
}

func TestBuild(t *testing.T) {
	t.Run("cube needs no supports", func(t *testing.T) {
		_, plan, _ := build(t, fixture.Cube())
		require.Zero(t, plan.DemandTriangles)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
	})

	for _, name := range []string{"bracket", "bridge"} {
		soup := map[string][][3]r3.Vec{"bracket": fixture.Bracket(), "bridge": fixture.Bridge()}[name]
		t.Run(name+" overhang is fully covered by valid pillars", func(t *testing.T) {
			placed, plan, tol := build(t, soup)
			require.Positive(t, plan.Samples)
			require.Empty(t, plan.Uncovered)
			require.NotEmpty(t, plan.Pillars)

			requireAllPassed(t, support.RecheckPlan(placed, plan, params, tol))
			checks, err := support.ValidateAssembly(t.Context(), placed, meshes(t, plan), allUpright(len(plan.Pillars)), params.TopGapMM, 0, tol)
			require.NoError(t, err)
			requireAllPassed(t, checks)

			for i, pl := range plan.Pillars {
				// Every pillar holds the 30 mm underside, with the top gap.
				require.InDelta(t, 30, pl.SurfaceZ, 1e-9)
				require.LessOrEqual(t, pl.TopZ, 30-params.TopGapMM)
				require.Greater(t, pl.TopZ, 30-params.TopGapMM-1e-3)
				for _, o := range plan.Pillars[i+1:] {
					apart := math.Abs(o.X-pl.X) >= params.BaseWidthMM || math.Abs(o.Y-pl.Y) >= params.BaseWidthMM
					require.True(t, apart, "bases of pillars at %v,%v and %v,%v overlap", pl.X, pl.Y, o.X, o.Y)
				}
			}
		})
	}

	t.Run("occluded overhang is reported, never dropped", func(t *testing.T) {
		_, plan, _ := build(t, fixture.Occluded())
		require.Positive(t, plan.DemandTriangles)
		require.Empty(t, plan.Pillars)
		require.Len(t, plan.Uncovered, plan.Samples)
		for _, u := range plan.Uncovered {
			require.Equal(t, support.ReasonOccluded, u.Reason)
			require.InDelta(t, 40, u.Point.Z, 1e-9)
		}
	})

	t.Run("slot narrower than a pillar's clearance zone is a collision", func(t *testing.T) {
		// A beam over a 3 mm slot: the slot leaves room for the pillar's
		// centerline and its 1 mm contact, but not for its 2 mm shaft
		// plus 0.5 mm side clearance on each side.
		slot := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {13, 30}, {13, 0}, {23, 0}, {23, 40}, {0, 40}}, 0, 20)
		_, plan, _ := build(t, slot)
		require.NotEmpty(t, plan.Uncovered)
		reasons := map[string]int{}
		for _, u := range plan.Uncovered {
			reasons[u.Reason]++
		}
		// Next to a wall the contact square overlaps the wall itself
		// (too low) or the vertical line grazes it (occluded); in the
		// middle of the slot only the full-volume check refuses.
		require.Positive(t, reasons[support.ReasonCollision])
		for r := range reasons {
			require.Contains(t, []string{support.ReasonCollision, support.ReasonOccluded, support.ReasonTooLow}, r)
		}
	})

	t.Run("overhang too close to the plate cannot hold a pillar", func(t *testing.T) {
		// A ledge whose underside is 1 mm up: lower than base plus tip.
		ledge := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 1}, {30, 1}, {30, 5}, {0, 5}}, 0, 20)
		_, plan, _ := build(t, ledge)
		require.NotEmpty(t, plan.Uncovered)
		require.Equal(t, support.ReasonTooLow, plan.Uncovered[len(plan.Uncovered)-1].Reason)
	})

	t.Run("triangle order does not change the pillars", func(t *testing.T) {
		soup := fixture.Bracket()
		_, a, _ := build(t, soup)
		rng := rand.New(rand.NewPCG(3, 4))
		rng.Shuffle(len(soup), func(i, j int) { soup[i], soup[j] = soup[j], soup[i] })
		_, b, _ := build(t, soup)
		requireSamePillars(t, a.Pillars, b.Pillars)
	})

	t.Run("equivalent planar triangulation does not change the pillars", func(t *testing.T) {
		_, a, _ := build(t, fixture.Bracket())
		_, b, _ := build(t, splitEveryTriangle(fixture.Bracket()))
		requireSamePillars(t, a.Pillars, b.Pillars)
	})

	t.Run("support limit", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		_, err := support.Build(t.Context(), placed, params, support.Limits{MaxSupports: 3, MaxSamples: 100_000}, tol)
		require.ErrorIs(t, err, support.ErrLimit)
	})

	t.Run("sample limit", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		_, err := support.Build(t.Context(), placed, params, support.Limits{MaxSupports: 1000, MaxSamples: 10}, tol)
		require.ErrorIs(t, err, support.ErrLimit)
	})

	t.Run("cancelled", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := support.Build(ctx, placed, params, limits, tol)
		require.ErrorIs(t, err, context.Canceled)
	})
}

// requireSamePillars compares positions, tops and origins exactly. The held
// surface height is interpolated inside whichever triangle the vertical line
// meets, so it may differ in the last bits.
func requireSamePillars(t *testing.T, want, got []support.Pillar) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		w, g := want[i], got[i]
		require.InDelta(t, w.SurfaceZ, g.SurfaceZ, 1e-9)
		w.SurfaceZ, g.SurfaceZ = 0, 0
		require.Equal(t, w, g)
	}
}

// splitEveryTriangle replaces each triangle with three that share its
// centroid. The surface and its winding are unchanged.
func splitEveryTriangle(soup [][3]r3.Vec) [][3]r3.Vec {
	var out [][3]r3.Vec
	for _, t := range soup {
		c := t[0].Add(t[1]).Add(t[2]).Scale(1.0 / 3)
		out = append(out, [3]r3.Vec{t[0], t[1], c}, [3]r3.Vec{t[1], t[2], c}, [3]r3.Vec{t[2], t[0], c})
	}
	return out
}

func TestValidateAssembly(t *testing.T) {
	placed, plan, tol := build(t, fixture.Bracket())
	good := meshes(t, plan)

	t.Run("pillar pushed into the model", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.TopZ = pl.SurfaceZ + 1
		bad := append([]*mesh.Mesh{first(t, pl)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, allUpright(len(bad)), params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportModelContact))
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopGap))
	})

	t.Run("pillar top inside the gap", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.TopZ = pl.SurfaceZ - params.TopGapMM/2
		bad := append([]*mesh.Mesh{first(t, pl)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, allUpright(len(bad)), params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusPassed, statusOf(checks, support.CheckSupportModelContact))
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopGap))
	})

	t.Run("overlapping supports", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.X += params.BaseWidthMM / 2
		bad := append([]*mesh.Mesh{first(t, pl)}, good...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, allUpright(len(bad)), params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportSeparation))
	})

	t.Run("support floating above the plate", func(t *testing.T) {
		lift, err := r3.Translation(r3.NewVec(0, 0, 0.5))
		require.NoError(t, err)
		bad := append([]*mesh.Mesh{good[0].Transformed(lift)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, allUpright(len(bad)), params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportOnPlate))
	})

	t.Run("open support mesh", func(t *testing.T) {
		open := &mesh.Mesh{Vertices: good[0].Vertices, Triangles: good[0].Triangles[1:]}
		bad := append([]*mesh.Mesh{open}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, allUpright(len(bad)), params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopology))
	})
}

func first(t *testing.T, pl support.Pillar) *mesh.Mesh {
	t.Helper()
	m, _ := pillarMesh(t, params, pl)
	return m
}

func allUpright(n int) []bool {
	out := make([]bool, n)
	for i := range out {
		out[i] = true
	}
	return out
}

func statusOf(checks []mesh.Check, name string) mesh.Status {
	for _, c := range checks {
		if c.Name == name {
			return c.Status
		}
	}
	return ""
}

func TestBridges(t *testing.T) {
	withBridges := func(maxMM float64) support.Params {
		p := params
		p.LayerHeightMM = 0.2
		p.MaxBridgeMM = maxMM
		p.MaxBridgeTiltDeg = 5
		return p
	}
	slot := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {13, 30}, {13, 0}, {23, 0}, {23, 40}, {0, 40}}, 0, 20)

	t.Run("a 3 mm slot is bridged", func(t *testing.T) {
		placed, tol := placedAsIs(t, slot)
		plan, err := support.Build(t.Context(), placed, withBridges(10), limits, tol)
		require.NoError(t, err)
		require.Positive(t, plan.Bridged)
		require.Positive(t, plan.Held, "the slot's top edges sit on its walls")
		require.Zero(t, plan.Samples)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
	})

	t.Run("bridges off leaves the slot unsupportable", func(t *testing.T) {
		placed, tol := placedAsIs(t, slot)
		plan, err := support.Build(t.Context(), placed, withBridges(0), limits, tol)
		require.NoError(t, err)
		require.Zero(t, plan.Bridged)
		require.Zero(t, plan.Held)
		require.NotEmpty(t, plan.Uncovered)
	})

	t.Run("a 30 mm span is longer than a 10 mm bridge", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bridge())
		plan, err := support.Build(t.Context(), placed, withBridges(10), limits, tol)
		require.NoError(t, err)
		require.Zero(t, plan.Bridged)
		require.NotEmpty(t, plan.Pillars)
	})

	t.Run("the same span is bridged under a 35 mm limit", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bridge())
		plan, err := support.Build(t.Context(), placed, withBridges(35), limits, tol)
		require.NoError(t, err)
		require.Positive(t, plan.Bridged)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
	})

	t.Run("a cantilever has one wall and is never a bridge", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		plan, err := support.Build(t.Context(), placed, withBridges(100), limits, tol)
		require.NoError(t, err)
		require.Zero(t, plan.Bridged)
		require.NotEmpty(t, plan.Pillars)
	})

	t.Run("a sloped ceiling over the tilt limit is not a bridge", func(t *testing.T) {
		// A 3 mm slot whose ceiling rises 2 mm across its width: 33.7
		// degrees from horizontal.
		sloped := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {13, 32}, {13, 0}, {23, 0}, {23, 40}, {0, 40}}, 0, 20)
		placed, tol := placedAsIs(t, sloped)
		plan, err := support.Build(t.Context(), placed, withBridges(10), limits, tol)
		require.NoError(t, err)
		require.Zero(t, plan.Bridged)
	})
}

func TestBranches(t *testing.T) {
	withBranches := params
	withBranches.MaxLeanDeg = 40

	t.Run("occluded overhang is covered by branches rooted beside the base", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Occluded())
		plan, err := support.Build(t.Context(), placed, withBranches, limits, tol)
		require.NoError(t, err)
		require.Empty(t, plan.Uncovered)
		require.NotEmpty(t, plan.Pillars)
		requireAllPassed(t, support.RecheckPlan(placed, plan, withBranches, tol))
		b := placed.Bounds()
		slope := math.Tan(40 * math.Pi / 180)
		for _, pl := range plan.Pillars {
			require.True(t, pl.IsBranch())
			// The foot is off the base, and the lean stays within 40
			// degrees of vertical.
			off := pl.FootX > b.Max.X || pl.FootX < b.Min.X || pl.FootY > b.Max.Y || pl.FootY < b.Min.Y
			require.True(t, off, "foot %v,%v", pl.FootX, pl.FootY)
			run := math.Hypot(pl.X-pl.FootX, pl.Y-pl.FootY)
			rise := pl.TopZ - withBranches.TipHeightMM - pl.KneeZ
			require.LessOrEqual(t, run, rise*slope+1e-9)
			require.GreaterOrEqual(t, pl.KneeZ, withBranches.BaseThicknessMM+1-1e-9)
		}
	})

	t.Run("branches are off when the lean limit is 0", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Occluded())
		plan, err := support.Build(t.Context(), placed, params, limits, tol)
		require.NoError(t, err)
		require.Empty(t, plan.Pillars)
		require.NotEmpty(t, plan.Uncovered)
	})

	t.Run("a branch body is one solid swept along its path", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Occluded())
		plan, err := support.Build(t.Context(), placed, withBranches, limits, tol)
		require.NoError(t, err)
		pl := plan.Pillars[0]
		body, err := withBranches.Body(t.Context(), sketch.NewWorld(), decad.New(), pl)
		require.NoError(t, err)
		require.True(t, body.IsSolid())
		require.Len(t, body.Lumps(), 1)
		bb, err := body.Bounds()
		require.NoError(t, err)
		require.InDelta(t, 0, bb.Min.Z, 1e-9)
		require.InDelta(t, pl.TopZ, bb.Max.Z, 1e-9)
		vol, err := body.Volume()
		require.NoError(t, err)
		got, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		// The 16-gon is inscribed in the circle the frustum estimate uses
		// (area ratio 0.9745), and the mitre wedges add a little.
		want := withBranches.Volume(pl)
		require.InDelta(t, want*0.9745, got, want*0.03)

		m, _, err := mesh.FromBody(t.Context(), body, tessellationTolMM)
		require.NoError(t, err)
		rep, err := mesh.Validate(t.Context(), m, mesh.ToleranceFor(m.Bounds()))
		require.NoError(t, err)
		require.Empty(t, rep.Failed())
	})
}
