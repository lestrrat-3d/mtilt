package support_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
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

func meshes(plan *support.Plan) []*mesh.Mesh {
	out := make([]*mesh.Mesh, len(plan.Pillars))
	for i, pl := range plan.Pillars {
		out[i] = params.Mesh(pl)
	}
	return out
}

func requireAllPassed(t *testing.T, checks []mesh.Check) {
	t.Helper()
	for _, c := range checks {
		require.Equal(t, mesh.StatusPassed, c.Status, "%s: %s", c.Name, c.Detail)
	}
}

func TestPillarMesh(t *testing.T) {
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
			m := p.Mesh(pl)
			rep, err := mesh.Validate(t.Context(), m, mesh.ToleranceFor(m.Bounds()))
			require.NoError(t, err)
			require.Empty(t, rep.Failed(), "%s top %g", name, top)
			b := m.Bounds()
			require.Zero(t, b.Min.Z)
			require.InDelta(t, top, b.Max.Z, 1e-12)
			require.InDelta(t, p.BaseWidthMM, b.Max.X-b.Min.X, 1e-12)

			// Volume: base box, shaft box, and the tip frustum
			// h/3 (A1 + A2 + sqrt(A1 A2)).
			shaftH := top - p.TipHeightMM - p.BaseThicknessMM
			a1, a2 := p.PillarWidthMM*p.PillarWidthMM, p.ContactWidthMM*p.ContactWidthMM
			want := p.BaseWidthMM*p.BaseWidthMM*p.BaseThicknessMM + a1*shaftH +
				p.TipHeightMM/3*(a1+a2+math.Sqrt(a1*a2))
			require.InDelta(t, want, m.Volume(), 1e-9, "%s top %g", name, top)
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
			checks, err := support.ValidateAssembly(t.Context(), placed, meshes(plan), params.TopGapMM, 0, tol)
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
	good := meshes(plan)

	t.Run("pillar pushed into the model", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.TopZ = pl.SurfaceZ + 1
		bad := append([]*mesh.Mesh{params.Mesh(pl)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportModelContact))
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopGap))
	})

	t.Run("pillar top inside the gap", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.TopZ = pl.SurfaceZ - params.TopGapMM/2
		bad := append([]*mesh.Mesh{params.Mesh(pl)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusPassed, statusOf(checks, support.CheckSupportModelContact))
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopGap))
	})

	t.Run("overlapping supports", func(t *testing.T) {
		pl := plan.Pillars[0]
		pl.X += params.BaseWidthMM / 2
		bad := append([]*mesh.Mesh{params.Mesh(pl)}, good...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportSeparation))
	})

	t.Run("support floating above the plate", func(t *testing.T) {
		lift, err := r3.Translation(r3.NewVec(0, 0, 0.5))
		require.NoError(t, err)
		bad := append([]*mesh.Mesh{good[0].Transformed(lift)}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportOnPlate))
	})

	t.Run("open support mesh", func(t *testing.T) {
		open := &mesh.Mesh{Vertices: good[0].Vertices, Triangles: good[0].Triangles[1:]}
		bad := append([]*mesh.Mesh{open}, good[1:]...)
		checks, err := support.ValidateAssembly(t.Context(), placed, bad, params.TopGapMM, 0, tol)
		require.NoError(t, err)
		require.Equal(t, mesh.StatusFailed, statusOf(checks, support.CheckSupportTopology))
	})
}

func statusOf(checks []mesh.Check, name string) mesh.Status {
	for _, c := range checks {
		if c.Name == name {
			return c.Status
		}
	}
	return ""
}
