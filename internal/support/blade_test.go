package support_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bridgeParams turns bridges, and so blades, on: 10 mm spans, 5 degrees.
func bridgeParams() support.Params {
	p := params
	p.LayerHeightMM = 0.2
	p.MaxBridgeMM = 10
	p.MaxBridgeTiltDeg = 5
	return p
}

func bladeMesh(t *testing.T, p support.Params, bl support.Blade) *mesh.Mesh {
	t.Helper()
	body, err := p.BladeBody(t.Context(), sketch.NewWorld(), decad.New(), bl)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	m, _, err := mesh.FromBody(t.Context(), body, tessellationTolMM)
	require.NoError(t, err)
	return m
}

func TestBladeBody(t *testing.T) {
	cases := map[string]struct {
		axis       [2]float64
		wantX      float64 // X extent of the bounds
		wantY      float64 // Y extent of the bounds
		boundsKnow bool
	}{
		"along X":       {axis: [2]float64{1, 0}, wantX: params.BaseWidthMM, wantY: 4, boundsKnow: true},
		"along Y":       {axis: [2]float64{0, 1}, wantX: 4, wantY: params.BaseWidthMM, boundsKnow: true},
		"at 30 degrees": {axis: [2]float64{math.Cos(math.Pi / 6), math.Sin(math.Pi / 6)}},
	}
	for name, c := range cases {
		for _, top := range []float64{params.MinHeight(), 10} {
			bl := support.Blade{X: 3, Y: -4, AxisX: c.axis[0], AxisY: c.axis[1], LengthMM: 4, SurfaceZ: top + 0.2, TopZ: top}
			body, err := params.BladeBody(t.Context(), sketch.NewWorld(), decad.New(), bl)
			require.NoError(t, err, "%s top %g", name, top)
			require.True(t, body.IsSolid())
			require.Len(t, body.Lumps(), 1)

			b, err := body.Bounds()
			require.NoError(t, err)
			require.InDelta(t, 0, b.Min.Z, 1e-9)
			require.InDelta(t, top, b.Max.Z, 1e-9)
			require.InDelta(t, 3, (b.Min.X+b.Max.X)/2, 1e-9, name)
			require.InDelta(t, -4, (b.Min.Y+b.Max.Y)/2, 1e-9, name)
			if c.boundsKnow {
				require.InDelta(t, c.wantX, b.Max.X-b.Min.X, 1e-9, name)
				require.InDelta(t, c.wantY, b.Max.Y-b.Min.Y, 1e-9, name)
			}
			box := params.BladeBounds(bl)
			require.InDelta(t, b.Min.X, box.Min.X, 1e-9, name)
			require.InDelta(t, b.Max.Y, box.Max.Y, 1e-9, name)

			// Volume: the section (base rectangle, wall rectangle and
			// tip trapezoid) times the length.
			wall := top - params.TipHeightMM - params.BaseThicknessMM
			section := params.BaseWidthMM*params.BaseThicknessMM + params.PillarWidthMM*wall +
				(params.PillarWidthMM+params.ContactWidthMM)/2*params.TipHeightMM
			require.InDelta(t, section*4, params.BladeVolume(bl), 1e-9, name)
			vol, err := body.Volume()
			require.NoError(t, err)
			got, err := vol.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.InDelta(t, section*4, got, 1e-6, "%s top %g", name, top)
			require.InDelta(t, params.ContactWidthMM*4, params.BladeContactArea(bl), 1e-12)

			m := bladeMesh(t, params, bl)
			rep, err := mesh.Validate(t.Context(), m, mesh.ToleranceFor(m.Bounds()))
			require.NoError(t, err)
			require.Empty(t, rep.Failed(), "%s top %g", name, top)
		}
	}
}

// requireBladePlanValid re-checks a plan and its built blades against the
// model.
func requireBladePlanValid(t *testing.T, placed *mesh.Mesh, plan *support.Plan, p support.Params, tol mesh.Tolerance) {
	t.Helper()
	requireAllPassed(t, support.RecheckPlan(placed, plan, p, tol))
	var ms []*mesh.Mesh
	for _, bl := range plan.Blades {
		ms = append(ms, bladeMesh(t, p, bl))
	}
	for _, pl := range plan.Pillars {
		m, _ := pillarMesh(t, p, pl)
		ms = append(ms, m)
	}
	upright := make([]bool, len(ms))
	for i := len(plan.Blades); i < len(ms); i++ {
		upright[i] = true
	}
	checks, err := support.ValidateAssembly(t.Context(), placed, ms, upright, p.TopGapMM, 0, tol)
	require.NoError(t, err)
	requireAllPassed(t, checks)
}

func TestBlades(t *testing.T) {
	p := bridgeParams()
	buildWith := func(t *testing.T, soup [][3]r3.Vec, p support.Params) (*mesh.Mesh, *support.Plan, mesh.Tolerance) {
		t.Helper()
		placed, tol := placedAsIs(t, soup)
		plan, err := support.Build(t.Context(), placed, p, limits, tol)
		require.NoError(t, err)
		return placed, plan, tol
	}

	t.Run("a narrow cantilever gets blades that bridge back to the post", func(t *testing.T) {
		placed, plan, tol := buildWith(t, fixture.NarrowBracket(), p)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
		require.Positive(t, plan.Samples)
		// The post at X = 10 anchors the strip's inner end; the 30 mm
		// strip splits into three 9 mm spans between top faces, the
		// last blade ending at the free end. Placement moves the part by
		// (-20, -2), centering it on the origin.
		require.Len(t, plan.Blades, 3)
		for i, x := range []float64{19.5, 29.5, 39.5} {
			bl := plan.Blades[i]
			require.InDelta(t, x-20, bl.X, 1e-9)
			require.InDelta(t, 0, bl.Y, 1e-9)
			require.Equal(t, [2]float64{1, 0}, [2]float64{bl.AxisX, bl.AxisY})
			require.InDelta(t, fixture.NarrowDepthMM, bl.LengthMM, 1e-9)
			require.InDelta(t, 30, bl.SurfaceZ, 1e-9)
			require.LessOrEqual(t, bl.TopZ, 30-p.TopGapMM)
			require.Greater(t, bl.TopZ, 30-p.TopGapMM-1e-3)
			require.Equal(t, 1, bl.Strip)
		}
		requireBladePlanValid(t, placed, plan, p, tol)
	})

	t.Run("a narrow span between two posts gets blades between them only", func(t *testing.T) {
		placed, plan, tol := buildWith(t, fixture.NarrowBridge(), p)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
		// Both ends sit on posts: the 30 mm span needs two blades, which
		// leave three equal spans of (30 - 2) / 3 mm. Placement moves the
		// part by -25 along X.
		require.Len(t, plan.Blades, 2)
		span := 28.0 / 3
		require.InDelta(t, 10+span+0.5-25, plan.Blades[0].X, 1e-9)
		require.InDelta(t, 10+2*span+1.5-25, plan.Blades[1].X, 1e-9)
		requireBladePlanValid(t, placed, plan, p, tol)
	})

	t.Run("blades follow a strip at an angle to the axes", func(t *testing.T) {
		turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
		require.NoError(t, err)
		soup := fixture.NarrowBracket()
		for i := range soup {
			for k := range 3 {
				soup[i][k] = turn.Apply(soup[i][k])
			}
		}
		placed, plan, tol := buildWith(t, soup, p)
		require.Empty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
		require.Len(t, plan.Blades, 3)
		for _, bl := range plan.Blades {
			require.InDelta(t, math.Cos(math.Pi/6), bl.AxisX, 1e-9)
			require.InDelta(t, math.Sin(math.Pi/6), bl.AxisY, 1e-9)
		}
		// Along the axis the blades are 10 mm apart, as unturned.
		for i := 1; i < len(plan.Blades); i++ {
			a, b := plan.Blades[i-1], plan.Blades[i]
			require.InDelta(t, 10, math.Hypot(b.X-a.X, b.Y-a.Y), 1e-9)
		}
		requireBladePlanValid(t, placed, plan, p, tol)
	})

	t.Run("a strip whose end blade does not fit falls back to pillars", func(t *testing.T) {
		// A block on the plate under the arm's free end leaves no room
		// for the end blade's base; pillars beside the block cover the
		// end instead.
		soup := append(fixture.NarrowBracket(), fixture.Box(r3.NewVec(39, -1, 0), r3.NewVec(41, 5, 20))...)
		placed, plan, tol := buildWith(t, soup, p)
		require.Empty(t, plan.Blades)
		require.NotEmpty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
		requireBladePlanValid(t, placed, plan, p, tol)
	})

	t.Run("without bridges a strip gets pillars", func(t *testing.T) {
		_, plan, _ := buildWith(t, fixture.NarrowBracket(), params)
		require.Empty(t, plan.Blades)
		require.NotEmpty(t, plan.Pillars)
		require.Empty(t, plan.Uncovered)
	})

	t.Run("a wide or short region is not a strip", func(t *testing.T) {
		// 30 x 20 mm is wider than two support spacings.
		_, plan, _ := buildWith(t, fixture.Bracket(), p)
		require.Empty(t, plan.Blades)
		require.NotEmpty(t, plan.Pillars)
		// 6 x 4 mm is shorter than two support spacings.
		short := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {16, 30}, {16, 40}, {0, 40}}, 0, fixture.NarrowDepthMM)
		_, plan, _ = buildWith(t, short, p)
		require.Empty(t, plan.Blades)
		require.NotEmpty(t, plan.Pillars)
	})

	t.Run("triangle order and retriangulation do not change the blades", func(t *testing.T) {
		_, a, _ := buildWith(t, fixture.NarrowBracket(), p)
		soup := fixture.NarrowBracket()
		rng := rand.New(rand.NewPCG(5, 6))
		rng.Shuffle(len(soup), func(i, j int) { soup[i], soup[j] = soup[j], soup[i] })
		_, b, _ := buildWith(t, soup, p)
		_, c, _ := buildWith(t, splitEveryTriangle(fixture.NarrowBracket()), p)
		for _, got := range []*support.Plan{b, c} {
			require.Len(t, got.Blades, len(a.Blades))
			for i := range a.Blades {
				w, g := a.Blades[i], got.Blades[i]
				require.InDelta(t, w.SurfaceZ, g.SurfaceZ, 1e-9)
				w.SurfaceZ, g.SurfaceZ = 0, 0
				require.Equal(t, w, g)
			}
		}
	})

	t.Run("blades count against the support limit", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.NarrowBracket())
		_, err := support.Build(t.Context(), placed, p, support.Limits{MaxSupports: 2, MaxSamples: 100_000}, tol)
		require.ErrorIs(t, err, support.ErrLimit)
	})

	t.Run("planned volume and contact count the blades", func(t *testing.T) {
		_, plan, _ := buildWith(t, fixture.NarrowBracket(), p)
		var vol, contact float64
		for _, bl := range plan.Blades {
			vol += p.BladeVolume(bl)
			contact += p.BladeContactArea(bl)
		}
		require.InDelta(t, vol, p.PlanVolume(plan), 1e-9)
		require.InDelta(t, 3*p.ContactWidthMM*fixture.NarrowDepthMM, p.PlanContactArea(plan), 1e-9)
		require.InDelta(t, contact, p.PlanContactArea(plan), 1e-9)
		require.Equal(t, 3, plan.Supports())
	})
}
