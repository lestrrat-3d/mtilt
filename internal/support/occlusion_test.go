package support_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// pitch is the sample grid pitch of params: SpacingMM / 4.
const pitch = 1.0

func TestSamples(t *testing.T) {
	t.Run("grid flag follows the lattice, vertices included", func(t *testing.T) {
		// The bracket with its arm reaching to X = 40.5: the arm's
		// underside spans X 10..40.5, Y 0..20 at Z = 30. Its corners at
		// X = 10 lie on the lattice and are pushed first as vertices; the
		// corners at X = 40.5 do not.
		// The part already stands on the plate; it is not centered, so the
		// lattice keeps its relation to the profile.
		arm := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {40.5, 30}, {40.5, 40}, {0, 40}}, 0, 20)
		placed, err := mesh.FromSoup(arm)
		require.NoError(t, err)
		tol := mesh.ToleranceFor(placed.Bounds())
		samples, bridged, held, err := support.Samples(t.Context(), placed, params, limits, tol)
		require.NoError(t, err)
		require.Zero(t, bridged)
		require.Zero(t, held)

		plan, err := support.Build(t.Context(), placed, params, limits, tol)
		require.NoError(t, err)
		require.Len(t, samples, plan.Samples)

		var grid, other int
		for _, s := range samples {
			onLattice := s.Point.X/pitch == math.Round(s.Point.X/pitch) && s.Point.Y/pitch == math.Round(s.Point.Y/pitch)
			require.Equal(t, onLattice, s.Grid, "sample %v", s.Point)
			if s.Grid {
				grid++
				continue
			}
			other++
		}
		// X 10..40 by Y 0..20; the two corners at X = 40.5 are the only
		// samples off the lattice.
		require.Equal(t, 31*21, grid)
		require.Equal(t, 2, other)
	})

	t.Run("no demand gives no samples", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Cube())
		samples, _, _, err := support.Samples(t.Context(), placed, params, limits, tol)
		require.NoError(t, err)
		require.Empty(t, samples)
	})

	t.Run("sample limit", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		_, _, _, err := support.Samples(t.Context(), placed, params, support.Limits{MaxSamples: 10}, tol)
		require.ErrorIs(t, err, support.ErrLimit)
	})
}

func TestOccluded(t *testing.T) {
	placed, tol := placedAsIs(t, fixture.StackedShelves())
	occ, err := support.NewOcclusion(t.Context(), placed, params, tol)
	require.NoError(t, err)
	// Placement centers the 20 x 20 mm footprint: the shelves span X
	// 0..10 and Y -10..10.
	require.False(t, occ.Occluded(r3.NewVec(5, 0, 20)), "the lower shelf's underside")
	require.True(t, occ.Occluded(r3.NewVec(5, 0, 40)), "the upper shelf: the line first meets the lower shelf, 20 mm lower")
	require.True(t, occ.Occluded(r3.NewVec(50, 50, 40)), "nothing above")

	placed, tol = placedAsIs(t, fixture.Occluded())
	occ, err = support.NewOcclusion(t.Context(), placed, params, tol)
	require.NoError(t, err)
	// The ⊐ part spans X -20..20; its top arm's underside lies over the
	// base, whose first triangle met is on the plate.
	require.True(t, occ.Occluded(r3.NewVec(5, 0, 40)))
}

func TestMeasureOcclusion(t *testing.T) {
	measure := func(t *testing.T, p support.Params, soup [][3]r3.Vec) (support.OcclusionReport, *support.Plan) {
		t.Helper()
		placed, tol := placedAsIs(t, soup)
		rep, err := support.MeasureOcclusion(t.Context(), placed, p, limits, tol)
		require.NoError(t, err)
		plan, err := support.Build(t.Context(), placed, p, limits, tol)
		require.NoError(t, err)
		require.NotNil(t, rep.Samples)
		require.Equal(t, plan.Samples, *rep.Samples)
		return rep, plan
	}

	t.Run("bracket", func(t *testing.T) {
		rep, plan := measure(t, params, fixture.Bracket())
		// The vertical line under the arm's edge at the post grazes the
		// post's bottom face first: 21 samples along Y. Pillars beside
		// them cover them, so the plan is complete.
		require.Equal(t, 21, *rep.OccludedSamples)
		require.InDelta(t, 21*pitch*pitch, *rep.OccludedGridAreaMM2, 1e-9)
		require.Empty(t, plan.Uncovered)
	})

	t.Run("occluded part", func(t *testing.T) {
		rep, plan := measure(t, params, fixture.Occluded())
		require.Equal(t, len(plan.Uncovered), *rep.OccludedSamples)
		require.Equal(t, plan.Samples, *rep.OccludedSamples)
		// The top arm's underside, X 10..40 by Y 0..20, at pitch 1.
		require.InDelta(t, 31*21*pitch*pitch, *rep.OccludedGridAreaMM2, 1e-9)
	})

	t.Run("stacked shelves", func(t *testing.T) {
		p := params
		p.LayerHeightMM = 0.2
		p.MaxBridgeMM = 10
		p.MaxBridgeTiltDeg = 5
		rep, plan := measure(t, p, fixture.StackedShelves())
		// The wall holds each shelf's X = 10 edge: 10 x 21 samples per
		// shelf remain.
		require.Equal(t, 420, plan.Samples)
		require.Equal(t, 210, *rep.OccludedSamples)
		require.Len(t, plan.Uncovered, 210)
		reasons := map[string]int{}
		for _, u := range plan.Uncovered {
			reasons[u.Reason]++
		}
		// The planner refuses each upper-shelf position for the pillar
		// standing there or beside it, never as occluded.
		require.Equal(t, map[string]int{support.ReasonCollision: 42, support.ReasonCrowded: 168}, reasons)
		require.InDelta(t, 210*pitch*pitch, *rep.OccludedGridAreaMM2, 1e-9)
	})

	t.Run("sample limit gives no numbers", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		rep, err := support.MeasureOcclusion(t.Context(), placed, params, support.Limits{MaxSamples: 10}, tol)
		require.NoError(t, err)
		require.Equal(t, support.OcclusionReport{}, rep)
	})

	t.Run("cancelled", func(t *testing.T) {
		placed, tol := placedAsIs(t, fixture.Bracket())
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := support.MeasureOcclusion(ctx, placed, params, limits, tol)
		require.ErrorIs(t, err, context.Canceled)
		_, err = support.NewOcclusion(ctx, placed, params, tol)
		require.ErrorIs(t, err, context.Canceled)
		plan, err := support.Build(t.Context(), placed, params, limits, tol)
		require.NoError(t, err)
		_, err = support.RecheckPlan(ctx, placed, plan, params, tol)
		require.ErrorIs(t, err, context.Canceled)
	})
}
