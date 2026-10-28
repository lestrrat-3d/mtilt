package mtilt_test

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type shape func(*fixture.Bodies, context.Context) (*decad.Body, error)

// Support kinds as SupportReport.Kind names them.
const (
	kindBranch = "branch"
	kindTree   = "tree"
)

func newBody(t *testing.T, f shape) *decad.Body {
	t.Helper()
	b, err := f(fixture.NewBodies(), t.Context())
	require.NoError(t, err)
	return b
}

func prepare(t *testing.T, body *decad.Body, opts mtilt.Options) (*mtilt.Result, error) {
	t.Helper()
	if opts.Profile.Name == "" {
		opts.Profile = mtilt.ExampleProfile()
	}
	return mtilt.Prepare(t.Context(), body, opts)
}

func requireFailure(t *testing.T, err error, target error) *mtilt.FailureError {
	t.Helper()
	require.ErrorIs(t, err, target)
	var fe *mtilt.FailureError
	require.ErrorAs(t, err, &fe)
	require.NotNil(t, fe.Report)
	return fe
}

func requirePassed(t *testing.T, checks []mesh.Check, name string) {
	t.Helper()
	for _, c := range checks {
		if c.Name == name {
			require.Equal(t, mesh.StatusPassed, c.Status, "%s: %s", c.Name, c.Detail)
			return
		}
	}
	t.Fatalf("no check %q", name)
}

func requireNoFailedCheck(t *testing.T, rep mtilt.Report) {
	t.Helper()
	for _, c := range rep.Validation {
		require.NotEqual(t, mesh.StatusFailed, c.Status, "%s: %s", c.Name, c.Detail)
	}
}

// requireMovedCopy checks that model is input moved by tr: decad's centroid
// and volume agree under the transform.
func requireMovedCopy(t *testing.T, input, model *decad.Body, tr r3.Transform) {
	t.Helper()
	require.False(t, tr.IsReflection())
	ci, err := input.Centroid()
	require.NoError(t, err)
	cm, err := model.Centroid()
	require.NoError(t, err)
	require.True(t, tr.Apply(ci.Value).Equal(cm.Value, 1e-6), "centroid %v moved to %v, want %v", ci.Value, cm.Value, tr.Apply(ci.Value))
	vi, err := input.Volume()
	require.NoError(t, err)
	vm, err := model.Volume()
	require.NoError(t, err)
	a, err := vi.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	b, err := vm.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, a, b, 1e-6*a)
}

func selected(res *mtilt.Result) mtilt.CandidateReport {
	return res.Report.Candidates[*res.Report.Selected]
}

func TestPrepare(t *testing.T) {
	t.Run("cube needs no supports and keeps its orientation", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).Cube)
		res, err := prepare(t, body, mtilt.Options{})
		require.NoError(t, err)
		require.Empty(t, res.Supports)
		require.Equal(t, 0, *res.Report.Selected)
		require.Equal(t, mtilt.AttemptNotNeeded, selected(res).Attempt.Status)
		require.Zero(t, res.Report.Input.Elongation)
		require.Equal(t, [3]float64{-10, -10, 0}, res.Report.Transform.Translation)
		requireMovedCopy(t, body, res.Model, res.Transform)
		requireNoFailedCheck(t, res.Report)

		// The input stays live; the moved copy joins its document.
		live := body.Document().Bodies()
		require.Contains(t, live, body)
		require.Contains(t, live, res.Model)
	})

	t.Run("oblique cuboid is turned onto its largest face", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).ObliqueCuboid)
		res, err := prepare(t, body, mtilt.Options{})
		require.NoError(t, err)
		sel := selected(res)
		require.Equal(t, 1, sel.Rank)
		require.Zero(t, sel.Metrics.SupportDemandAreaMM2)
		require.InDelta(t, 800, sel.Metrics.BedContactAreaMM2, 1e-6)
		require.InDelta(t, 10, sel.Metrics.HeightMM, 1e-6)
		// As given, the cuboid's long axis rises over the default limit.
		require.False(t, res.Report.Candidates[0].TiltAllowed)
		require.LessOrEqual(t, sel.Metrics.LongAxisElevationDeg, float64(mtilt.DefaultMaxLongAxisTiltDeg))
		requireMovedCopy(t, body, res.Model, res.Transform)

		b, err := res.Model.Bounds()
		require.NoError(t, err)
		require.InDelta(t, 0, b.Min.Z, 1e-6)
		require.InDelta(t, 10, b.Max.Z, 1e-6)
	})

	t.Run("fixed-orientation bracket gets pillar bodies", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).Bracket)
		res, err := prepare(t, body, mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.Equal(t, "keep_orientation", res.Report.Mode)
		require.Len(t, res.Report.Candidates, 1)
		require.Len(t, res.Supports, 41)
		require.Len(t, res.Report.Supports, 41)
		require.True(t, res.Transform.Basis() == r3.Identity().Basis(), "rotation kept")
		requireMovedCopy(t, body, res.Model, res.Transform)
		requireNoFailedCheck(t, res.Report)
		requirePassed(t, res.Report.Validation, "decad_interference")
		requirePassed(t, res.Report.Validation, "decad_body_validity")

		mb, err := res.Model.Bounds()
		require.NoError(t, err)
		live := body.Document().Bodies()
		for i, s := range res.Supports {
			require.True(t, s.IsSolid())
			require.Same(t, body.Document(), s.Document())
			require.Contains(t, live, s)
			sr := res.Report.Supports[i]
			require.Equal(t, fmt.Sprintf("support-%04d", i+1), sr.ID)
			b, err := s.Bounds()
			require.NoError(t, err)
			require.InDelta(t, 0, b.Min.Z, 1e-9)
			// Same frame as the model: each pillar stops under the arm.
			require.InDelta(t, 30-0.2, b.Max.Z, 1e-3)
			require.GreaterOrEqual(t, sr.CenterMM[0], mb.Min.X+10)
			require.Positive(t, sr.VolumeMM3)
		}
	})

	t.Run("fixed-orientation bridge gets supports between its posts only", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Bridge), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.NotEmpty(t, res.Supports)
		for _, sr := range res.Report.Supports {
			require.Greater(t, sr.CenterMM[0], -15.0)
			require.Less(t, sr.CenterMM[0], 15.0)
		}
		requireNoFailedCheck(t, res.Report)
	})

	t.Run("occluded overhang gets branches from beside the base", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).Occluded)
		res, err := prepare(t, body, mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		requireNoFailedCheck(t, res.Report)
		requirePassed(t, res.Report.Validation, "decad_interference")
		require.NotEmpty(t, res.Supports)
		mb, err := res.Model.Bounds()
		require.NoError(t, err)
		for i, sr := range res.Report.Supports {
			// The base covers the whole footprint under the arm, so every
			// support is a branch or a tree whose foot stands outside the
			// model's bounding box in X or Y.
			require.Contains(t, []string{kindBranch, kindTree}, sr.Kind)
			require.NotNil(t, sr.FootMM)
			f := *sr.FootMM
			outside := f[0] > mb.Max.X || f[0] < mb.Min.X || f[1] > mb.Max.Y || f[1] < mb.Min.Y
			require.True(t, outside, "branch %d foot %v is under the model", i+1, f)
			require.True(t, res.Supports[i].IsSolid())
			require.Len(t, res.Supports[i].Lumps(), 1)
		}
		requirePassed(t, res.Report.Validation, "decad_support_lumps")
		trees, tips := 0, 0
		for _, sr := range res.Report.Supports {
			if sr.Kind == kindTree {
				trees++
				tips += len(sr.TipsMM)
				continue
			}
			tips++
		}
		require.Positive(t, trees)
		require.Greater(t, tips, len(res.Supports), "trees carry more tips than there are bodies")
	})

	t.Run("occluded overhang fails and adds no bodies without branches", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).Occluded)
		before := len(body.Document().Bodies())
		p := mtilt.ExampleProfile()
		p.MaxBranchLeanDeg = 0
		res, err := prepare(t, body, mtilt.Options{Profile: p, KeepOrientation: true})
		require.Nil(t, res)
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Nil(t, fe.Result)
		att := fe.Report.Candidates[0].Attempt
		require.Equal(t, mtilt.AttemptFailed, att.Status)
		require.Contains(t, att.Reason, "occluded")
		require.Len(t, body.Document().Bodies(), before)
	})

	t.Run("cancelled", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).Bracket)
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := mtilt.Prepare(ctx, body, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestPrepareBridges(t *testing.T) {
	t.Run("a 3 mm slot is bridged, so the upright part needs no supports", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Slot), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		att := res.Report.Candidates[0].Attempt
		require.Positive(t, att.BridgedSamples)
		require.Zero(t, att.Samples)
		require.Empty(t, res.Supports)
	})

	t.Run("without bridges the same slot cannot be supported", func(t *testing.T) {
		p := mtilt.ExampleProfile()
		p.MaxBridgeMM = 0
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Slot), mtilt.Options{Profile: p, KeepOrientation: true})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, fe.Report.Candidates[0].Attempt.Reason, "coverage gap")
	})

	t.Run("a 30 mm span still gets pillars under a 10 mm bridge limit", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Bridge), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.Zero(t, res.Report.Candidates[0].Attempt.BridgedSamples)
		require.NotEmpty(t, res.Supports)
	})
}

func TestPrepareStrength(t *testing.T) {
	t.Run("a standing stick is laid down", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Stick), mtilt.Options{})
		require.NoError(t, err)
		require.Greater(t, res.Report.Input.Elongation, 0.95)
		require.InDelta(t, 90, res.Report.Candidates[0].Metrics.LongAxisElevationDeg, 1e-6)
		require.False(t, res.Report.Candidates[0].TiltAllowed)
		require.InDelta(t, 0, selected(res).Metrics.LongAxisElevationDeg, 1e-6)
		require.InDelta(t, 8, selected(res).Metrics.HeightMM, 1e-6)
		require.Empty(t, res.Supports)
	})

	t.Run("a standing rod is laid down; its underside is plate-anchored", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Rod), mtilt.Options{})
		require.NoError(t, err)
		sel := selected(res)
		require.InDelta(t, 0, sel.Metrics.LongAxisElevationDeg, 1e-6)
		require.Zero(t, sel.Metrics.SupportDemandAreaMM2)
		require.Positive(t, sel.Metrics.AnchoredAreaMM2)
		require.Empty(t, res.Supports)
		require.Positive(t, res.Report.Input.TessellationBoundMM)
	})

	t.Run("a nail on its flange is laid down, tilted to save supports", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Nail), mtilt.Options{})
		require.NoError(t, err)
		requireNoFailedCheck(t, res.Report)
		sel := selected(res)
		require.Greater(t, sel.Metrics.LongAxisElevationDeg, 1.0)
		require.LessOrEqual(t, sel.Metrics.LongAxisElevationDeg, float64(mtilt.DefaultMaxLongAxisTiltDeg))
		require.NotEmpty(t, res.Supports)
		require.True(t, res.Report.Search.Constrained)
		for _, c := range res.Report.Candidates {
			if c.Planned != nil && c.ID != sel.ID {
				require.GreaterOrEqual(t, c.Planned.Score, sel.Planned.Score-1e-9, "candidate %d", c.ID)
			}
		}
	})

	t.Run("without the tilt limit the nail stands on its flange", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Nail), mtilt.Options{MaxLongAxisTiltDeg: 90})
		require.NoError(t, err)
		require.False(t, res.Report.Search.Constrained)
		require.InDelta(t, 90, selected(res).Metrics.LongAxisElevationDeg, 1e-6)
		require.Empty(t, res.Supports)
	})

	t.Run("every finalist respects the tilt limit", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).Nail), mtilt.Options{Profile: mtilt.ExampleProfile(), MaxLongAxisTiltDeg: 10})
		require.NoError(t, err)
		for _, c := range a.Report.Candidates[1:] {
			require.True(t, c.TiltAllowed)
			require.LessOrEqual(t, c.Metrics.LongAxisElevationDeg, 10+1e-9)
			require.GreaterOrEqual(t, c.Metrics.FirstLayerAreaMM2, mtilt.ExampleProfile().MinFirstLayerAreaMM2)
			require.Zero(t, c.Estimate.TooLowMM2)
		}
	})
}

// rankPlanned returns the planned candidates of rep in orient.Rank order:
// the order the lowest planned cost and the tie-breaks give, with no band.
func rankPlanned(rep mtilt.Report) []int {
	var entries []orient.Entry
	for _, c := range rep.Candidates {
		if c.Planned == nil {
			continue
		}
		entries = append(entries, orient.Entry{
			ID: c.ID, Feasible: true, Score: c.Planned.Score, FirstLayerAreaMM2: c.Metrics.FirstLayerAreaMM2,
			HeightMM: c.Metrics.HeightMM, ElevationDeg: c.Metrics.LongAxisElevationDeg,
		})
	}
	return orient.Rank(entries)
}

func TestPrepareSelection(t *testing.T) {
	t.Run("the nail's support band keeps ids 1 and 7, and id 1 wins", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Nail), mtilt.Options{})
		require.NoError(t, err)
		require.Equal(t, mtilt.DefaultSupportBand, res.Report.Objective.SupportBand)
		require.Equal(t, 1, *res.Report.Selected)
		notes := make([]string, len(res.Report.Candidates))
		for i, c := range res.Report.Candidates {
			notes[i] = c.SelectionNote
		}
		// Planned scores: id 1 0.00797504, ids 2, 3 and 7 under 1.05 times
		// that, ids 4 to 6 0.0117631. Id 7 has the band's largest first
		// layer, 62.77 mm2; ids 2 and 3 have under 75% of it.
		require.Equal(t, []string{
			"not planned: the long axis rises 90.0 degrees, over the 15.0 degree limit",
			"selected",
			"first layer 31.1 mm2 under 75% of 62.8 mm2",
			"first layer 26.4 mm2 under 75% of 62.8 mm2",
			"outside the support band: score 0.0117631 > 0.0083738",
			"outside the support band: score 0.0117631 > 0.0083738",
			"outside the support band: score 0.0117631 > 0.0083738",
			"lost to candidate 1 on score",
		}, notes)
	})

	t.Run("NoSupportBand selects what the lowest planned cost and the tie-breaks give", func(t *testing.T) {
		for _, f := range []shape{(*fixture.Bodies).Nail, (*fixture.Bodies).ObliqueCuboid, (*fixture.Bodies).Stick, (*fixture.Bodies).Bridge} {
			res, err := prepare(t, newBody(t, f), mtilt.Options{SupportBand: mtilt.NoSupportBand})
			require.NoError(t, err)
			require.Zero(t, res.Report.Objective.SupportBand)
			require.Equal(t, rankPlanned(res.Report)[0], *res.Report.Selected)
		}
	})

	t.Run("candidates past the attempt limit say so", func(t *testing.T) {
		// The upright bridge is allowed and ranks second by estimate, after
		// the bridge on its side. The search returns one finalist per
		// attempt, so only the allowed original can be left over.
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Bridge), mtilt.Options{Limits: mtilt.Limits{MaxSupportAttempts: 1}})
		require.NoError(t, err)
		require.True(t, res.Report.Search.AttemptLimitReached)
		require.Len(t, res.Report.Candidates, 2)
		require.Equal(t, 1, *res.Report.Selected)
		require.Equal(t, "selected", res.Report.Candidates[1].SelectionNote)
		orig := res.Report.Candidates[0]
		require.True(t, orig.TiltAllowed)
		require.Equal(t, mtilt.AttemptNotAttempted, orig.Attempt.Status)
		require.Equal(t, "not planned: support attempt limit reached", orig.SelectionNote)

		// The standing nail keeps its tilt reason under the same limit.
		res, err = prepare(t, newBody(t, (*fixture.Bodies).Nail), mtilt.Options{Limits: mtilt.Limits{MaxSupportAttempts: 1}})
		require.NoError(t, err)
		require.False(t, res.Report.Search.AttemptLimitReached)
		require.Equal(t, "not planned: the long axis rises 90.0 degrees, over the 15.0 degree limit", res.Report.Candidates[0].SelectionNote)
		require.Equal(t, "selected", selected(res).SelectionNote)
	})

	t.Run("a failed plan carries its reason", func(t *testing.T) {
		p := mtilt.ExampleProfile()
		p.MaxBridgeMM = 0
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Slot), mtilt.Options{Profile: p, KeepOrientation: true})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		c := fe.Report.Candidates[0]
		require.Contains(t, c.Attempt.Reason, "coverage gap")
		require.Equal(t, "plan failed: "+c.Attempt.Reason, c.SelectionNote)
	})

	t.Run("Analyze writes no notes", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).Nail), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.Equal(t, mtilt.DefaultSupportBand, a.Report.Objective.SupportBand)
		for _, c := range a.Report.Candidates {
			require.Empty(t, c.SelectionNote, "candidate %d", c.ID)
		}
		b, err := json.Marshal(a.Report.Candidates[0])
		require.NoError(t, err)
		require.NotContains(t, string(b), "selection_note")
	})
}

func TestPreparePlates(t *testing.T) {
	t.Run("the plate box lies flat; without the limit its on-edge finalists return", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).PlateBox), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.InDelta(t, 0.125, a.Report.Input.Flatness, 1e-9)
		require.True(t, a.Report.Objective.PlateLimited)
		require.Equal(t, float64(mtilt.DefaultMaxPlateTiltDeg), a.Report.Objective.MaxPlateTiltDeg)
		require.Positive(t, a.Report.Search.PlateExcluded)
		for _, c := range a.Report.Candidates {
			require.True(t, c.PlateAllowed, "candidate %d", c.ID)
			require.LessOrEqual(t, c.Metrics.PlateNormalTiltDeg, float64(mtilt.DefaultMaxPlateTiltDeg))
		}

		a, err = mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).PlateBox), mtilt.Options{Profile: mtilt.ExampleProfile(), MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		require.False(t, a.Report.Objective.PlateLimited)
		require.Zero(t, a.Report.Search.PlateExcluded)
		var onEdge int
		for _, c := range a.Report.Candidates {
			if math.Abs(c.Down[1]) > 1-1e-9 {
				onEdge++
				require.InDelta(t, 90, c.Metrics.PlateNormalTiltDeg, 1e-9)
			}
		}
		require.Equal(t, 2, onEdge, "d = -Y and d = +Y")
	})

	t.Run("the channel plate lies flat and its channel gets pillars", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).ChannelPlate)
		res, err := prepare(t, body, mtilt.Options{})
		require.NoError(t, err)
		requireNoFailedCheck(t, res.Report)
		require.InDelta(t, 0.129, res.Report.Input.Flatness, 5e-4)
		require.True(t, res.Report.Search.PlateLimited)
		sel := selected(res)
		require.True(t, sel.PlateAllowed)
		require.InDelta(t, 8, sel.Metrics.HeightMM, 1e-6)
		// X 0..20 and 40..120, 60 mm deep.
		require.InDelta(t, 6000, sel.Metrics.BedContactAreaMM2, 1e-6)
		require.NotEmpty(t, res.Supports)
		requireMovedCopy(t, body, res.Model, res.Transform)
	})

	t.Run("without the plate limit the channel plate stands on edge", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).ChannelPlate), mtilt.Options{MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		require.False(t, res.Report.Search.PlateLimited)
		require.InDelta(t, 60, selected(res).Metrics.HeightMM, 1e-6)
		require.Empty(t, res.Supports)
	})

	t.Run("the recess plate is too low flat and fails; without the limit it stands", func(t *testing.T) {
		body := newBody(t, (*fixture.Bodies).RecessPlate)
		before := len(body.Document().Bodies())
		res, err := prepare(t, body, mtilt.Options{})
		require.Nil(t, res)
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Nil(t, fe.Result)
		n := fe.Report.Search.PlateExcluded
		require.Positive(t, n)
		require.Contains(t, err.Error(), fmt.Sprintf("the plate tilt limit excluded %d", n))
		orig := fe.Report.Candidates[0].Attempt
		require.Equal(t, mtilt.AttemptFailed, orig.Status)
		require.Contains(t, orig.Reason, "too low")
		require.Len(t, body.Document().Bodies(), before)

		res, err = prepare(t, newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		require.InDelta(t, 60, selected(res).Metrics.HeightMM, 1e-6)
	})

	t.Run("the armed stick is both long and flat; every finalist meets both limits", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).ArmedStick), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.InDelta(t, 0.124, a.Report.Input.Flatness, 5e-4)
		require.True(t, a.Report.Objective.TiltLimited)
		require.True(t, a.Report.Objective.PlateLimited)
		require.Greater(t, len(a.Report.Candidates), 1)
		for _, c := range a.Report.Candidates[1:] {
			require.True(t, c.TiltAllowed, "candidate %d", c.ID)
			require.True(t, c.PlateAllowed, "candidate %d", c.ID)
			require.LessOrEqual(t, c.Metrics.LongAxisElevationDeg, float64(mtilt.DefaultMaxLongAxisTiltDeg))
			require.LessOrEqual(t, c.Metrics.PlateNormalTiltDeg, float64(mtilt.DefaultMaxPlateTiltDeg))
		}
	})

	t.Run("brackets and L-plates are not plates", func(t *testing.T) {
		for _, f := range []shape{(*fixture.Bodies).Bracket, (*fixture.Bodies).LPlate} {
			a, err := mtilt.Analyze(t.Context(), newBody(t, f), mtilt.Options{Profile: mtilt.ExampleProfile()})
			require.NoError(t, err)
			require.False(t, a.Report.Objective.PlateLimited, "flatness %v", a.Report.Input.Flatness)
			require.Zero(t, a.Report.Search.PlateExcluded)
		}
	})

	t.Run("KeepOrientation ignores the plate limit", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).PlateBox), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.True(t, selected(res).PlateAllowed)
		require.Zero(t, selected(res).Metrics.PlateNormalTiltDeg)
	})
}

// occlusionOf returns a candidate's measured sample counts and occluded grid
// area, which must be present.
func occlusionOf(t *testing.T, c mtilt.CandidateReport) (int, int, float64) {
	t.Helper()
	o := c.Occlusion
	require.NotNil(t, o.Samples, "candidate %d", c.ID)
	require.NotNil(t, o.OccludedSamples, "candidate %d", c.ID)
	require.NotNil(t, o.OccludedGridAreaMM2, "candidate %d", c.ID)
	return *o.Samples, *o.OccludedSamples, *o.OccludedGridAreaMM2
}

func TestPrepareOcclusion(t *testing.T) {
	t.Run("the slot's ceiling is occluded on its side and not on its face", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Slot), mtilt.Options{})
		require.NoError(t, err)
		var sides, faces int
		for _, c := range res.Report.Candidates {
			samples, occluded, area := occlusionOf(t, c)
			switch {
			case c.ID == 0:
				// Upright, the slot's ceiling is bridged.
				require.Zero(t, samples)
				require.Zero(t, occluded)
			case c.Down == [3]float64{1, 0, 0} || c.Down == [3]float64{-1, 0, 0}:
				sides++
				require.Equal(t, 631, occluded)
				require.Equal(t, c.Attempt.UncoveredCount, occluded, "the planner's count")
				require.Equal(t, c.Attempt.Samples, samples)
				// 630 distinct grid XY points at a 1 mm pitch: the lattice
				// includes the edges, so the area overcounts the 600 mm2
				// ceiling by at most half its perimeter times the pitch.
				require.InDelta(t, 630, area, 1e-9)
				require.InDelta(t, 600, area, 50)
			case c.Down == [3]float64{0, 1, 0} || c.Down == [3]float64{0, -1, 0}:
				faces++
				require.Zero(t, occluded)
				require.Zero(t, area)
			}
		}
		require.Equal(t, 2, sides)
		require.Equal(t, 2, faces)

		// Analyze measures the same candidates the same way.
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).Slot), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.Len(t, a.Report.Candidates, len(res.Report.Candidates))
		for i, c := range a.Report.Candidates {
			require.Equal(t, res.Report.Candidates[i].Occlusion, c.Occlusion, "candidate %d", c.ID)
		}
	})

	t.Run("branches cover all but one of the kept stacked shelves' occluded samples", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).StackedShelves), mtilt.Options{KeepOrientation: true})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		c := fe.Report.Candidates[0]
		samples, occluded, area := occlusionOf(t, c)
		require.Equal(t, 420, samples)
		require.Equal(t, c.Attempt.Samples, samples)
		// Every upper-shelf sample is occluded by the lower shelf.
		require.Equal(t, 210, occluded)
		require.InDelta(t, 210, area, 1e-9)
		// Branches reach under the upper shelf from beside the lower one
		// and cover all but one occluded sample.
		require.Equal(t, 1, c.Attempt.UncoveredCount)

		// Without branches no occluded sample is covered.
		noBranches := mtilt.ExampleProfile()
		noBranches.MaxBranchLeanDeg = 0
		_, err = prepare(t, newBody(t, (*fixture.Bodies).StackedShelves), mtilt.Options{KeepOrientation: true, Profile: noBranches})
		fe = requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		c = fe.Report.Candidates[0]
		_, occluded, _ = occlusionOf(t, c)
		require.Equal(t, 210, occluded)
		require.Equal(t, occluded, c.Attempt.UncoveredCount)
	})

	t.Run("the occluded part kept upright is occluded and plans fully", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Occluded), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		c := res.Report.Candidates[0]
		samples, occluded, _ := occlusionOf(t, c)
		require.Equal(t, 630, samples)
		require.Equal(t, 630, occluded)
		require.Equal(t, mtilt.AttemptSucceeded, c.Attempt.Status)
		require.Zero(t, c.Attempt.UncoveredCount)
	})

	t.Run("occlusion counts at least the planner's uncovered samples", func(t *testing.T) {
		// With the plate limit off, three finalists lie across the armed
		// stick's arms. Pillars beside the arms and branches cover most of
		// their occluded samples; only candidate 2 keeps one uncovered.
		res, err := prepare(t, newBody(t, (*fixture.Bodies).ArmedStick), mtilt.Options{MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		type counts struct{ occluded, uncovered int }
		got := map[int]counts{}
		failed := 0
		for _, c := range res.Report.Candidates {
			if c.Attempt.Status == mtilt.AttemptNotAttempted {
				continue
			}
			_, occluded, _ := occlusionOf(t, c)
			require.GreaterOrEqual(t, occluded, c.Attempt.UncoveredCount, "candidate %d", c.ID)
			got[c.ID] = counts{occluded, c.Attempt.UncoveredCount}
			if c.Attempt.Status == mtilt.AttemptFailed {
				failed++
			}
		}
		// The occluded counts differ by one sample between platforms (macOS
		// reports 64 and 67 for candidates 2 and 3), so they are checked
		// within one sample; the planner's uncovered counts match exactly.
		wantOccluded := map[int]int{1: 66, 2: 63, 3: 66, 4: 0, 5: 0}
		wantUncovered := map[int]int{1: 0, 2: 1, 3: 0, 4: 0, 5: 0}
		require.Len(t, got, len(wantOccluded))
		for id, want := range wantOccluded {
			require.InDelta(t, want, got[id].occluded, 1, "candidate %d occluded", id)
			require.Equal(t, wantUncovered[id], got[id].uncovered, "candidate %d uncovered", id)
		}
		require.Equal(t, 1, failed)
	})

	t.Run("over the sample limit the occlusion numbers are null", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Bracket), mtilt.Options{KeepOrientation: true, Limits: mtilt.Limits{MaxSamples: 10}})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		c := fe.Report.Candidates[0]
		require.Equal(t, support.OcclusionReport{}, c.Occlusion)
		b, err := json.Marshal(c.Occlusion)
		require.NoError(t, err)
		require.JSONEq(t, `{"samples":null,"occluded_samples":null,"occluded_grid_area_mm2":null}`, string(b))
	})
}

// requireNoSupportFreePose checks that no allowed orientation of the search
// needs no support. The finalists are the allowed directions with the
// lowest estimated cost, and refinement only lowers it, so a positive
// estimate on every allowed candidate holds for every allowed direction the
// search tried. Every plan that succeeded also placed supports.
func requireNoSupportFreePose(t *testing.T, rep mtilt.Report) {
	t.Helper()
	allowed := 0
	for _, c := range rep.Candidates {
		if !c.TiltAllowed || !c.PlateAllowed {
			continue
		}
		allowed++
		require.Positive(t, c.Estimate.ContactMM2, "candidate %d", c.ID)
		require.NotEqual(t, mtilt.AttemptNotNeeded, c.Attempt.Status, "candidate %d", c.ID)
		if c.Attempt.Status == mtilt.AttemptSucceeded {
			require.Positive(t, c.Attempt.Supports, "candidate %d", c.ID)
		}
	}
	require.Positive(t, allowed)
}

// requireSearchedPose checks the selected candidate of a search: its ID,
// its down direction, its height, and its long axis's elevation, which
// must also be within the default tilt limit.
func requireSearchedPose(t *testing.T, res *mtilt.Result, id int, height, elevation float64) mtilt.CandidateReport {
	t.Helper()
	require.Equal(t, "optimize", res.Report.Mode)
	sel := selected(res)
	require.Equal(t, id, sel.ID)
	require.Equal(t, [3]float64{0, 0, -1}, sel.Down)
	// The height is measured on the tessellation, which a union can leave
	// a fraction of a micrometer off the exact solid.
	require.InDelta(t, height, sel.Metrics.HeightMM, 1e-3)
	require.InDelta(t, elevation, sel.Metrics.LongAxisElevationDeg, 0.05)
	require.LessOrEqual(t, sel.Metrics.LongAxisElevationDeg, float64(mtilt.DefaultMaxLongAxisTiltDeg))
	requireNoFailedCheck(t, res.Report)
	requirePassed(t, res.Report.Validation, "decad_interference")
	requirePassed(t, res.Report.Validation, "decad_body_validity")
	return sel
}

func TestPrepareSearchedSupports(t *testing.T) {
	t.Run("the mounting bracket lies flat and pillars hold its hook's arm", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).MountingBracket), mtilt.Options{})
		require.NoError(t, err)
		require.True(t, res.Report.Objective.PlateLimited)
		require.Equal(t, 6, res.Report.Search.Allowed)
		requireNoSupportFreePose(t, res.Report)
		sel := requireSearchedPose(t, res, 0, 15, 5.4)
		require.Len(t, res.Supports, 46)
		require.Zero(t, sel.Attempt.BridgedSamples)
		mb, err := res.Model.Bounds()
		require.NoError(t, err)
		for _, sr := range res.Report.Supports {
			// Every pillar stands past the plate's edge at X = 70, under
			// the arm's underside at Z = 8.
			require.Equal(t, "pillar", sr.Kind)
			require.Greater(t, sr.CenterMM[0], mb.Min.X+70)
			require.InDelta(t, 8, sr.SurfaceZMM, 1e-6)
		}
	})

	t.Run("the bridge-ledge part lies flat; its 8 mm span is bridged and its ledge gets pillars", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).BridgeLedge), mtilt.Options{})
		require.NoError(t, err)
		require.True(t, res.Report.Objective.PlateLimited)
		require.Equal(t, 6, res.Report.Search.Allowed)
		requireNoSupportFreePose(t, res.Report)
		sel := requireSearchedPose(t, res, 0, 14, 8.0)
		require.Len(t, res.Supports, 61)
		require.Equal(t, 427, sel.Attempt.BridgedSamples)
		require.Equal(t, 183, sel.Attempt.HeldSamples)
		mb, err := res.Model.Bounds()
		require.NoError(t, err)
		for _, sr := range res.Report.Supports {
			// No pillar stands in the span (X 60..68); all of them are
			// past the leg's outer face at X = 72, under the ledge.
			require.Equal(t, "pillar", sr.Kind)
			require.Greater(t, sr.CenterMM[0], mb.Min.X+72)
			require.InDelta(t, 10, sr.SurfaceZMM, 1e-6)
		}
	})

	t.Run("the housing stays upright and branches reach under its top", func(t *testing.T) {
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Housing), mtilt.Options{})
		require.NoError(t, err)
		require.True(t, res.Report.Objective.TiltLimited)
		require.False(t, res.Report.Objective.PlateLimited)
		require.Equal(t, 5, res.Report.Search.Allowed)
		requireNoSupportFreePose(t, res.Report)
		sel := requireSearchedPose(t, res, 0, 38, 8.3)
		requirePassed(t, res.Report.Validation, "decad_support_lumps")
		_, occluded, _ := occlusionOf(t, sel)
		require.InDelta(t, 800, occluded, 1)
		require.Zero(t, sel.Attempt.UncoveredCount)
		require.Equal(t, 47, sel.Attempt.Supports)

		mb, err := res.Model.Bounds()
		require.NoError(t, err)
		kinds := map[string]int{}
		tips := 0
		for i, sr := range res.Report.Supports {
			// The base lies under the whole top, so every support's foot
			// stands outside the model's bounding box.
			require.Contains(t, []string{kindBranch, kindTree}, sr.Kind)
			require.NotNil(t, sr.FootMM)
			f := *sr.FootMM
			outside := f[0] > mb.Max.X || f[0] < mb.Min.X || f[1] > mb.Max.Y || f[1] < mb.Min.Y
			require.True(t, outside, "support %d foot %v is under the model", i+1, f)
			kinds[sr.Kind]++
			if sr.Kind == kindTree {
				tips += len(sr.TipsMM)
				continue
			}
			tips++
		}
		require.Equal(t, map[string]int{kindBranch: 17, kindTree: 13}, kinds)
		require.Equal(t, sel.Attempt.Supports, tips)
	})
}

func TestPrepareInputErrors(t *testing.T) {
	t.Run("two lumps", func(t *testing.T) {
		fb := fixture.NewBodies()
		a, err := fb.Box(t.Context(), r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
		require.NoError(t, err)
		b, err := fb.Box(t.Context(), r3.NewVec(20, 0, 0), r3.NewVec(30, 10, 10))
		require.NoError(t, err)
		both, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		_, err = prepare(t, both, mtilt.Options{})
		fe := requireFailure(t, err, mtilt.ErrUnsupportedInput)
		require.Equal(t, 2, fe.Report.Input.Lumps)
		require.Empty(t, fe.Report.Candidates)
	})

	t.Run("nil body", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), nil, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("missing profile", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), newBody(t, (*fixture.Bodies).Cube), mtilt.Options{})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("negative cost weight", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{CostWeights: mtilt.CostWeights{Volume: -1}})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("tilt limit out of range", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{MaxLongAxisTiltDeg: 91})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("plate tilt limit out of range", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{MaxPlateTiltDeg: -1})
		require.ErrorContains(t, err, "plate tilt limit")
	})

	t.Run("support band out of range", func(t *testing.T) {
		for _, band := range []float64{0.6, -0.5, math.NaN()} {
			_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{SupportBand: band})
			require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
			require.ErrorContains(t, err, "support band")
		}
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{SupportBand: mtilt.MaxSupportBand})
		require.NoError(t, err)
		require.Equal(t, mtilt.MaxSupportBand, res.Report.Objective.SupportBand)
	})

	t.Run("first-layer floor nothing meets", func(t *testing.T) {
		p := mtilt.ExampleProfile()
		p.MinFirstLayerAreaMM2 = 1000
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{Profile: p})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, fe.Report.Candidates[0].Attempt.Reason, "first layer area")
	})

	t.Run("triangle limit", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{Limits: mtilt.Limits{MaxTriangles: 11}})
		require.ErrorIs(t, err, mtilt.ErrLimit)
	})
}

func TestPrepareLimitsAndVolume(t *testing.T) {
	t.Run("build volume", func(t *testing.T) {
		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 30, YMM: 30, ZMM: 30, MarginMM: 2}
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{Profile: p})
		require.NoError(t, err)
		require.Equal(t, "fits", selected(res).Fit)

		p.BuildVolume = &mtilt.BuildVolume{XMM: 22, YMM: 22, ZMM: 30, MarginMM: 2}
		_, err = prepare(t, newBody(t, (*fixture.Bodies).Cube), mtilt.Options{Profile: p})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		for _, c := range fe.Report.Candidates {
			require.Equal(t, "exceeds", c.Fit)
		}
		require.Zero(t, fe.Report.Search.SupportAttempts)
	})

	t.Run("the height limit tilts the nail less to fit a 30.1 mm volume", func(t *testing.T) {
		// Without a volume the nail stands 104 mm tall as given, and its
		// seven finalists, tilted about 4.6 degrees, are 30.22 to 30.50 mm
		// tall: none would fit.
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).Nail), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.Len(t, a.Report.Candidates, 8)
		require.InDelta(t, 104, a.Report.Candidates[0].Metrics.HeightMM, 1e-6)
		for _, c := range a.Report.Candidates[1:] {
			require.Greater(t, c.Metrics.HeightMM, 30.2, "candidate %d", c.ID)
			require.Less(t, c.Metrics.HeightMM, 30.51, "candidate %d", c.ID)
		}

		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 200, YMM: 200, ZMM: 30.1}
		res, err := prepare(t, newBody(t, (*fixture.Bodies).Nail), mtilt.Options{Profile: p})
		require.NoError(t, err)
		requireNoFailedCheck(t, res.Report)
		rep := res.Report
		allow := 30.1 + rep.Tolerances.LengthMM + rep.Input.TessellationBoundMM
		// The search finds the four turns of one pose, tilted 1.42 degrees
		// about X or Y, 30.0899 mm tall.
		require.Len(t, rep.Candidates, 5)
		require.Equal(t, "exceeds", rep.Candidates[0].Fit)
		for _, c := range rep.Candidates[1:] {
			require.LessOrEqual(t, c.Metrics.HeightMM, allow, "candidate %d", c.ID)
			require.InDelta(t, 30.08987, c.Metrics.HeightMM, 1e-5, "candidate %d", c.ID)
			require.Equal(t, "fits", c.Fit, "candidate %d", c.ID)
		}
		sel := selected(res)
		require.Equal(t, 1, sel.ID)
		require.InDelta(t, 1.41926, sel.Metrics.LongAxisElevationDeg, 1e-5)
		require.Len(t, res.Supports, 75)
	})

	t.Run("the height limit drops the recess plate's on-edge directions", func(t *testing.T) {
		// With no volume, the plate limit excludes 8 directions, all near
		// d = +-Y, where the plate stands 60 to 61.1 mm tall.
		a, err := mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.Equal(t, 8, a.Report.Search.PlateExcluded)

		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 200, YMM: 200, ZMM: 59}
		a, err = mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{Profile: p})
		require.NoError(t, err)
		require.Zero(t, a.Report.Search.PlateExcluded)

		// With the plate limit off, those 8 directions are allowed without a
		// volume, and none is allowed in the 59 mm volume.
		a, err = mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{Profile: mtilt.ExampleProfile(), MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		require.Equal(t, 8, a.Report.Search.Allowed)
		for _, c := range a.Report.Candidates[1:] {
			require.Greater(t, math.Abs(c.Down[1]), 0.99, "candidate %d", c.ID)
			require.Greater(t, c.Metrics.HeightMM, 59.0, "candidate %d", c.ID)
		}
		a, err = mtilt.Analyze(t.Context(), newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{Profile: p, MaxPlateTiltDeg: 90})
		require.NoError(t, err)
		require.Zero(t, a.Report.Search.Allowed)
		require.Len(t, a.Report.Candidates, 1, "only the original")

		_, err = prepare(t, newBody(t, (*fixture.Bodies).RecessPlate), mtilt.Options{Profile: p})
		requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.NotContains(t, err.Error(), "plate tilt limit excluded")
	})

	t.Run("supports in the volume margin disqualify the candidate", func(t *testing.T) {
		// The bracket alone is 40 x 20; a pillar base reaches 1.5 mm past
		// the arm's edge, so a 44 x 24 volume with a 2 mm margin holds the
		// model but not the supports.
		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 44, YMM: 24, ZMM: 50, MarginMM: 2}
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Bracket), mtilt.Options{Profile: p, KeepOrientation: true})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, fe.Report.Candidates[0].Attempt.Reason, "exceed the build volume")
	})

	t.Run("support limit fails the attempt", func(t *testing.T) {
		_, err := prepare(t, newBody(t, (*fixture.Bodies).Bracket), mtilt.Options{KeepOrientation: true, Limits: mtilt.Limits{MaxSupports: 5}})
		fe := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, fe.Report.Candidates[0].Attempt.Reason, "work limit")
	})
}

func TestPrepareDeterminism(t *testing.T) {
	encode := func(r mtilt.Report) []byte {
		b, err := json.Marshal(r)
		require.NoError(t, err)
		return b
	}
	a, err := prepare(t, newBody(t, (*fixture.Bodies).Bracket), mtilt.Options{KeepOrientation: true})
	require.NoError(t, err)
	b, err := prepare(t, newBody(t, (*fixture.Bodies).Bracket), mtilt.Options{KeepOrientation: true})
	require.NoError(t, err)
	require.Equal(t, encode(a.Report), encode(b.Report))
	require.NotContains(t, string(encode(a.Report)), "-0,")
}

func TestTransformReport(t *testing.T) {
	body := newBody(t, (*fixture.Bodies).ObliqueCuboid)
	res, err := prepare(t, body, mtilt.Options{})
	require.NoError(t, err)
	tr := res.Report.Transform
	apply := func(m [4][4]float64, p r3.Vec) r3.Vec {
		v := [3]float64{p.X, p.Y, p.Z}
		var q [3]float64
		for i := range 3 {
			q[i] = m[i][0]*v[0] + m[i][1]*v[1] + m[i][2]*v[2] + m[i][3]
		}
		return r3.NewVec(q[0], q[1], q[2])
	}
	for _, v := range body.Vertices() {
		p := v.Position().Value
		q := apply(tr.Matrix, p)
		require.True(t, q.Equal(res.Transform.Apply(p), 1e-9))
		require.True(t, apply(tr.Inverse, q).Equal(p, 1e-9))
	}
	require.Equal(t, [4]float64{0, 0, 0, 1}, tr.Matrix[3])
}

func TestAnalyze(t *testing.T) {
	body := newBody(t, (*fixture.Bodies).Bracket)
	before := len(body.Document().Bodies())
	a, err := mtilt.Analyze(t.Context(), body, mtilt.Options{Profile: mtilt.ExampleProfile()})
	require.NoError(t, err)
	require.Len(t, a.Ranking, len(a.Report.Candidates))
	for i, id := range a.Ranking {
		c := a.Report.Candidates[id]
		require.Equal(t, i+1, c.Rank)
		require.Equal(t, mtilt.AttemptNotAttempted, c.Attempt.Status)
	}
	require.Nil(t, a.Report.Selected)
	require.Len(t, body.Document().Bodies(), before, "Analyze adds no bodies")
	require.True(t, slices.ContainsFunc(a.Report.Unchecked, func(u mtilt.Unchecked) bool { return u.Property == "layer_strength" }))
}
