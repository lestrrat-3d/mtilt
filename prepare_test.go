package mtilt_test

import (
	"context"
	"encoding/json"
	"fmt"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type shape func(*fixture.Bodies, context.Context) (*decad.Body, error)

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
			require.Contains(t, []string{"branch", "tree"}, sr.Kind)
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
			if sr.Kind == "tree" {
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
