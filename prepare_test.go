package mtilt_test

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireFailure(t *testing.T, err error, target error) *mtilt.Report {
	t.Helper()
	require.ErrorIs(t, err, target)
	var fe *mtilt.FailureError
	require.ErrorAs(t, err, &fe)
	require.NotNil(t, fe.Report)
	return fe.Report
}

func requireValidationPassed(t *testing.T, rep mtilt.Report) {
	t.Helper()
	for _, c := range rep.Validation {
		require.NotEqual(t, mesh.StatusFailed, c.Status, "%s: %s", c.Name, c.Detail)
	}
}

// requireRigidCopy checks that out is in, moved by tr, vertex for vertex.
func requireRigidCopy(t *testing.T, in, out *mesh.Mesh, tr r3.Transform, scale float64) {
	t.Helper()
	require.False(t, tr.IsReflection())
	require.Equal(t, in.Triangles, out.Triangles)
	for i, v := range in.Vertices {
		require.True(t, tr.Apply(v.Scale(scale)).Equal(out.Vertices[i], 1e-9))
	}
	require.InDelta(t, in.Volume()*scale*scale*scale, out.Volume(), 1e-6)
}

func sortedSize(b mesh.Box) []float64 {
	s := b.Size()
	out := []float64{s.X, s.Y, s.Z}
	slices.Sort(out)
	return out
}

func TestPrepare(t *testing.T) {
	t.Run("cube needs no supports and keeps its orientation", func(t *testing.T) {
		in := loadFixture(t, "cube.stl")
		res, err := prepare(t, in, mtilt.Options{})
		require.NoError(t, err)
		require.Empty(t, res.Supports)
		require.Empty(t, res.Report.Supports)
		require.Equal(t, 0, *res.Report.Selected)
		require.Equal(t, mtilt.AttemptNotNeeded, res.Report.Candidates[0].Attempt.Status)
		require.True(t, res.Transform.Basis() == r3.Identity().Basis())
		require.Equal(t, [3]float64{-10, -10, 0}, res.Report.Transform.Translation)
		requireRigidCopy(t, in, res.Model, res.Transform, 1)
		requireValidationPassed(t, res.Report)
	})

	t.Run("oblique cuboid is turned onto its largest face", func(t *testing.T) {
		in := loadFixture(t, "oblique-cuboid.stl")
		res, err := prepare(t, in, mtilt.Options{})
		require.NoError(t, err)
		sel := res.Report.Candidates[*res.Report.Selected]
		require.Equal(t, "planar_face", sel.Source)
		require.Equal(t, 1, sel.Rank)
		require.Zero(t, sel.Metrics.SupportDemandAreaMM2)
		require.InDelta(t, 800, sel.Metrics.BedContactAreaMM2, 1e-3)
		require.InDelta(t, 10, sel.Metrics.HeightMM, 1e-4)
		require.Empty(t, res.Supports)

		orig := res.Report.Candidates[0]
		require.Equal(t, "original", orig.Source)
		require.Positive(t, orig.Metrics.SupportDemandAreaMM2)
		require.Greater(t, orig.Score, sel.Score)

		requireRigidCopy(t, in, res.Model, res.Transform, 1)
		// Planar-face candidates get no yaw alignment, so only the height
		// of the bounding box is a cuboid dimension.
		require.InDelta(t, 10, res.Model.Bounds().Size().Z, 1e-4)
		requireValidationPassed(t, res.Report)
	})

	t.Run("fixed-orientation bracket gets real supports", func(t *testing.T) {
		in := loadFixture(t, "bracket.stl")
		res, err := prepare(t, in, mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.Equal(t, "keep_orientation", res.Report.Mode)
		require.Len(t, res.Report.Candidates, 1)
		require.NotEmpty(t, res.Supports)
		require.Len(t, res.Report.Supports, len(res.Supports))
		require.True(t, res.Transform.Basis() == r3.Identity().Basis(), "rotation kept")
		requireRigidCopy(t, in, res.Model, res.Transform, 1)
		requireValidationPassed(t, res.Report)

		mb := res.Model.Bounds()
		for i, s := range res.Supports {
			sr := res.Report.Supports[i]
			require.Equal(t, fmt.Sprintf("support-%04d", i+1), sr.ID)
			b := s.Bounds()
			require.Zero(t, b.Min.Z)
			// Same frame as the model: each pillar stands under the arm.
			require.InDelta(t, 30-0.2, b.Max.Z, 1e-3)
			require.GreaterOrEqual(t, sr.CenterMM[0], mb.Min.X+10)
			require.LessOrEqual(t, sr.CenterMM[0], mb.Max.X)
			require.InDelta(t, sr.VolumeMM3, s.Volume(), 1e-9)
		}
	})

	t.Run("fixed-orientation bridge gets supports between its posts only", func(t *testing.T) {
		res, err := prepare(t, loadFixture(t, "bridge.stl"), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.NotEmpty(t, res.Supports)
		for _, sr := range res.Report.Supports {
			require.Greater(t, sr.CenterMM[0], -15.0)
			require.Less(t, sr.CenterMM[0], 15.0)
		}
		requireValidationPassed(t, res.Report)
	})

	t.Run("occluded overhang fails explicitly", func(t *testing.T) {
		res, err := prepare(t, loadFixture(t, "occluded.stl"), mtilt.Options{KeepOrientation: true})
		require.Nil(t, res)
		rep := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Nil(t, rep.Selected)
		require.Nil(t, rep.Transform)
		att := rep.Candidates[0].Attempt
		require.Equal(t, mtilt.AttemptFailed, att.Status)
		require.Contains(t, att.Reason, "coverage gap")
		require.Contains(t, att.Reason, "occluded")
		require.Equal(t, att.Samples, att.UncoveredCount)
	})

	t.Run("occluded part succeeds when allowed to rotate", func(t *testing.T) {
		res, err := prepare(t, loadFixture(t, "occluded.stl"), mtilt.Options{})
		require.NoError(t, err)
		require.NotEqual(t, 0, *res.Report.Selected)
	})

	t.Run("unit conversion happens once", func(t *testing.T) {
		in := loadFixture(t, "cube.stl")
		res, err := mtilt.Prepare(t.Context(), mtilt.Input{Mesh: in, Unit: mtilt.UnitInch}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.InDelta(t, 25.4, res.Report.Input.ScaleToMM, 0)
		require.InDelta(t, 25.4, res.Report.Transform.ScaleToMM, 0)
		requireRigidCopy(t, in, res.Model, res.Transform, 25.4)
		require.InDeltaSlice(t, []float64{508, 508, 508}, sortedSize(res.Model.Bounds()), 1e-9)
	})

	t.Run("build volume", func(t *testing.T) {
		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 30, YMM: 30, ZMM: 30, MarginMM: 2}
		res, err := prepare(t, loadFixture(t, "cube.stl"), mtilt.Options{Profile: p})
		require.NoError(t, err)
		require.Equal(t, "fits", res.Report.Candidates[*res.Report.Selected].Fit)
		requireCheck(t, res.Report.Validation, "assembly_build_volume_fit", mesh.StatusPassed)

		p.BuildVolume = &mtilt.BuildVolume{XMM: 22, YMM: 22, ZMM: 30, MarginMM: 2}
		_, err = prepare(t, loadFixture(t, "cube.stl"), mtilt.Options{Profile: p})
		rep := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		for _, c := range rep.Candidates {
			require.Equal(t, "exceeds", c.Fit)
			require.Equal(t, mtilt.AttemptNotAttempted, c.Attempt.Status)
		}
		require.Zero(t, rep.Search.SupportAttempts)
	})

	t.Run("supports in the volume margin disqualify the assembly", func(t *testing.T) {
		// The bracket alone is 40 x 20; a pillar base reaches 1.5 mm past
		// the arm's edge, so a 44 x 24 volume with a 2 mm margin holds the
		// model but not the assembly.
		p := mtilt.ExampleProfile()
		p.BuildVolume = &mtilt.BuildVolume{XMM: 44, YMM: 24, ZMM: 50, MarginMM: 2}
		_, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{Profile: p, KeepOrientation: true})
		rep := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, rep.Candidates[0].Attempt.Reason, "assembly_build_volume_fit")
	})

	t.Run("no build volume leaves fit unchecked", func(t *testing.T) {
		res, err := prepare(t, loadFixture(t, "cube.stl"), mtilt.Options{})
		require.NoError(t, err)
		require.Equal(t, "unchecked", res.Report.Candidates[0].Fit)
		requireCheck(t, res.Report.Validation, "assembly_build_volume_fit", mesh.StatusUnchecked)
		require.True(t, slices.ContainsFunc(res.Report.Unchecked, func(u mtilt.Unchecked) bool { return u.Property == "build_volume_fit" }))
	})

	t.Run("attempt limit is recorded", func(t *testing.T) {
		_, err := prepare(t, loadFixture(t, "occluded.stl"), mtilt.Options{
			KeepOrientation: true,
			Limits:          mtilt.Limits{MaxSupportAttempts: 1},
		})
		rep := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Equal(t, 1, rep.Search.SupportAttempts)
		require.False(t, rep.Search.AttemptLimitReached, "the only candidate was tried")
	})

	t.Run("support limit fails the attempt", func(t *testing.T) {
		_, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{KeepOrientation: true, Limits: mtilt.Limits{MaxSupports: 5}})
		rep := requireFailure(t, err, mtilt.ErrNoFeasibleCandidate)
		require.Contains(t, rep.Candidates[0].Attempt.Reason, "work limit")
	})

	t.Run("triangle limit", func(t *testing.T) {
		_, err := prepare(t, loadFixture(t, "cube.stl"), mtilt.Options{Limits: mtilt.Limits{MaxTriangles: 11}})
		require.ErrorIs(t, err, mtilt.ErrLimit)
	})

	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := mtilt.Prepare(ctx, mtilt.Input{Mesh: loadFixture(t, "bracket.stl"), Unit: mtilt.UnitMillimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, context.Canceled)
	})
}

func requireCheck(t *testing.T, checks []mesh.Check, name string, want mesh.Status) {
	t.Helper()
	for _, c := range checks {
		if c.Name == name {
			require.Equal(t, want, c.Status, "%s: %s", c.Name, c.Detail)
			return
		}
	}
	t.Fatalf("no check %q", name)
}

func TestPrepareInputErrors(t *testing.T) {
	cases := []struct {
		file string
		want error
	}{
		{"open-mesh.stl", mtilt.ErrInvalidMesh},
		{"inconsistent-winding.stl", mtilt.ErrInvalidMesh},
		{"two-components.stl", mtilt.ErrUnsupportedInput},
	}
	for _, tc := range cases {
		t.Run(tc.file, func(t *testing.T) {
			_, err := prepare(t, loadFixture(t, tc.file), mtilt.Options{})
			rep := requireFailure(t, err, tc.want)
			require.Empty(t, rep.Candidates)
			require.NotEmpty(t, rep.Unchecked)
		})
	}

	t.Run("nil mesh", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), mtilt.Input{Unit: mtilt.UnitMillimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("unknown unit", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), mtilt.Input{Mesh: loadFixture(t, "cube.stl"), Unit: "furlong"}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("missing unit", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), mtilt.Input{Mesh: loadFixture(t, "cube.stl")}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("missing profile", func(t *testing.T) {
		_, err := mtilt.Prepare(t.Context(), mtilt.Input{Mesh: loadFixture(t, "cube.stl"), Unit: mtilt.UnitMillimeter}, mtilt.Options{})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	t.Run("negative weight", func(t *testing.T) {
		_, err := prepare(t, loadFixture(t, "cube.stl"), mtilt.Options{Weights: mtilt.Weights{Height: -1}})
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})
}

func TestPrepareDeterminism(t *testing.T) {
	encode := func(r mtilt.Report) []byte {
		b, err := json.Marshal(r)
		require.NoError(t, err)
		return b
	}

	t.Run("same input, same report", func(t *testing.T) {
		a, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		b, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		require.Equal(t, encode(a.Report), encode(b.Report))
	})

	for _, name := range []string{"oblique-cuboid.stl", "bracket.stl"} {
		t.Run("triangle order does not change the result: "+name, func(t *testing.T) {
			in := loadFixture(t, name)
			soup := in.Soup()
			rng := rand.New(rand.NewPCG(5, 6))
			rng.Shuffle(len(soup), func(i, j int) { soup[i], soup[j] = soup[j], soup[i] })
			for _, keep := range []bool{false, true} {
				a, errA := prepare(t, in, mtilt.Options{KeepOrientation: keep})
				b, errB := prepare(t, soupMesh(t, soup), mtilt.Options{KeepOrientation: keep})
				if errA != nil {
					// The oblique cuboid stands on a vertex as given, so its
					// lowest overhang is too close to the plate.
					require.EqualError(t, errB, errA.Error())
					continue
				}
				require.NoError(t, errB)
				require.Equal(t, *a.Report.Selected, *b.Report.Selected)
				require.True(t, a.Transform.Equal(b.Transform, 1e-9))
				require.Len(t, b.Report.Supports, len(a.Report.Supports))
				for i := range a.Report.Supports {
					require.Equal(t, a.Report.Supports[i].CenterMM, b.Report.Supports[i].CenterMM)
				}
			}
		})
	}

	t.Run("equivalent triangulation does not change the result", func(t *testing.T) {
		a, err := prepare(t, soupMesh(t, fixture.Cube()), mtilt.Options{})
		require.NoError(t, err)
		b, err := prepare(t, loadFixture(t, "cube-split-faces.stl"), mtilt.Options{})
		require.NoError(t, err)
		require.Equal(t, *a.Report.Selected, *b.Report.Selected)
		require.True(t, a.Transform.Equal(b.Transform, 1e-12))
		require.Empty(t, b.Supports)
		require.InDelta(t, a.Report.Candidates[0].Score, b.Report.Candidates[0].Score, 1e-12)
	})

	t.Run("plan JSON has no negative zero and no timing", func(t *testing.T) {
		res, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{KeepOrientation: true})
		require.NoError(t, err)
		b := encode(res.Report)
		require.NotContains(t, string(b), "-0,")
		require.NotContains(t, string(b), "elapsed")
		require.NotContains(t, string(b), "time")
	})
}

func TestTransformReport(t *testing.T) {
	in := loadFixture(t, "oblique-cuboid.stl")
	res, err := mtilt.Prepare(t.Context(), mtilt.Input{Mesh: in, Unit: mtilt.UnitCentimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
	require.NoError(t, err)
	tr := res.Report.Transform

	apply := func(m [4][4]float64, p [3]float64) [3]float64 {
		var q [3]float64
		for i := range 3 {
			q[i] = m[i][0]*p[0] + m[i][1]*p[1] + m[i][2]*p[2] + m[i][3]
		}
		return q
	}
	for i, v := range in.Vertices {
		p := [3]float64{v.X * tr.ScaleToMM, v.Y * tr.ScaleToMM, v.Z * tr.ScaleToMM}
		q := apply(tr.Matrix, p)
		out := res.Model.Vertices[i]
		require.InDelta(t, out.X, q[0], 1e-9)
		require.InDelta(t, out.Y, q[1], 1e-9)
		require.InDelta(t, out.Z, q[2], 1e-9)
		back := apply(tr.Inverse, q)
		for k := range 3 {
			require.InDelta(t, p[k], back[k], 1e-9)
		}
	}

	// The rotation block is proper: orthonormal rows, determinant +1.
	m := tr.Matrix
	det := m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
	require.InDelta(t, 1, det, 1e-12)
	require.Equal(t, [4]float64{0, 0, 0, 1}, m[3])
}

func TestValidateSerialized(t *testing.T) {
	res, err := prepare(t, loadFixture(t, "bracket.stl"), mtilt.Options{KeepOrientation: true})
	require.NoError(t, err)

	roundTrip := func(m *mesh.Mesh) *mesh.Mesh {
		var buf bytes.Buffer
		require.NoError(t, stl.WriteBinary(&buf, m.Soup(), "test"))
		f, err := stl.Read(&buf, stl.Limits{})
		require.NoError(t, err)
		return soupMesh(t, f.Triangles)
	}
	model := roundTrip(res.Model)
	supports := make([]*mesh.Mesh, len(res.Supports))
	for i, s := range res.Supports {
		supports[i] = roundTrip(s)
	}

	checks, err := mtilt.ValidateSerialized(t.Context(), res, model, supports)
	require.NoError(t, err)
	for _, c := range checks {
		require.NotEqual(t, mesh.StatusFailed, c.Status, "%s: %s", c.Name, c.Detail)
	}
	requireCheck(t, checks, "serialized_support_top_gap", mesh.StatusPassed)

	t.Run("moved model is caught", func(t *testing.T) {
		shift, err := r3.Translation(r3.NewVec(0.01, 0, 0))
		require.NoError(t, err)
		checks, err := mtilt.ValidateSerialized(t.Context(), res, model.Transformed(shift), supports)
		require.NoError(t, err)
		requireCheck(t, checks, "serialized_model_matches", mesh.StatusFailed)
	})

	t.Run("missing support is caught", func(t *testing.T) {
		checks, err := mtilt.ValidateSerialized(t.Context(), res, model, supports[1:])
		require.NoError(t, err)
		requireCheck(t, checks, "serialized_supports_match", mesh.StatusFailed)
	})
}

func TestAnalyze(t *testing.T) {
	t.Run("ranks candidates without building supports", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), mtilt.Input{Mesh: loadFixture(t, "bracket.stl"), Unit: mtilt.UnitMillimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.NoError(t, err)
		require.Len(t, a.Ranking, len(a.Report.Candidates))
		for i, id := range a.Ranking {
			c := a.Report.Candidates[id]
			require.Equal(t, i+1, c.Rank)
			require.Equal(t, mtilt.AttemptNotAttempted, c.Attempt.Status)
			if i > 0 {
				require.GreaterOrEqual(t, c.Score, a.Report.Candidates[a.Ranking[i-1]].Score-1e-9)
			}
		}
		require.Nil(t, a.Report.Selected)
	})

	t.Run("invalid mesh still returns the checks", func(t *testing.T) {
		a, err := mtilt.Analyze(t.Context(), mtilt.Input{Mesh: loadFixture(t, "open-mesh.stl"), Unit: mtilt.UnitMillimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		require.ErrorIs(t, err, mtilt.ErrInvalidMesh)
		require.NotNil(t, a)
		requireCheck(t, a.Report.Validation, "input_closed", mesh.StatusFailed)
		requireCheck(t, a.Report.Validation, "input_self_intersection", mesh.StatusUnchecked)
	})
}

func FuzzAnalyze(f *testing.F) {
	f.Add([]byte{0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10})
	f.Fuzz(func(t *testing.T, data []byte) {
		// Up to 64 triangles with small integer coordinates: enough to
		// form closed and broken shapes without exceeding any limit.
		var soup [][3]r3.Vec
		for i := 0; i+9 <= len(data) && len(soup) < 64; i += 9 {
			var tri [3]r3.Vec
			for k := range 3 {
				b := data[i+3*k : i+3*k+3]
				tri[k] = r3.NewVec(float64(b[0]%16), float64(b[1]%16), float64(b[2]%16))
			}
			soup = append(soup, tri)
		}
		m, err := mesh.FromSoup(soup)
		if err != nil {
			return
		}
		a, err := mtilt.Analyze(t.Context(), mtilt.Input{Mesh: m, Unit: mtilt.UnitMillimeter}, mtilt.Options{Profile: mtilt.ExampleProfile()})
		if err != nil {
			require.True(t, errors.Is(err, mtilt.ErrInvalidMesh) || errors.Is(err, mtilt.ErrUnsupportedInput), "unexpected error: %v", err)
			return
		}
		for _, c := range a.Report.Candidates {
			require.False(t, math.IsNaN(c.Score))
		}
	})
}
