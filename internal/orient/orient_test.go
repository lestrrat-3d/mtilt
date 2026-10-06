package orient_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var limits = orient.Limits{MaxCandidates: 64, MaxPlanarFaces: 12}

func build(t *testing.T, soup [][3]r3.Vec) *mesh.Mesh {
	t.Helper()
	m, err := mesh.FromSoup(soup)
	require.NoError(t, err)
	return m
}

func TestGenerate(t *testing.T) {
	t.Run("axis-aligned set is the 24 proper rotations", func(t *testing.T) {
		cands, gen, err := orient.Generate(t.Context(), build(t, fixture.Cube()), limits)
		require.NoError(t, err)
		// Every planar face of a cube is axis-aligned, so its face-down
		// rotation duplicates an axis-aligned one and is dropped.
		require.Len(t, cands, 24)
		require.Equal(t, 24, gen.Distinct)
		require.False(t, gen.Truncated)
		require.Equal(t, orient.SourceOriginal, cands[0].Source)
		for i, c := range cands {
			require.Equal(t, i, c.ID)
			require.False(t, c.Rotation.IsReflection())
			for _, prev := range cands[:i] {
				require.False(t, prev.Rotation.Equal(c.Rotation, 1e-9), "candidate %d repeats", i)
			}
		}
	})

	t.Run("planar candidates turn dominant faces down, largest first", func(t *testing.T) {
		m := build(t, fixture.ObliqueCuboid())
		cands, _, err := orient.Generate(t.Context(), m, limits)
		require.NoError(t, err)
		var planar []orient.Candidate
		for _, c := range cands {
			if c.Source == orient.SourcePlanarFace {
				planar = append(planar, c)
			}
		}
		require.Len(t, planar, 6)
		down := r3.NewVec(0, 0, -1)
		for i, c := range planar {
			require.True(t, c.Rotation.ApplyDir(c.FaceNormal).Equal(down, 1e-9))
			if i > 0 {
				require.GreaterOrEqual(t, planar[i-1].FaceAreaMM2, c.FaceAreaMM2)
			}
		}
		require.InDelta(t, 800, planar[0].FaceAreaMM2, 1e-3)
	})

	t.Run("candidate limit truncates and reports it", func(t *testing.T) {
		cands, gen, err := orient.Generate(t.Context(), build(t, fixture.ObliqueCuboid()), orient.Limits{MaxCandidates: 5, MaxPlanarFaces: 12})
		require.NoError(t, err)
		require.Len(t, cands, 5)
		require.True(t, gen.Truncated)
		// Original, 23 more axis-aligned rotations, 6 face-down ones.
		require.Equal(t, 30, gen.Distinct)
	})

	t.Run("triangle order does not change candidates", func(t *testing.T) {
		soup := fixture.ObliqueCuboid()
		a, _, err := orient.Generate(t.Context(), build(t, soup), limits)
		require.NoError(t, err)
		rng := rand.New(rand.NewPCG(1, 2))
		rng.Shuffle(len(soup), func(i, j int) { soup[i], soup[j] = soup[j], soup[i] })
		b, _, err := orient.Generate(t.Context(), build(t, soup), limits)
		require.NoError(t, err)
		require.Len(t, b, len(a))
		for i := range a {
			require.True(t, a[i].Rotation.Equal(b[i].Rotation, 1e-12))
		}
	})

	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, err := orient.Generate(ctx, build(t, fixture.Cube()), limits)
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestPlace(t *testing.T) {
	m := build(t, fixture.ObliqueCuboid())
	cands, _, err := orient.Generate(t.Context(), m, limits)
	require.NoError(t, err)
	for _, c := range cands {
		placed, full, err := orient.Place(m, c.Rotation)
		require.NoError(t, err)
		b := placed.Bounds()
		require.Zero(t, b.Min.Z)
		require.InDelta(t, 0, b.Min.X+b.Max.X, 1e-9)
		require.InDelta(t, 0, b.Min.Y+b.Max.Y, 1e-9)
		require.False(t, full.IsReflection())
		require.InDelta(t, m.Volume(), placed.Volume(), 1e-6)

		inv, err := full.Inverse()
		require.NoError(t, err)
		for i, v := range placed.Vertices {
			require.True(t, inv.Apply(v).Equal(m.Vertices[i], 1e-9))
		}
	}
}

func TestMeasure(t *testing.T) {
	tol := mesh.Tolerance{Length: 1e-9, Serialization: 1e-6, Plate: 4e-6}

	t.Run("cube on the plate", func(t *testing.T) {
		placed, _, err := orient.Place(build(t, fixture.Cube()), r3.Identity())
		require.NoError(t, err)
		met := orient.Measure(placed, 45, tol)
		require.InDelta(t, 20, met.HeightMM, 1e-12)
		require.InDelta(t, 400, met.BedContactAreaMM2, 1e-9)
		require.Zero(t, met.SupportDemandAreaMM2)
		require.NotNil(t, met.CentroidOverContact)
		require.True(t, *met.CentroidOverContact)
		require.InDelta(t, 10, *met.ContactHullMarginMM, 1e-9)
	})

	t.Run("bracket arm underside is demand; contact is the post only", func(t *testing.T) {
		placed, _, err := orient.Place(build(t, fixture.Bracket()), r3.Identity())
		require.NoError(t, err)
		met := orient.Measure(placed, 45, tol)
		require.InDelta(t, 600, met.SupportDemandAreaMM2, 1e-9)
		require.InDelta(t, 600, met.SupportDemandProjectedAreaMM2, 1e-9)
		// The bounding box covers 40 x 20 mm; the post touches 10 x 20.
		require.InDelta(t, 200, met.BedContactAreaMM2, 1e-9)
		require.InDelta(t, 800, met.FootprintMM[0]*met.FootprintMM[1], 1e-9)
		require.NotNil(t, met.CentroidOverContact)
		require.False(t, *met.CentroidOverContact)
	})

	t.Run("oblique cuboid touches the plate at a vertex", func(t *testing.T) {
		placed, _, err := orient.Place(build(t, fixture.ObliqueCuboid()), r3.Identity())
		require.NoError(t, err)
		met := orient.Measure(placed, 45, tol)
		require.Zero(t, met.BedContactAreaMM2)
		require.Nil(t, met.CentroidOverContact, "no contact polygon")
		require.Greater(t, met.SupportDemandAreaMM2, 0.0)
	})
}

func TestRank(t *testing.T) {
	order := orient.Rank([]orient.Entry{
		{ID: 0, Feasible: true, Score: 0.5},
		{ID: 1, Feasible: false, Score: -10},
		{ID: 2, Feasible: true, Score: 0.1},
		{ID: 3, Feasible: true, Score: 0.1 + 1e-12}, // ties with 2 after rounding
		{ID: 4, Feasible: true, Score: 0.1 - 1e-6},
	})
	require.Equal(t, []int{4, 2, 3, 0, 1}, order)
}

func TestScore(t *testing.T) {
	met := orient.Metrics{HeightMM: 10, SupportDemandProjectedAreaMM2: 50, BedContactAreaMM2: 25}
	terms, score := orient.Score(met, orient.Weights{SupportDemand: 2, Height: 1, BedContact: 4}, orient.Scale{SurfaceAreaMM2: 100, SizeMM: 20})
	require.InDelta(t, 1.0, terms.SupportDemand, 1e-12)
	require.InDelta(t, 0.5, terms.Height, 1e-12)
	require.InDelta(t, -1.0, terms.BedContact, 1e-12)
	require.InDelta(t, 0.5, score, 1e-12)
	require.False(t, math.IsNaN(score))
}
