package orient_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func build(t *testing.T, soup [][3]r3.Vec) *mesh.Mesh {
	t.Helper()
	m, err := mesh.FromSoup(soup)
	require.NoError(t, err)
	return m
}

func TestMeasure(t *testing.T) {
	tol := mesh.Tolerance{Length: 1e-9, Serialization: 1e-6, Plate: 4e-6}

	t.Run("cube on the plate", func(t *testing.T) {
		placed, _, err := orient.Place(build(t, fixture.Cube()), r3.Identity())
		require.NoError(t, err)
		met := orient.Measure(placed, 45, 0, 0.2, tol)
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
		met := orient.Measure(placed, 45, 0, 0.2, tol)
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
		met := orient.Measure(placed, 45, 0, 0.2, tol)
		require.Zero(t, met.BedContactAreaMM2)
		require.Nil(t, met.CentroidOverContact, "no contact polygon")
		require.Greater(t, met.SupportDemandAreaMM2, 0.0)
	})
}

func TestPrincipal(t *testing.T) {
	t.Run("box axes come out sorted, long axis first", func(t *testing.T) {
		// 8 x 100 x 20 box: long axis Y, then Z, then X.
		m := build(t, fixture.Box(r3.NewVec(0, 0, 0), r3.NewVec(8, 100, 20)))
		pr := orient.NewPrincipal(m.Inertia())
		require.True(t, pr.Axes[0].Equal(r3.NewVec(0, 1, 0), 1e-9), "%v", pr.Axes[0])
		require.True(t, pr.Axes[1].Equal(r3.NewVec(0, 0, 1), 1e-9), "%v", pr.Axes[1])
		require.InDelta(t, 1, pr.Axes[0].Cross(pr.Axes[1]).Dot(pr.Axes[2]), 1e-12, "right-handed")
		require.Less(t, pr.Moments[0], pr.Moments[1])
		require.Less(t, pr.Moments[1], pr.Moments[2])
		// 1 - (8^2+20^2)/(8^2+100^2)
		require.InDelta(t, 1-(64.0+400)/(64+10000), pr.Elongation, 1e-9)
	})

	t.Run("cube has no long axis", func(t *testing.T) {
		pr := orient.NewPrincipal(build(t, fixture.Cube()).Inertia())
		require.InDelta(t, 0, pr.Elongation, 1e-9)
	})

	t.Run("rotated tensor gives the rotated axis", func(t *testing.T) {
		m := build(t, fixture.Box(r3.NewVec(-4, -50, -4), r3.NewVec(4, 50, 4)))
		rot, err := r3.FromBasis(r3.Basis{
			EX: r3.NewVec(0.6, 0.8, 0), EY: r3.NewVec(-0.8, 0.6, 0), EZ: r3.NewVec(0, 0, 1),
		}, r3.Vec{})
		require.NoError(t, err)
		pr := orient.NewPrincipal(m.Transformed(rot).Inertia())
		want := rot.ApplyDir(r3.NewVec(0, 1, 0))
		require.InDelta(t, 1, math.Abs(pr.Axes[0].Dot(want)), 1e-9)
	})
}

func FuzzPrincipal(f *testing.F) {
	f.Add(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
	f.Add(5.0, 5.0, 5.0, 0.0, 0.0, 0.0)
	f.Fuzz(func(t *testing.T, xx, yy, zz, xy, xz, yz float64) {
		for _, v := range []float64{xx, yy, zz, xy, xz, yz} {
			if math.IsNaN(v) || math.IsInf(v, 0) || math.Abs(v) > 1e6 {
				return
			}
		}
		pr := orient.NewPrincipal(mesh.InertiaTensor{XX: xx, YY: yy, ZZ: zz, XY: xy, XZ: xz, YZ: yz})
		scale := math.Abs(xx) + math.Abs(yy) + math.Abs(zz) + math.Abs(xy) + math.Abs(xz) + math.Abs(yz) + 1
		for i, a := range pr.Axes {
			require.InDelta(t, 1, a.Len(), 1e-9)
			// A v = lambda v for each axis.
			av := r3.NewVec(xx*a.X+xy*a.Y+xz*a.Z, xy*a.X+yy*a.Y+yz*a.Z, xz*a.X+yz*a.Y+zz*a.Z)
			require.True(t, av.Equal(a.Scale(pr.Moments[i]), 1e-9*scale), "axis %d", i)
		}
		require.InDelta(t, 1, pr.Axes[0].Cross(pr.Axes[1]).Dot(pr.Axes[2]), 1e-9)
		require.LessOrEqual(t, pr.Moments[0], pr.Moments[1])
		require.LessOrEqual(t, pr.Moments[1], pr.Moments[2])
	})
}

var searchOpts = orient.SearchOptions{Directions: 500, MaxPlanarFaces: 12, Finalists: 6, MaxTiltDeg: 15, MinFirstLayerMM2: 20}

func estimator(m *mesh.Mesh) *orient.Estimator {
	tol := mesh.Tolerance{Length: 1e-9, Serialization: 1e-6, Plate: 4e-6}
	return orient.NewEstimator(m, 45, 0, 0.2, 2.8, tol, orient.CostWeights{Volume: 1, Contact: 1}, orient.Scale{SurfaceAreaMM2: 1, SizeMM: 1})
}

func TestEstimator(t *testing.T) {
	down := r3.NewVec(0, 0, -1)

	t.Run("cube on its face needs nothing", func(t *testing.T) {
		e := estimator(build(t, fixture.Cube()))
		c := e.Estimate(down)
		require.Zero(t, c.VolumeMM3)
		require.Zero(t, c.ContactMM2)
		require.InDelta(t, 400, e.FirstLayer(down), 1e-9)
	})

	t.Run("bracket arm: the column under 600 mm2 at 30 mm", func(t *testing.T) {
		e := estimator(build(t, fixture.Bracket()))
		c := e.Estimate(down)
		require.InDelta(t, 600, c.ContactMM2, 1e-9)
		require.InDelta(t, 600*30, c.VolumeMM3, 1e-6)
		require.Zero(t, c.TooLowMM2)
		require.InDelta(t, 600*30+600, c.Score, 1e-6)
		require.InDelta(t, 200, e.FirstLayer(down), 1e-9)
	})

	t.Run("an overhang under the shortest pillar is too low", func(t *testing.T) {
		ledge := fixture.Prism([][2]float64{{0, 0}, {10, 0}, {10, 2}, {30, 2}, {30, 5}, {0, 5}}, 0, 20)
		c := estimator(build(t, ledge)).Estimate(down)
		require.InDelta(t, 400, c.TooLowMM2, 1e-9)
	})

	t.Run("the estimate does not depend on the frame", func(t *testing.T) {
		m := build(t, fixture.Bracket())
		rot, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(0, 1, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(1, 0, 0)}, r3.NewVec(5, 6, 7))
		require.NoError(t, err)
		a := estimator(m).Estimate(down)
		b := estimator(m.Transformed(rot)).Estimate(rot.ApplyDir(down))
		require.InDelta(t, a.VolumeMM3, b.VolumeMM3, 1e-6)
		require.InDelta(t, a.ContactMM2, b.ContactMM2, 1e-6)
	})
}

func TestSearch(t *testing.T) {
	t.Run("oblique cuboid: the best finalist puts a large face down", func(t *testing.T) {
		m := build(t, fixture.ObliqueCuboid())
		pr := orient.NewPrincipal(m.Inertia())
		cands, stats, err := orient.Search(t.Context(), m, pr, estimator(m), searchOpts)
		require.NoError(t, err)
		require.True(t, stats.Constrained)
		require.Equal(t, 500, stats.Swept)
		require.Equal(t, orient.SourceOriginal, cands[0].Source)
		require.False(t, cands[0].Allowed, "the given pose tilts the long axis 16 degrees")
		best := cands[1]
		require.Zero(t, best.Estimate.Score)
		require.True(t, best.Rotation.ApplyDir(best.Down).Equal(r3.NewVec(0, 0, -1), 1e-9))
		placed, _, err := orient.Place(m, best.Rotation)
		require.NoError(t, err)
		require.Greater(t, placed.SliceArea(0.2), 200.0)
		for _, c := range cands[1:] {
			require.True(t, c.Allowed)
			require.LessOrEqual(t, c.ElevationDeg, 15+1e-9)
		}
	})

	t.Run("cube: no long axis, no tilt limit", func(t *testing.T) {
		m := build(t, fixture.Cube())
		_, stats, err := orient.Search(t.Context(), m, orient.NewPrincipal(m.Inertia()), estimator(m), searchOpts)
		require.NoError(t, err)
		require.False(t, stats.Constrained)
	})

	t.Run("finalists are sorted, distinct, and refined ones are no worse", func(t *testing.T) {
		m := build(t, fixture.Bracket())
		cands, _, err := orient.Search(t.Context(), m, orient.NewPrincipal(m.Inertia()), estimator(m), searchOpts)
		require.NoError(t, err)
		fin := cands[1:]
		require.LessOrEqual(t, len(fin), searchOpts.Finalists)
		for i := range fin {
			if i > 0 {
				require.GreaterOrEqual(t, fin[i].Estimate.Score, fin[i-1].Estimate.Score-orient.ScoreQuantum)
			}
			for _, o := range fin[:i] {
				require.Less(t, fin[i].Down.Dot(o.Down), math.Cos(math.Pi/180))
			}
		}
	})

	t.Run("triangle order does not change the result", func(t *testing.T) {
		soup := fixture.ObliqueCuboid()
		m := build(t, soup)
		a, _, err := orient.Search(t.Context(), m, orient.NewPrincipal(m.Inertia()), estimator(m), searchOpts)
		require.NoError(t, err)
		rng := rand.New(rand.NewPCG(1, 2))
		rng.Shuffle(len(soup), func(i, j int) { soup[i], soup[j] = soup[j], soup[i] })
		m2 := build(t, soup)
		b, _, err := orient.Search(t.Context(), m2, orient.NewPrincipal(m2.Inertia()), estimator(m2), searchOpts)
		require.NoError(t, err)
		require.Len(t, b, len(a))
		for i := range a {
			require.True(t, a[i].Down.Equal(b[i].Down, 1e-9), "candidate %d", i)
		}
	})

	t.Run("cancelled", func(t *testing.T) {
		m := build(t, fixture.Cube())
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, err := orient.Search(ctx, m, orient.NewPrincipal(m.Inertia()), estimator(m), searchOpts)
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestPlace(t *testing.T) {
	m := build(t, fixture.ObliqueCuboid())
	cands, _, err := orient.Search(t.Context(), m, orient.NewPrincipal(m.Inertia()), estimator(m), searchOpts)
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

func TestRank(t *testing.T) {
	order := orient.Rank([]orient.Entry{
		{ID: 0, Feasible: true, Score: 0.5},
		{ID: 1, Feasible: false, Score: -10},
		{ID: 2, Feasible: true, Score: 0.1, FirstLayerAreaMM2: 100},
		{ID: 3, Feasible: true, Score: 0.1 + 1e-12, FirstLayerAreaMM2: 400}, // ties 2 on score, wins on first layer
		{ID: 4, Feasible: true, Score: 0.1 - 1e-6},
		{ID: 5, Feasible: true, Score: 0.1, FirstLayerAreaMM2: 400, HeightMM: 5}, // loses to 3 on height
		{ID: 6, Feasible: true, Score: 0.1, FirstLayerAreaMM2: 400, ElevationDeg: 3},
	})
	require.Equal(t, []int{4, 3, 6, 5, 2, 0, 1}, order)
}
