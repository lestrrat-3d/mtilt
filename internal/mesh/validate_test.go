package mesh_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func build(t *testing.T, soup [][3]r3.Vec) *mesh.Mesh {
	t.Helper()
	m, err := mesh.FromSoup(soup)
	require.NoError(t, err)
	return m
}

func validate(t *testing.T, m *mesh.Mesh) mesh.Report {
	t.Helper()
	r, err := mesh.Validate(t.Context(), m, mesh.ToleranceFor(m.Bounds()))
	require.NoError(t, err)
	return r
}

func statusOf(t *testing.T, r mesh.Report, name string) mesh.Status {
	t.Helper()
	s, ok := r.Status(name)
	require.True(t, ok, "report has no check %q", name)
	return s
}

func TestValidate(t *testing.T) {
	t.Run("fixture shapes pass every implemented check", func(t *testing.T) {
		shapes := map[string][][3]r3.Vec{
			"cube":             fixture.Cube(),
			"cube split faces": fixture.CubeSplitFaces(),
			"oblique cuboid":   fixture.ObliqueCuboid(),
			"bracket":          fixture.Bracket(),
			"bridge":           fixture.Bridge(),
			"occluded":         fixture.Occluded(),
		}
		for name, soup := range shapes {
			r := validate(t, build(t, soup))
			require.Empty(t, r.Failed(), name)
			require.Equal(t, 1, r.Components, name)
			require.Equal(t, mesh.StatusUnchecked, statusOf(t, r, mesh.CheckSelfIntersection), name)
		}
	})

	t.Run("open mesh fails closed and leaves orientation unchecked", func(t *testing.T) {
		r := validate(t, build(t, fixture.Cube()[1:]))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckClosed))
		require.Equal(t, mesh.StatusUnchecked, statusOf(t, r, mesh.CheckOutward))
	})

	t.Run("one reversed triangle fails consistent winding", func(t *testing.T) {
		soup := fixture.Cube()
		soup[3] = [3]r3.Vec{soup[3][0], soup[3][2], soup[3][1]}
		r := validate(t, build(t, soup))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckConsistentWind))
		require.Equal(t, mesh.StatusPassed, statusOf(t, r, mesh.CheckClosed))
	})

	t.Run("all triangles reversed fails outward orientation", func(t *testing.T) {
		soup := fixture.Cube()
		for i := range soup {
			soup[i] = [3]r3.Vec{soup[i][0], soup[i][2], soup[i][1]}
		}
		r := validate(t, build(t, soup))
		require.Equal(t, mesh.StatusPassed, statusOf(t, r, mesh.CheckConsistentWind))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckOutward))
	})

	t.Run("two disjoint cubes fail single component", func(t *testing.T) {
		soup := append(fixture.Cube(), fixture.Box(r3.NewVec(40, 0, 0), r3.NewVec(60, 20, 20))...)
		r := validate(t, build(t, soup))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckSingleComponent))
		require.Equal(t, 2, r.Components)
	})

	t.Run("cubes sharing one corner fail vertex manifold", func(t *testing.T) {
		soup := append(fixture.Cube(), fixture.Box(r3.NewVec(20, 20, 20), r3.NewVec(40, 40, 40))...)
		r := validate(t, build(t, soup))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckVertexManifold))
		require.Equal(t, mesh.StatusPassed, statusOf(t, r, mesh.CheckEdgeManifold))
	})

	t.Run("a third triangle on an edge fails edge manifold", func(t *testing.T) {
		soup := fixture.Cube()
		fin := [3]r3.Vec{soup[0][0], soup[0][1], r3.NewVec(-5, -5, -5)}
		r := validate(t, build(t, append(soup, fin)))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckEdgeManifold))
	})

	t.Run("collinear sliver fails nondegenerate", func(t *testing.T) {
		soup := fixture.Cube()
		soup = append(soup, [3]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0)})
		r := validate(t, build(t, soup))
		require.Equal(t, mesh.StatusFailed, statusOf(t, r, mesh.CheckNondegenerate))
	})

	t.Run("cancelled context", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		m := build(t, fixture.Cube())
		_, err := mesh.Validate(ctx, m, mesh.ToleranceFor(m.Bounds()))
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestFromSoup(t *testing.T) {
	t.Run("merges exactly equal positions only", func(t *testing.T) {
		m := build(t, fixture.Cube())
		require.Len(t, m.Vertices, 8)
		require.Len(t, m.Triangles, 12)

		soup := fixture.Cube()
		soup[0][0] = soup[0][0].Add(r3.NewVec(1e-12, 0, 0))
		require.Len(t, build(t, soup).Vertices, 9)
	})

	t.Run("rejects non-finite coordinates", func(t *testing.T) {
		soup := fixture.Cube()
		soup[2][1] = r3.NewVec(math.NaN(), 0, 0)
		_, err := mesh.FromSoup(soup)
		require.ErrorIs(t, err, mesh.ErrNonFinite)
	})
}

func TestMeasurements(t *testing.T) {
	m := build(t, fixture.Cube())
	require.InDelta(t, 8000, m.Volume(), 1e-9)
	require.InDelta(t, 2400, m.SurfaceArea(), 1e-9)
	require.True(t, m.Centroid().Equal(r3.NewVec(10, 10, 10), 1e-9))

	bracket := build(t, fixture.Bracket())
	// Post 10 x 40 plus arm 30 x 10, times 20 deep.
	require.InDelta(t, (10*40+30*10)*20, bracket.Volume(), 1e-9)
}

func TestInertia(t *testing.T) {
	// A 40 x 20 x 10 box: I_xx = V (b^2 + c^2) / 12 etc., products zero.
	m := build(t, fixture.Box(r3.NewVec(5, -3, 2), r3.NewVec(45, 17, 12)))
	v := 40.0 * 20 * 10
	in := m.Inertia()
	require.InDelta(t, v*(20*20+10*10)/12, in.XX, 1e-6)
	require.InDelta(t, v*(40*40+10*10)/12, in.YY, 1e-6)
	require.InDelta(t, v*(40*40+20*20)/12, in.ZZ, 1e-6)
	require.InDelta(t, 0, in.XY, 1e-6)
	require.InDelta(t, 0, in.XZ, 1e-6)
	require.InDelta(t, 0, in.YZ, 1e-6)
}

func TestTransformed(t *testing.T) {
	m := build(t, fixture.ObliqueCuboid())
	rot, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(0, 1, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(1, 0, 0)}, r3.NewVec(3, -4, 5))
	require.NoError(t, err)

	moved := m.Transformed(rot)
	require.InDelta(t, m.Volume(), moved.Volume(), 1e-9)
	require.InDelta(t, m.SurfaceArea(), moved.SurfaceArea(), 1e-9)
	require.Empty(t, validate(t, moved).Failed())

	inv, err := rot.Inverse()
	require.NoError(t, err)
	back := moved.Transformed(inv)
	for i := range m.Vertices {
		require.True(t, back.Vertices[i].Equal(m.Vertices[i], 1e-12))
	}

	a, b := m.Inertia(), moved.Inertia()
	// The trace of an inertia tensor does not change under rotation.
	require.InDelta(t, a.XX+a.YY+a.ZZ, b.XX+b.YY+b.ZZ, 1e-6*(a.XX+a.YY+a.ZZ))
}
