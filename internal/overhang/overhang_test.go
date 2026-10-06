package overhang_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// downFacing returns a triangle at height z whose plane makes angleDeg with
// the horizontal and whose normal points down.
func downFacing(angleDeg, z float64) [3]r3.Vec {
	a := angleDeg * math.Pi / 180
	// The plane contains the Y axis and the direction (cos a, 0, sin a).
	// Winding (p0, p2, p1) puts the normal below the plane.
	p0 := r3.NewVec(0, 0, z)
	p1 := r3.NewVec(math.Cos(a), 0, z+math.Sin(a))
	p2 := r3.NewVec(0, 1, z)
	return [3]r3.Vec{p0, p2, p1}
}

func normal(t [3]r3.Vec) r3.Vec {
	n, _ := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
	return n
}

func TestAngle(t *testing.T) {
	require.InDelta(t, 0, overhang.Angle(r3.NewVec(0, 0, -1)), 1e-12, "downward ceiling")
	require.InDelta(t, 90, overhang.Angle(r3.NewVec(1, 0, 0)), 1e-12, "vertical wall")
	require.InDelta(t, 90, overhang.Angle(r3.NewVec(0, 0, 1)), 1e-12, "upward face")
	for _, deg := range []float64{10, 30, 44.9, 45.1, 60, 80} {
		tri := downFacing(deg, 5)
		require.Less(t, normal(tri).Z, 0.0)
		require.InDelta(t, deg, overhang.Angle(normal(tri)), 1e-9)
	}
}

func TestClassify(t *testing.T) {
	const threshold, bedTol = 45.0, 1e-6
	require.Equal(t, overhang.Demand, overhang.Classify(downFacing(0, 5), threshold, bedTol), "ceiling")
	require.Equal(t, overhang.Demand, overhang.Classify(downFacing(44.9, 5), threshold, bedTol))
	require.Equal(t, overhang.None, overhang.Classify(downFacing(45.1, 5), threshold, bedTol))
	require.Equal(t, overhang.BedContact, overhang.Classify(downFacing(0, 0), threshold, bedTol), "face on the plate")
	require.Equal(t, overhang.Demand, overhang.Classify(downFacing(0, 2*bedTol), threshold, bedTol), "just above the plate")

	up := downFacing(0, 5)
	up[1], up[2] = up[2], up[1]
	require.Equal(t, overhang.None, overhang.Classify(up, threshold, bedTol), "upward face")

	sliver := [3]r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(2, 0, 1)}
	require.Equal(t, overhang.None, overhang.Classify(sliver, threshold, bedTol), "zero area")
}
