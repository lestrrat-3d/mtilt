package orient

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
)

// Principal holds a body's principal axes of inertia: the directions in which
// its mass is most and least spread out. Axes[0] is the long axis; for a
// plate, Axes[2] is the plate's normal.
type Principal struct {
	// Axes are unit vectors sorted by ascending moment. Axes[0] has the
	// smallest moment of inertia: it is the body's long axis. The three form
	// a right-handed frame.
	Axes [3]r3.Vec
	// Moments are the principal moments, ascending, in the unit the tensor
	// was given in.
	Moments [3]float64
	// Elongation is 1 - Moments[0]/Moments[1]: 0 when no single axis is
	// longer than the others (a cube, a sphere, a flat square plate), and
	// close to 1 for a thin stick. It is 0 when Moments[1] is 0.
	Elongation float64
	// Spread holds the second moments of the mass along each axis,
	// descending: Spread[i] = max(0, (Mj + Mk - Mi)/2) for Moments M, with
	// j and k the other two axes. For a box it is the mass times a^2/12 for
	// the box's extent a along Axes[i].
	Spread [3]float64
	// Flatness is sqrt(Spread[2]/Spread[1]): the body's thickness along
	// Axes[2] over its width along Axes[1]. It is 0.125 for a
	// 100 x 40 x 5 mm plate and 1 for a cube. It is 1 when Spread[1] is 0.
	Flatness float64
}

// Jacobi iteration limits. A 3x3 symmetric matrix converges to rounding in
// well under ten sweeps; the iteration stops early once the off-diagonal sum
// falls below jacobiRelTol times the diagonal sum.
const (
	jacobiSweeps = 50
	jacobiRelTol = 1e-15
)

// NewPrincipal diagonalizes the inertia tensor in by cyclic Jacobi
// rotations. Each axis's sign is fixed so its largest-magnitude component is
// positive, and the third axis is then replaced by the cross product of the
// first two, so the result does not depend on which signs the iteration
// produced.
func NewPrincipal(in mesh.InertiaTensor) Principal {
	a := [3][3]float64{{in.XX, in.XY, in.XZ}, {in.XY, in.YY, in.YZ}, {in.XZ, in.YZ, in.ZZ}}
	v := [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	for range jacobiSweeps {
		off := math.Abs(a[0][1]) + math.Abs(a[0][2]) + math.Abs(a[1][2])
		scale := math.Abs(a[0][0]) + math.Abs(a[1][1]) + math.Abs(a[2][2])
		if off <= jacobiRelTol*scale || off == 0 {
			break
		}
		for _, pq := range [][2]int{{0, 1}, {0, 2}, {1, 2}} {
			p, q := pq[0], pq[1]
			if a[p][q] == 0 {
				continue
			}
			theta := (a[q][q] - a[p][p]) / (2 * a[p][q])
			t := math.Copysign(1, theta) / (math.Abs(theta) + math.Sqrt(theta*theta+1))
			c := 1 / math.Sqrt(t*t+1)
			s := t * c
			for k := range 3 {
				akp, akq := a[k][p], a[k][q]
				a[k][p], a[k][q] = c*akp-s*akq, s*akp+c*akq
			}
			for k := range 3 {
				apk, aqk := a[p][k], a[q][k]
				a[p][k], a[q][k] = c*apk-s*aqk, s*apk+c*aqk
			}
			for k := range 3 {
				vkp, vkq := v[k][p], v[k][q]
				v[k][p], v[k][q] = c*vkp-s*vkq, s*vkp+c*vkq
			}
		}
	}

	type pair struct {
		moment float64
		axis   r3.Vec
	}
	pairs := make([]pair, 3)
	for i := range 3 {
		pairs[i] = pair{moment: a[i][i], axis: canonicalSign(r3.NewVec(v[0][i], v[1][i], v[2][i]))}
	}
	slices.SortStableFunc(pairs, func(x, y pair) int { return cmp.Compare(x.moment, y.moment) })

	var pr Principal
	for i, p := range pairs {
		pr.Moments[i] = p.moment
		pr.Axes[i] = p.axis
	}
	pr.Axes[2] = pr.Axes[0].Cross(pr.Axes[1])
	if pr.Moments[1] > 0 {
		pr.Elongation = math.Max(0, 1-pr.Moments[0]/pr.Moments[1])
	}
	m := pr.Moments
	for i := range 3 {
		pr.Spread[i] = math.Max(0, (m[(i+1)%3]+m[(i+2)%3]-m[i])/2)
	}
	pr.Flatness = 1
	if pr.Spread[1] > 0 {
		pr.Flatness = math.Sqrt(pr.Spread[2] / pr.Spread[1])
	}
	return pr
}

// canonicalSign flips v so that its largest-magnitude component (the first
// one, on ties) is positive.
func canonicalSign(v r3.Vec) r3.Vec {
	c := []float64{v.X, v.Y, v.Z}
	best := 0
	for i := 1; i < 3; i++ {
		if math.Abs(c[i]) > math.Abs(c[best]) {
			best = i
		}
	}
	if c[best] < 0 {
		return v.Scale(-1)
	}
	return v
}

// ElevationDeg returns the angle in degrees between direction d and the
// build plate: 0 when d is horizontal, 90 when it is vertical.
func ElevationDeg(d r3.Vec) float64 {
	l := d.Len()
	if l == 0 {
		return 0
	}
	return math.Asin(math.Min(1, math.Abs(d.Z)/l)) * 180 / math.Pi
}
