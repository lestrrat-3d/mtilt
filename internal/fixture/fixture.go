// Package fixture builds the synthetic meshes mtilt's tests and the files in
// testdata/ are made from. Every shape is a closed, outward-wound mesh with
// known dimensions, built in millimeters.
package fixture

import (
	"fmt"
	"math"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Box returns the 12 triangles of the axis-aligned box from lo to hi.
func Box(lo, hi r3.Vec) [][3]r3.Vec {
	return Prism([][2]float64{{lo.X, lo.Z}, {hi.X, lo.Z}, {hi.X, hi.Z}, {lo.X, hi.Z}}, lo.Y, hi.Y)
}

// Cube returns a 20 mm cube with one corner at the origin.
func Cube() [][3]r3.Vec {
	return Box(r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 20))
}

// CubeSplitFaces returns the same 20 mm cube as Cube, with each square face
// split into four triangles around its center instead of two. It encloses
// the same solid with a different triangulation.
func CubeSplitFaces() [][3]r3.Vec {
	const s = 20.0
	c := func(x, y, z float64) r3.Vec { return r3.NewVec(x, y, z) }
	// Each face lists its corners counter-clockwise seen from outside.
	faces := [][4]r3.Vec{
		{c(0, 0, 0), c(0, s, 0), c(s, s, 0), c(s, 0, 0)}, // bottom, -Z
		{c(0, 0, s), c(s, 0, s), c(s, s, s), c(0, s, s)}, // top, +Z
		{c(0, 0, 0), c(s, 0, 0), c(s, 0, s), c(0, 0, s)}, // -Y
		{c(0, s, 0), c(0, s, s), c(s, s, s), c(s, s, 0)}, // +Y
		{c(0, 0, 0), c(0, 0, s), c(0, s, s), c(0, s, 0)}, // -X
		{c(s, 0, 0), c(s, s, 0), c(s, s, s), c(s, 0, s)}, // +X
	}
	var out [][3]r3.Vec
	for _, f := range faces {
		center := f[0].Add(f[1]).Add(f[2]).Add(f[3]).Scale(0.25)
		for k := range 4 {
			out = append(out, [3]r3.Vec{f[k], f[(k+1)%4], center})
		}
	}
	return out
}

// ObliqueCuboid returns a 40 x 20 x 10 mm cuboid rotated 37 degrees about the
// axis (1, 2, 3), so that none of its faces is axis-aligned. Its lowest
// vertex is at Z = 5.
func ObliqueCuboid() [][3]r3.Vec {
	tris := Box(r3.NewVec(-20, -10, -5), r3.NewVec(20, 10, 5))
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	if err != nil {
		panic(err) // a fixed, valid axis and angle
	}
	minZ := math.Inf(1)
	for i := range tris {
		for k := range 3 {
			tris[i][k] = rot.Apply(tris[i][k])
			minZ = math.Min(minZ, tris[i][k].Z)
		}
	}
	lift := r3.NewVec(0, 0, 5-minZ)
	for i := range tris {
		for k := range 3 {
			tris[i][k] = tris[i][k].Add(lift)
		}
	}
	return tris
}

// Bracket returns a Γ-shaped bracket, 20 mm deep along Y: a 10 x 40 mm post
// at X 0..10 with a 30 mm arm at Z 30..40 reaching out to X = 40. In the
// orientation given, the arm's underside is a 30 x 20 mm horizontal
// overhang at Z = 30 with nothing below it.
func Bracket() [][3]r3.Vec {
	return Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {40, 30}, {40, 40}, {0, 40}}, 0, 20)
}

// Bridge returns a Π-shaped part, 20 mm deep along Y: posts at X 0..10 and
// X 40..50, joined by a beam at Z 30..40. The beam's underside between the
// posts is a 30 x 20 mm horizontal span at Z = 30 with nothing below it.
func Bridge() [][3]r3.Vec {
	return Prism([][2]float64{{0, 0}, {10, 0}, {10, 30}, {40, 30}, {40, 0}, {50, 0}, {50, 40}, {0, 40}}, 0, 20)
}

// Occluded returns a ⊐-shaped part, 20 mm deep along Y: a base at Z 0..10, a
// back wall at X 0..10, and a top arm at Z 40..50 that reaches over the
// base. In the orientation given, the top arm's underside at Z = 40 needs
// support, and the base lies directly below it, so no support rooted on the
// build plate can reach it.
func Occluded() [][3]r3.Vec {
	return Prism([][2]float64{{0, 0}, {40, 0}, {40, 10}, {10, 10}, {10, 40}, {40, 40}, {40, 50}, {0, 50}}, 0, 20)
}

// Prism extrudes a simple polygon in the XZ plane from Y = y0 to Y = y1. The
// polygon may be given in either winding; the result is wound outward.
func Prism(poly [][2]float64, y0, y1 float64) [][3]r3.Vec {
	if signedArea(poly) < 0 {
		rev := make([][2]float64, len(poly))
		for i, p := range poly {
			rev[len(poly)-1-i] = p
		}
		poly = rev
	}
	at := func(p [2]float64, y float64) r3.Vec { return r3.NewVec(p[0], y, p[1]) }
	var out [][3]r3.Vec
	for _, t := range earClip(poly) {
		a, b, c := poly[t[0]], poly[t[1]], poly[t[2]]
		// A counter-clockwise (X, Z) triangle at Y = y0 faces -Y.
		out = append(out, [3]r3.Vec{at(a, y0), at(b, y0), at(c, y0)})
		out = append(out, [3]r3.Vec{at(a, y1), at(c, y1), at(b, y1)})
	}
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		out = append(out,
			[3]r3.Vec{at(p, y0), at(q, y1), at(q, y0)},
			[3]r3.Vec{at(p, y0), at(p, y1), at(q, y1)},
		)
	}
	return out
}

func signedArea(poly [][2]float64) float64 {
	var a float64
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		a += p[0]*q[1] - q[0]*p[1]
	}
	return a / 2
}

// earClip triangulates a simple counter-clockwise polygon.
func earClip(poly [][2]float64) [][3]int {
	idx := make([]int, len(poly))
	for i := range idx {
		idx[i] = i
	}
	var out [][3]int
	for len(idx) > 3 {
		clipped := false
		for i := range idx {
			a, b, c := idx[(i+len(idx)-1)%len(idx)], idx[i], idx[(i+1)%len(idx)]
			if cross2(poly[a], poly[b], poly[c]) <= 0 {
				continue
			}
			if anyInside(poly, idx, a, b, c) {
				continue
			}
			out = append(out, [3]int{a, b, c})
			idx = append(idx[:i], idx[i+1:]...)
			clipped = true
			break
		}
		if !clipped {
			panic("fixture: polygon is not simple")
		}
	}
	return append(out, [3]int{idx[0], idx[1], idx[2]})
}

func cross2(a, b, c [2]float64) float64 {
	return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
}

func anyInside(poly [][2]float64, idx []int, a, b, c int) bool {
	for _, j := range idx {
		if j == a || j == b || j == c {
			continue
		}
		p := poly[j]
		if cross2(poly[a], poly[b], p) >= 0 && cross2(poly[b], poly[c], p) >= 0 && cross2(poly[c], poly[a], p) >= 0 {
			return true
		}
	}
	return false
}

// ASCII renders tris as an ASCII STL file named name. Coordinates are
// written with the shortest decimal text that parses back to the same
// float64. Facet normals are computed from the winding.
func ASCII(name string, tris [][3]r3.Vec) string {
	var sb strings.Builder
	num := func(v float64) string { return strconv.FormatFloat(v, 'g', -1, 64) }
	vec := func(v r3.Vec) string { return num(v.X) + " " + num(v.Y) + " " + num(v.Z) }
	fmt.Fprintf(&sb, "solid %s\n", name)
	for _, t := range tris {
		n, _ := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
		fmt.Fprintf(&sb, "  facet normal %s\n    outer loop\n", vec(n))
		for _, v := range t {
			fmt.Fprintf(&sb, "      vertex %s\n", vec(v))
		}
		sb.WriteString("    endloop\n  endfacet\n")
	}
	fmt.Fprintf(&sb, "endsolid %s\n", name)
	return sb.String()
}
