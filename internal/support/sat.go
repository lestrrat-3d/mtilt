package support

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// convex is a convex solid described for separating-axis tests: its
// vertices, the normals of its faces, and the directions of its edges.
// Extra normals or edge directions are harmless: any axis on which the
// projections are disjoint proves the shapes are apart, so a superset of the
// required axes only adds work.
type convex struct {
	verts   []r3.Vec
	normals []r3.Vec
	edges   []r3.Vec
}

func (c convex) bounds() (r3.Vec, r3.Vec) {
	lo, hi := c.verts[0], c.verts[0]
	for _, v := range c.verts[1:] {
		lo = r3.NewVec(math.Min(lo.X, v.X), math.Min(lo.Y, v.Y), math.Min(lo.Z, v.Z))
		hi = r3.NewVec(math.Max(hi.X, v.X), math.Max(hi.Y, v.Y), math.Max(hi.Z, v.Z))
	}
	return lo, hi
}

var (
	axisX = r3.NewVec(1, 0, 0)
	axisY = r3.NewVec(0, 1, 0)
	axisZ = r3.NewVec(0, 0, 1)
)

// frustum returns the convex solid between an axis-aligned square of
// half-size h0 at height z0 and one of half-size h1 at height z1 > z0, both
// centered on (x, y). Equal half-sizes give a box.
func frustum(x, y, z0, h0, z1, h1 float64) convex {
	sq := func(z, h float64) []r3.Vec {
		return []r3.Vec{
			r3.NewVec(x-h, y-h, z), r3.NewVec(x+h, y-h, z),
			r3.NewVec(x+h, y+h, z), r3.NewVec(x-h, y+h, z),
		}
	}
	lo, hi := sq(z0, h0), sq(z1, h1)
	c := convex{
		verts:   append(lo, hi...),
		normals: []r3.Vec{axisZ},
		edges:   []r3.Vec{axisX, axisY},
	}
	for k := range 4 {
		side := hi[k].Sub(lo[k])
		c.edges = append(c.edges, side)
		c.normals = append(c.normals, lo[(k+1)%4].Sub(lo[k]).Cross(side))
	}
	return c
}

// sweptUp returns the convex hull of c and c moved up by h: every point
// within vertical distance h above c.
func sweptUp(c convex, h float64) convex {
	out := convex{
		verts:   make([]r3.Vec, 0, 2*len(c.verts)),
		normals: append([]r3.Vec(nil), c.normals...),
		edges:   append(append([]r3.Vec(nil), c.edges...), axisZ),
	}
	lift := r3.NewVec(0, 0, h)
	for _, v := range c.verts {
		out.verts = append(out.verts, v, v.Add(lift))
	}
	for _, e := range c.edges {
		out.normals = append(out.normals, e.Cross(axisZ))
	}
	return out
}

// triangleConvex describes a triangle as a convex solid, so two triangles
// can be tested against each other.
func triangleConvex(t [3]r3.Vec) convex {
	e := []r3.Vec{t[1].Sub(t[0]), t[2].Sub(t[1]), t[0].Sub(t[2])}
	return convex{verts: t[:], normals: []r3.Vec{e[0].Cross(e[1])}, edges: e}
}

// touches reports whether convex solid c and triangle t share a point. A gap
// of at most eps along every tested axis counts as touching, so a triangle
// lying exactly on c's surface touches it.
func touches(c convex, t [3]r3.Vec, eps float64) bool {
	tc := triangleConvex(t)
	try := func(axis r3.Vec) bool {
		l := axis.Len()
		if l < 1e-12 {
			return false
		}
		a := axis.Scale(1 / l)
		cmin, cmax := project(c.verts, a)
		tmin, tmax := project(tc.verts, a)
		return cmax < tmin-eps || tmax < cmin-eps
	}
	if slices.ContainsFunc(c.normals, try) {
		return false
	}
	if try(tc.normals[0]) {
		return false
	}
	for _, te := range tc.edges {
		for _, ce := range c.edges {
			if try(te.Cross(ce)) {
				return false
			}
		}
	}
	return true
}

func project(verts []r3.Vec, a r3.Vec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, v := range verts {
		d := v.Dot(a)
		lo = math.Min(lo, d)
		hi = math.Max(hi, d)
	}
	return lo, hi
}

// anyTouching reports whether any model triangle touches c, and returns the
// first such triangle's index.
func (ix *index) anyTouching(c convex, eps float64) (int, bool) {
	lo, hi := c.bounds()
	found := -1
	ix.query(lo.X-eps, lo.Y-eps, hi.X+eps, hi.Y+eps, func(i int) bool {
		b := ix.boxes[i]
		if b.Max.Z < lo.Z-eps || b.Min.Z > hi.Z+eps {
			return true
		}
		if touches(c, ix.tris[i], eps) {
			found = i
			return false
		}
		return true
	})
	return found, found >= 0
}
