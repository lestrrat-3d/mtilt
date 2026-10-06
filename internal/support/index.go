package support

import (
	"math"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
)

// index is a uniform XY grid over a mesh's triangles. Each cell lists the
// triangles whose XY bounding box overlaps it. It is built once per placed
// model and is not safe for concurrent use (query deduplication keeps a
// per-index stamp).
type index struct {
	tris  [][3]r3.Vec
	boxes []mesh.Box

	minX, minY float64
	cell       float64
	nx, ny     int
	cells      [][]int32

	stamp []uint32
	gen   uint32
}

const maxIndexCellsPerAxis = 1024

func newIndex(m *mesh.Mesh) *index {
	ix := &index{
		tris:  m.Soup(),
		boxes: make([]mesh.Box, len(m.Triangles)),
		stamp: make([]uint32, len(m.Triangles)),
	}
	b := m.Bounds()
	ix.minX, ix.minY = b.Min.X, b.Min.Y
	w, h := b.Max.X-b.Min.X, b.Max.Y-b.Min.Y
	n := max(1, len(m.Triangles))
	ix.cell = math.Max(math.Sqrt(math.Max(w*h, 1e-12)/float64(n))*2, 1e-6)
	ix.nx = min(maxIndexCellsPerAxis, max(1, int(math.Ceil(w/ix.cell))))
	ix.ny = min(maxIndexCellsPerAxis, max(1, int(math.Ceil(h/ix.cell))))
	ix.cell = math.Max(math.Max(w/float64(ix.nx), h/float64(ix.ny)), 1e-6)
	ix.cells = make([][]int32, ix.nx*ix.ny)
	for i, t := range ix.tris {
		tb := triBox(t)
		ix.boxes[i] = tb
		x0, y0, x1, y1 := ix.cellRange(tb.Min.X, tb.Min.Y, tb.Max.X, tb.Max.Y)
		for cy := y0; cy <= y1; cy++ {
			for cx := x0; cx <= x1; cx++ {
				c := cy*ix.nx + cx
				ix.cells[c] = append(ix.cells[c], int32(i))
			}
		}
	}
	return ix
}

func triBox(t [3]r3.Vec) mesh.Box {
	b := mesh.Box{Min: t[0], Max: t[0]}
	for _, v := range t[1:] {
		b = b.Union(mesh.Box{Min: v, Max: v})
	}
	return b
}

func (ix *index) cellRange(x0, y0, x1, y1 float64) (int, int, int, int) {
	cx := func(x float64) int {
		return min(ix.nx-1, max(0, int(math.Floor((x-ix.minX)/ix.cell))))
	}
	cy := func(y float64) int {
		return min(ix.ny-1, max(0, int(math.Floor((y-ix.minY)/ix.cell))))
	}
	return cx(x0), cy(y0), cx(x1), cy(y1)
}

// query calls fn for every triangle whose XY bounding box overlaps the
// rectangle [x0, x1] x [y0, y1], once each. It stops early when fn returns
// false.
func (ix *index) query(x0, y0, x1, y1 float64, fn func(i int) bool) {
	ix.gen++
	if ix.gen == 0 {
		clear(ix.stamp)
		ix.gen = 1
	}
	cx0, cy0, cx1, cy1 := ix.cellRange(x0, y0, x1, y1)
	for cy := cy0; cy <= cy1; cy++ {
		for cx := cx0; cx <= cx1; cx++ {
			for _, ti := range ix.cells[cy*ix.nx+cx] {
				if ix.stamp[ti] == ix.gen {
					continue
				}
				ix.stamp[ti] = ix.gen
				b := ix.boxes[ti]
				if b.Max.X < x0 || b.Min.X > x1 || b.Max.Y < y0 || b.Min.Y > y1 {
					continue
				}
				if !fn(int(ti)) {
					return
				}
			}
		}
	}
}

// zAt returns the Z of triangle t's plane above (x, y), and whether (x, y)
// lies inside t's XY projection, edges included within eps. A triangle
// whose projection has no area (a vertical wall) never contains a point.
func zAt(t [3]r3.Vec, x, y, eps float64) (float64, bool) {
	ax, ay := t[0].X, t[0].Y
	bx, by := t[1].X, t[1].Y
	cx, cy := t[2].X, t[2].Y
	area2 := (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)
	if math.Abs(area2) <= eps*eps {
		return 0, false
	}
	sign := 1.0
	if area2 < 0 {
		sign = -1
	}
	edge := func(px, py, qx, qy float64) bool {
		e := ((qx-px)*(y-py) - (qy-py)*(x-px)) * sign
		return e >= -eps*math.Hypot(qx-px, qy-py)
	}
	if !edge(ax, ay, bx, by) || !edge(bx, by, cx, cy) || !edge(cx, cy, ax, ay) {
		return 0, false
	}
	wa := ((bx-x)*(cy-y) - (by-y)*(cx-x)) / area2
	wb := ((cx-x)*(ay-y) - (cy-y)*(ax-x)) / area2
	wc := 1 - wa - wb
	return wa*t[0].Z + wb*t[1].Z + wc*t[2].Z, true
}

// lowestHit returns the lowest Z at or above zFrom where a vertical line
// through (x, y) meets the model, and the triangle it meets there. ok is
// false when the line meets nothing at or above zFrom.
func (ix *index) lowestHit(x, y, zFrom, eps float64) (float64, int, bool) {
	best, bestTri := math.Inf(1), -1
	ix.query(x-eps, y-eps, x+eps, y+eps, func(i int) bool {
		z, ok := zAt(ix.tris[i], x, y, eps)
		if ok && z >= zFrom-eps && z < best {
			best, bestTri = z, i
		}
		return true
	})
	return best, bestTri, bestTri >= 0
}

// lowestOver returns the lowest Z of any part of the model whose XY
// projection falls inside the square of half-size half centered on (x, y),
// or +Inf when no part does. Each triangle is clipped to the square, and the
// minimum is taken over the clipped polygon's vertices, so the result is
// exact for planar triangles.
func (ix *index) lowestOver(x, y, half float64) float64 {
	best := math.Inf(1)
	x0, y0, x1, y1 := x-half, y-half, x+half, y+half
	ix.query(x0, y0, x1, y1, func(i int) bool {
		if ix.boxes[i].Min.Z >= best {
			return true
		}
		for _, v := range clipToRect(ix.tris[i][:], x0, y0, x1, y1) {
			best = math.Min(best, v.Z)
		}
		return true
	})
	return best
}

// lowestOverAbove is lowestOver restricted to the part of the model at or
// above zFloor: each triangle is also clipped to the half-space Z >= zFloor.
// It returns +Inf when nothing of the model is there.
func (ix *index) lowestOverAbove(x, y, half, zFloor float64) float64 {
	best := math.Inf(1)
	x0, y0, x1, y1 := x-half, y-half, x+half, y+half
	ix.query(x0, y0, x1, y1, func(i int) bool {
		bb := ix.boxes[i]
		if bb.Max.Z < zFloor || bb.Min.Z >= best {
			return true
		}
		poly := clipToRect(ix.tris[i][:], x0, y0, x1, y1)
		poly = clipAbove(poly, zFloor)
		for _, v := range poly {
			best = math.Min(best, v.Z)
		}
		return true
	})
	return best
}

// clipAbove clips a polygon to the half-space Z >= z.
func clipAbove(poly []r3.Vec, z float64) []r3.Vec {
	var out []r3.Vec
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		dp, dq := p.Z-z, q.Z-z
		if dp >= 0 {
			out = append(out, p)
		}
		if (dp >= 0) != (dq >= 0) {
			s := dp / (dp - dq)
			out = append(out, p.Add(q.Sub(p).Scale(s)))
		}
	}
	return out
}

// clipToRect clips a polygon against the XY rectangle by Sutherland-Hodgman,
// interpolating Z along cut edges.
func clipToRect(poly []r3.Vec, x0, y0, x1, y1 float64) []r3.Vec {
	// Signed distance to each rectangle side, non-negative inside.
	planes := []func(v r3.Vec) float64{
		func(v r3.Vec) float64 { return v.X - x0 },
		func(v r3.Vec) float64 { return x1 - v.X },
		func(v r3.Vec) float64 { return v.Y - y0 },
		func(v r3.Vec) float64 { return y1 - v.Y },
	}
	out := poly
	for _, d := range planes {
		if len(out) == 0 {
			return nil
		}
		in := out
		out = nil
		for i := range in {
			p, q := in[i], in[(i+1)%len(in)]
			dp, dq := d(p), d(q)
			if dp >= 0 {
				out = append(out, p)
			}
			if (dp >= 0) != (dq >= 0) {
				s := dp / (dp - dq)
				out = append(out, p.Add(q.Sub(p).Scale(s)))
			}
		}
	}
	return out
}
