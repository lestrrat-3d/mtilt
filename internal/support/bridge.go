package support

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// bridgeDirections is the number of horizontal directions, spread over half
// a turn, along which bridged looks for anchors on both sides of a sample.
const bridgeDirections = 8

// spanKind is what bridgeCheck finds under a demand sample.
type spanKind int

const (
	// spanNone: the sample needs support.
	spanNone spanKind = iota
	// spanHeld: the sample sits on top of a wall, within the length
	// tolerance of the cross-section below it, so the wall holds it.
	spanHeld
	// spanBridged: the sample is on a short bridge between two walls.
	spanBridged
)

// bridgeCheck decides whether demand sample s, on a surface tilted tiltDeg
// from horizontal, needs no support because of the walls around it. It looks
// at the model's cross-section one layer below s along bridgeDirections
// horizontal lines through s, spread over half a turn:
//
//   - when the cross-section passes within the length tolerance of s, s sits
//     on a wall and is held;
//   - otherwise, when the surface is tilted at most MaxBridgeTiltDeg and along
//     some line the cross-section lies on both sides of s with at most
//     MaxBridgeMM of air between them, s is on a bridge the printer draws in
//     the air.
//
// A MaxBridgeMM of 0 turns both off.
func (b *builder) bridgeCheck(s r3.Vec, tiltDeg float64) spanKind {
	if b.p.MaxBridgeMM <= 0 {
		return spanNone
	}
	h := s.Z - b.p.LayerHeightMM
	if h <= 0 {
		return spanNone
	}
	reach := b.p.MaxBridgeMM
	segs := b.sliceNear(s.X, s.Y, h, reach)
	if len(segs) == 0 {
		return spanNone
	}
	result := spanNone
	for k := range bridgeDirections {
		th := math.Pi * float64(k) / bridgeDirections
		dx, dy := math.Cos(th), math.Sin(th)
		fwd, okF := firstHit(segs, s.X, s.Y, dx, dy, reach)
		back, okB := firstHit(segs, s.X, s.Y, -dx, -dy, reach)
		if (okF && fwd <= b.eps) || (okB && back <= b.eps) {
			return spanHeld
		}
		if okF && okB && fwd+back <= reach && tiltDeg <= b.p.MaxBridgeTiltDeg {
			result = spanBridged
		}
	}
	return result
}

type segment2 struct {
	ax, ay, bx, by float64
}

// sliceNear returns the XY segments where the model's triangles cross the
// plane Z = h, for triangles whose XY bounds come within reach of (x, y).
func (b *builder) sliceNear(x, y, h, reach float64) []segment2 {
	var out []segment2
	b.ix.query(x-reach, y-reach, x+reach, y+reach, func(i int) bool {
		bb := b.ix.boxes[i]
		if bb.Min.Z > h || bb.Max.Z < h {
			return true
		}
		t := b.ix.tris[i]
		var pts [2][2]float64
		n := 0
		for k := range 3 {
			p, q := t[k], t[(k+1)%3]
			if (p.Z < h) == (q.Z < h) {
				continue
			}
			f := (h - p.Z) / (q.Z - p.Z)
			if n < 2 {
				pts[n] = [2]float64{p.X + (q.X-p.X)*f, p.Y + (q.Y-p.Y)*f}
			}
			n++
		}
		if n == 2 {
			out = append(out, segment2{pts[0][0], pts[0][1], pts[1][0], pts[1][1]})
		}
		return true
	})
	return out
}

// firstHit returns the distance from (x, y) along unit direction (dx, dy) to
// the nearest segment, and false when none lies within maxDist.
func firstHit(segs []segment2, x, y, dx, dy, maxDist float64) (float64, bool) {
	best := math.Inf(1)
	for _, s := range segs {
		ex, ey := s.bx-s.ax, s.by-s.ay
		den := dx*ey - dy*ex
		if den == 0 {
			continue
		}
		wx, wy := s.ax-x, s.ay-y
		t := (wx*ey - wy*ex) / den
		u := (wx*dy - wy*dx) / den
		if t >= 0 && u >= 0 && u <= 1 && t < best {
			best = t
		}
	}
	return best, best <= maxDist
}
