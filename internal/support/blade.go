package support

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A blade is a thin wall standing on the plate across a strip: a long,
// narrow, flat demand region. Blades at the strip's ends, and between them
// where the strip is longer than MaxBridgeMM, turn the strip into a row of
// short bridges, so it needs no field of pillars. Seen along the strip, a
// blade has a pillar's section: a base BaseWidthMM thick and BaseThicknessMM
// tall, a wall PillarWidthMM thick, and a tip narrowing to ContactWidthMM
// over TipHeightMM, its top face TopGapMM (plus a numeric slack) below the
// strip. Across the strip it is LengthMM long.

// Strip shape rules. See findStrips.
const (
	// stripMaxWidthSpacings caps a strip's width, in support spacings: a
	// region narrower than two spacings is what one row of pillars covers.
	stripMaxWidthSpacings = 2
	// stripMinLengthSpacings is the shortest strip, in support spacings: a
	// shorter region fits under the reach of one pillar.
	stripMinLengthSpacings = 2
	// stripMinAspect is the least length-to-width ratio of a strip.
	stripMinAspect = 3
	// stripMinFill is the least share of its bounding rectangle a strip's
	// projected area fills, so an L-shaped or curved region is no strip.
	stripMinFill = 0.9
)

// Blade is one blade support. Its base is centered on (X, Y).
type Blade struct {
	X, Y float64
	// AxisX and AxisY are the horizontal unit direction of the blade's
	// thickness: the long axis of the strip it holds.
	AxisX, AxisY float64
	// LengthMM is the blade's horizontal length across the strip.
	LengthMM float64
	// SurfaceZ is where the vertical line through (X, Y) first meets the
	// part; TopZ is the height of the blade's top face.
	SurfaceZ, TopZ float64
	// Strip is the 1-based number of the strip the blade was placed for.
	Strip int
}

// across returns the horizontal unit direction of the blade's length.
func (bl Blade) across() (float64, float64) { return -bl.AxisY, bl.AxisX }

// local returns (x, y)'s offset from the blade's center along its axis and
// across it.
func (bl Blade) local(x, y float64) (float64, float64) {
	dx, dy := x-bl.X, y-bl.Y
	nx, ny := bl.across()
	return dx*bl.AxisX + dy*bl.AxisY, dx*nx + dy*ny
}

// rect returns the corners, counter-clockwise, of the rectangle centered on
// the blade's center with half-size ha along the axis and hn across it.
func (bl Blade) rect(ha, hn float64) [4][2]float64 {
	nx, ny := bl.across()
	corner := func(sa, sn float64) [2]float64 {
		return [2]float64{bl.X + sa*ha*bl.AxisX + sn*hn*nx, bl.Y + sa*ha*bl.AxisY + sn*hn*ny}
	}
	return [4][2]float64{corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)}
}

// box returns the convex solid between the blade-aligned rectangle of
// half-sizes (ha0, hn0) at z0 and the one of half-sizes (ha1, hn1) at z1.
func (bl Blade) box(z0, ha0, hn0, z1, ha1, hn1 float64) convex {
	at := func(r [4][2]float64, z float64) []r3.Vec {
		out := make([]r3.Vec, 4)
		for k, c := range r {
			out[k] = r3.NewVec(c[0], c[1], z)
		}
		return out
	}
	lo, hi := at(bl.rect(ha0, hn0), z0), at(bl.rect(ha1, hn1), z1)
	nx, ny := bl.across()
	c := convex{
		verts:   append(lo, hi...),
		normals: []r3.Vec{axisZ},
		edges:   []r3.Vec{r3.NewVec(bl.AxisX, bl.AxisY, 0), r3.NewVec(nx, ny, 0)},
	}
	for k := range 4 {
		side := hi[k].Sub(lo[k])
		c.edges = append(c.edges, side)
		c.normals = append(c.normals, lo[(k+1)%4].Sub(lo[k]).Cross(side))
	}
	return c
}

// bladeLevels returns the blade's section along its axis from the plate up,
// as a pillar's levels: each a height and a half-thickness.
func (p Params) bladeLevels(bl Blade) []level { return p.levels(Pillar{TopZ: bl.TopZ}) }

// bladeZones returns the solids around a blade that must not touch the
// model: the base and the wall widened on every side by margin, and the tip
// swept up by the top gap. With margin 0 the tip is not swept, and the
// solids enclose the blade itself.
func (p Params) bladeZones(bl Blade, margin float64) []convex {
	base, shaft, contact := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
	hn := bl.LengthMM / 2
	tip := p.tipStart(Pillar{TopZ: bl.TopZ})
	zones := []convex{bl.box(0, base+margin, hn+margin, p.BaseThicknessMM, base+margin, hn+margin)}
	if tip > p.BaseThicknessMM {
		zones = append(zones, bl.box(p.BaseThicknessMM, shaft+margin, hn+margin, tip, shaft+margin, hn+margin))
	}
	top := bl.box(tip, shaft, hn, bl.TopZ, contact, hn)
	if margin == 0 {
		return append(zones, top)
	}
	return append(zones, sweptUp(top, p.TopGapMM))
}

// bladeOutline returns the blade's section as a closed polygon in (along
// the axis, height) coordinates, counter-clockwise from the right end of
// the base's underside.
func (p Params) bladeOutline(bl Blade) [][2]float64 {
	lv := p.bladeLevels(bl)
	out := make([][2]float64, 0, 2*len(lv))
	for _, l := range lv {
		out = append(out, [2]float64{l.half, l.z})
	}
	for _, l := range slices.Backward(lv) {
		out = append(out, [2]float64{-l.half, l.z})
	}
	return out
}

// BladeBody builds the blade as a decad body in doc: its section outline,
// drawn in a new sketch in w, extruded LengthMM and turned so the section
// stands across the strip. One extrusion, so no boolean is involved.
func (p Params) BladeBody(ctx context.Context, w *sketch.World, doc *decad.Document, bl Blade) (*decad.Body, error) {
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	outline := p.bladeOutline(bl)
	pts := make([]*sketch.Point, len(outline))
	for i, o := range outline {
		pts[i] = s.CreatePoint(o[0], o[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, fmt.Errorf("support: solving blade outline: %w", err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, fmt.Errorf("support: blade outline gave %d profiles", len(profiles))
	}
	body, err := decad.Extrude(ctx, doc, s, profiles[0], decad.Distance{D: units.Millimeters(bl.LengthMM), Dir: decad.Along})
	if err != nil {
		return nil, fmt.Errorf("support: extruding blade: %w", err)
	}
	// The sketch's U becomes the blade's axis, its V becomes +Z, and the
	// extrusion runs across the strip, centered on the blade.
	ex := r3.NewVec(bl.AxisX, bl.AxisY, 0)
	ez := ex.Cross(axisZ)
	origin := r3.NewVec(bl.X, bl.Y, 0).Sub(ez.Scale(bl.LengthMM / 2))
	stand, err := r3.FromBasis(r3.Basis{EX: ex, EY: axisZ, EZ: ez}, origin)
	if err != nil {
		return nil, err
	}
	return decad.Transform(ctx, body, stand)
}

// BladeVolume returns the volume in mm^3 of the body BladeBody builds: the
// section outline's area times LengthMM.
func (p Params) BladeVolume(bl Blade) float64 {
	o := p.bladeOutline(bl)
	var a float64
	for i := range o {
		q := o[(i+1)%len(o)]
		a += o[i][0]*q[1] - q[0]*o[i][1]
	}
	return math.Abs(a) / 2 * bl.LengthMM
}

// BladeBounds returns the axis-aligned box that holds the blade: its base
// rectangle's corners from the plate up to its top face.
func (p Params) BladeBounds(bl Blade) mesh.Box {
	r := bl.rect(p.BaseWidthMM/2, bl.LengthMM/2)
	box := mesh.Box{Min: r3.NewVec(r[0][0], r[0][1], 0), Max: r3.NewVec(r[0][0], r[0][1], bl.TopZ)}
	for _, c := range r[1:] {
		box = box.Union(mesh.Box{Min: r3.NewVec(c[0], c[1], 0), Max: r3.NewVec(c[0], c[1], bl.TopZ)})
	}
	return box
}

// BladeContactArea returns the area in mm^2 of a blade's top face.
func (p Params) BladeContactArea(bl Blade) float64 { return p.ContactWidthMM * bl.LengthMM }

// strip is a demand region that blades may hold. u runs along its unit axis
// (ax, ay), v across it along (-ay, ax).
type strip struct {
	id      int
	ax, ay  float64
	v0, v1  float64
	samples []int // indices into the sample list, by u, then v
	sampleU []float64
	sampleV []float64
}

func (st *strip) uv(x, y float64) (float64, float64) {
	return x*st.ax + y*st.ay, -x*st.ay + y*st.ax
}

func (st *strip) at(u, v float64) (float64, float64) {
	return u*st.ax - v*st.ay, u*st.ay + v*st.ax
}

// regions groups the demand triangles into edge- or vertex-connected
// regions. It returns each demand triangle's region root (a triangle index)
// and -1 for every other triangle.
func (b *builder) regions() []int {
	parent := make([]int, len(b.tris))
	for i := range parent {
		parent[i] = -1
	}
	find := func(i int) int {
		for parent[i] != i {
			parent[i] = parent[parent[i]]
			i = parent[i]
		}
		return i
	}
	owner := make(map[uint32]int)
	for i, t := range b.tris {
		if b.kinds[i] != overhang.Demand {
			continue
		}
		parent[i] = i
		for _, v := range t {
			o, ok := owner[v]
			if !ok {
				owner[v] = i
				continue
			}
			ra, rb := find(o), find(i)
			if ra == rb {
				continue
			}
			// The lower index becomes the root, so roots do not depend on
			// the order of the unions.
			parent[max(ra, rb)] = min(ra, rb)
		}
	}
	root := make([]int, len(b.tris))
	for i := range root {
		root[i] = -1
		if parent[i] >= 0 {
			root[i] = find(i)
		}
	}
	return root
}

// findStrips returns the demand regions blades may hold, numbered from 1 in
// the order of their first sample. A region is a strip when bridges are on
// and:
//
//   - every triangle is tilted at most MaxBridgeTiltDeg from horizontal;
//   - its smallest-area bounding rectangle in the plate's plane (one side
//     along an edge of the convex hull of its vertices) is at least
//     PillarWidthMM and at most stripMaxWidthSpacings support spacings wide,
//     longer than stripMinLengthSpacings spacings, and stripMinAspect times
//     longer than wide;
//   - its projected area fills at least stripMinFill of that rectangle.
//
// It also sets b.stripOf for every triangle of a strip.
func (b *builder) findStrips(ctx context.Context, samples []r3.Vec, sampleTri []int) ([]*strip, error) {
	b.stripOf = make([]int, len(b.tris))
	if b.p.MaxBridgeMM <= 0 {
		return nil, nil
	}
	root := b.regions()
	members := make(map[int][]int)
	for i, r := range root {
		if r >= 0 {
			members[r] = append(members[r], i)
		}
	}
	byRoot := make(map[int]*strip)
	var out []*strip
	for si, ti := range sampleTri {
		if si%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		r := root[ti]
		st, ok := byRoot[r]
		if !ok {
			st = b.stripShape(members[r])
			byRoot[r] = st
			if st != nil {
				st.id = len(out) + 1
				out = append(out, st)
			}
		}
		if st == nil {
			continue
		}
		u, v := st.uv(samples[si].X, samples[si].Y)
		st.samples = append(st.samples, si)
		st.sampleU = append(st.sampleU, u)
		st.sampleV = append(st.sampleV, v)
	}
	for i, r := range root {
		if r < 0 {
			continue
		}
		if st := byRoot[r]; st != nil {
			b.stripOf[i] = st.id
		}
	}
	for _, st := range out {
		order := make([]int, len(st.samples))
		for i := range order {
			order[i] = i
		}
		slices.SortFunc(order, func(i, j int) int {
			return cmp.Or(cmp.Compare(st.sampleU[i], st.sampleU[j]), cmp.Compare(st.sampleV[i], st.sampleV[j]))
		})
		ss, su, sv := make([]int, len(order)), make([]float64, len(order)), make([]float64, len(order))
		for k, i := range order {
			ss[k], su[k], sv[k] = st.samples[i], st.sampleU[i], st.sampleV[i]
		}
		st.samples, st.sampleU, st.sampleV = ss, su, sv
	}
	return out, nil
}

// stripShape returns the strip made of the region's triangles, or nil when
// the region is not one (see findStrips).
func (b *builder) stripShape(tris []int) *strip {
	var pts [][2]float64
	var area float64
	for _, i := range tris {
		t := b.ix.tris[i]
		n, _ := mesh.TriangleNormal(t)
		if overhang.Angle(n) > b.p.MaxBridgeTiltDeg {
			return nil
		}
		area += math.Abs((t[1].X-t[0].X)*(t[2].Y-t[0].Y)-(t[1].Y-t[0].Y)*(t[2].X-t[0].X)) / 2
		for _, v := range t {
			pts = append(pts, [2]float64{v.X, v.Y})
		}
	}
	hull := convexHull(pts)
	if len(hull) < 3 {
		return nil
	}
	// Smallest-area rectangle with a side along a hull edge.
	bestArea := math.Inf(1)
	var ax, ay float64
	for i := range hull {
		ex, ey := hull[(i+1)%len(hull)][0]-hull[i][0], hull[(i+1)%len(hull)][1]-hull[i][1]
		l := math.Hypot(ex, ey)
		dx, dy := ex/l, ey/l
		lu, hu, lv, hv := extent(hull, dx, dy)
		a := (hu - lu) * (hv - lv)
		if a < bestArea {
			bestArea = a
			if hu-lu >= hv-lv {
				ax, ay = dx, dy
			} else {
				ax, ay = -dy, dx
			}
		}
	}
	if ax < 0 || (ax == 0 && ay < 0) {
		ax, ay = -ax, -ay
	}
	u0, u1, v0, v1 := extent(hull, ax, ay)
	length, width := u1-u0, v1-v0
	sp := b.p.SpacingMM
	switch {
	case width < b.p.PillarWidthMM-b.eps,
		width > stripMaxWidthSpacings*sp+b.eps,
		length <= stripMinLengthSpacings*sp,
		length < stripMinAspect*width,
		area < stripMinFill*length*width:
		return nil
	}
	return &strip{ax: ax, ay: ay, v0: v0, v1: v1}
}

// extent returns the range of pts along unit direction (dx, dy) and across
// it, along (-dy, dx).
func extent(pts [][2]float64, dx, dy float64) (float64, float64, float64, float64) {
	lu, hu, lv, hv := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, p := range pts {
		u, v := p[0]*dx+p[1]*dy, -p[0]*dy+p[1]*dx
		lu, hu = math.Min(lu, u), math.Max(hu, u)
		lv, hv = math.Min(lv, v), math.Max(hv, v)
	}
	return lu, hu, lv, hv
}

// convexHull returns the convex hull of pts, counter-clockwise from the
// lowest-X (then lowest-Y) point, without collinear points (Andrew's
// monotone chain). The result does not depend on the order of pts.
func convexHull(pts [][2]float64) [][2]float64 {
	ps := slices.Clone(pts)
	slices.SortFunc(ps, func(a, c [2]float64) int { return cmp.Or(cmp.Compare(a[0], c[0]), cmp.Compare(a[1], c[1])) })
	ps = slices.Compact(ps)
	if len(ps) < 3 {
		return ps
	}
	cross := func(o, a, c [2]float64) float64 {
		return (a[0]-o[0])*(c[1]-o[1]) - (a[1]-o[1])*(c[0]-o[0])
	}
	var hull [][2]float64
	for _, p := range ps {
		for len(hull) >= 2 && cross(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, p)
	}
	lower := len(hull) + 1
	for i := len(ps) - 2; i >= 0; i-- {
		p := ps[i]
		for len(hull) >= lower && cross(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, p)
	}
	return hull[:len(hull)-1]
}

// bladePass gives each strip blades, strip by strip in order (see
// placeStrip). A strip keeps its blades only when they hold or bridge every
// one of its samples; otherwise they are taken out again and the strip is
// left to pillars. It returns, for each sample, whether blades cover it.
func (b *builder) bladePass(ctx context.Context, samples []r3.Vec, sampleTri []int) ([]bool, error) {
	strips, err := b.findStrips(ctx, samples, sampleTri)
	if err != nil {
		return nil, err
	}
	b.bladed = make(map[int]struct{})
	for _, st := range strips {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		before := len(b.blades)
		ok, err := b.placeStrip(ctx, st, samples)
		if err != nil {
			return nil, err
		}
		if ok {
			for _, si := range st.samples {
				if !b.bladeCovers(samples[si], b.tiltOf(sampleTri[si])) {
					ok = false
					break
				}
			}
		}
		if !ok {
			b.blades = b.blades[:before]
			continue
		}
		b.bladed[st.id] = struct{}{}
	}
	covered := make([]bool, len(samples))
	if len(b.blades) == 0 {
		return covered, nil
	}
	for si, s := range samples {
		if si%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		covered[si] = b.bladeCovers(s, b.tiltOf(sampleTri[si]))
	}
	return covered, nil
}

func (b *builder) tiltOf(tri int) float64 {
	n, _ := mesh.TriangleNormal(b.ix.tris[tri])
	return overhang.Angle(n)
}

// placeStrip places blades along a strip, from its low-u end to its high-u
// end. A strip end next to a model wall (the model's cross-section one layer
// below the end sample, within MaxBridgeMM along the axis) needs no blade:
// the wall is that end's anchor. A free end gets a blade whose top face ends
// exactly at the last sample. Between the anchors, blades go where the
// remaining length splits into equal spans of at most MaxBridgeMM between
// top faces; a blade that does not fit there moves toward the previous
// anchor, in SpacingMM/16 steps, as long as a span remains. It reports
// false when a blade fits nowhere.
func (b *builder) placeStrip(ctx context.Context, st *strip, samples []r3.Vec) (bool, error) {
	n := len(st.samples)
	if n == 0 {
		return false, nil
	}
	tc := b.p.ContactWidthMM
	maxB := b.p.MaxBridgeMM
	step := b.p.SpacingMM / nearbyDivisions
	first, last := samples[st.samples[0]], samples[st.samples[n-1]]
	uFirst, uLast := st.sampleU[0], st.sampleU[n-1]

	var edge float64
	if d, ok := b.wallDistance(first, -st.ax, -st.ay); ok {
		edge = uFirst - d
	} else {
		placed, err := b.placeBladeNear(ctx, st, []float64{uFirst + tc/2})
		if err != nil || !placed {
			return false, err
		}
		edge = uFirst + tc
	}
	farWall, walled := 0.0, false
	if d, ok := b.wallDistance(last, st.ax, st.ay); ok {
		farWall, walled = uLast+d, true
	}
	for {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if walled && farWall-edge <= maxB+b.eps {
			return true, nil
		}
		if !walled && edge >= uLast-b.eps {
			return true, nil
		}
		var target float64
		var centers []float64
		if walled {
			r := farWall - edge
			m := math.Ceil((r - maxB) / (maxB + tc))
			span := (r - m*tc) / (m + 1)
			target = edge + span + tc/2
		} else {
			r := uLast - edge
			m := math.Ceil(r / (maxB + tc))
			span := r/m - tc
			target = edge + span + tc/2
			if m <= 1 {
				// The last blade ends at the strip's free end.
				centers = []float64{uLast - tc/2}
			}
		}
		if centers == nil {
			hi := math.Min(edge+maxB+tc/2, uLast-tc/2)
			for c := math.Min(target, hi); c-tc/2 > edge+b.eps; c -= step {
				centers = append(centers, c)
			}
		}
		placed, err := b.placeBladeNear(ctx, st, centers)
		if err != nil || !placed {
			return false, err
		}
		edge = b.blades[len(b.blades)-1].localU(st) + tc/2
	}
}

// localU returns the blade's center along its strip's axis.
func (bl Blade) localU(st *strip) float64 {
	u, _ := st.uv(bl.X, bl.Y)
	return u
}

// placeBladeNear adds the first blade that fits at one of the centers (along
// the strip's axis), each tried across the strip's full width first and
// then with its ends drawn in (see fitBlade). It reports whether it added
// one.
func (b *builder) placeBladeNear(ctx context.Context, st *strip, centers []float64) (bool, error) {
	step := b.p.SpacingMM / nearbyDivisions
	width := st.v1 - st.v0
	maxTrim := int(math.Floor((width - b.p.PillarWidthMM + b.eps) / step))
	for _, c := range centers {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		trims := [][2]float64{{0, 0}}
		for k := 1; k <= maxTrim; k++ {
			t := float64(k) * step
			trims = append(trims, [2]float64{t, 0}, [2]float64{0, t})
			if 2*k <= maxTrim {
				trims = append(trims, [2]float64{t, t})
			}
		}
		for _, tr := range trims {
			bl, reason := b.fitBlade(st, c, st.v0+tr[0], st.v1-tr[1])
			if reason != "" {
				continue
			}
			return true, b.addBlade(bl)
		}
	}
	return false, nil
}

// fitBlade decides whether a blade centered at u = c on the strip's axis,
// spanning v in [lo, hi] across it, can stand. It returns the blade or the
// reason it cannot: the vertical line through its center must first meet
// the strip; the model's lowest point over its top face must be no lower
// than the strip's tilt allows across that face, and leave room for
// MinHeight plus TopGapMM; no clearance zone may touch the model; and it
// must not touch any accepted support.
func (b *builder) fitBlade(st *strip, c, lo, hi float64) (Blade, string) {
	x, y := st.at(c, (lo+hi)/2)
	surf, tri, ok := b.ix.lowestHit(x, y, 0, b.eps)
	if !ok {
		return Blade{}, ReasonNoSurface
	}
	if b.stripOf[tri] != st.id {
		return Blade{}, ReasonOccluded
	}
	bl := Blade{X: x, Y: y, AxisX: st.ax, AxisY: st.ay, LengthMM: hi - lo, SurfaceZ: surf, Strip: st.id}
	face := bl.rect(b.p.ContactWidthMM/2, bl.LengthMM/2)
	lowest := b.ix.lowestOverPoly(face[:], math.Inf(-1))
	reach := math.Hypot(b.p.ContactWidthMM/2, bl.LengthMM/2) * math.Tan(b.p.MaxBridgeTiltDeg*math.Pi/180)
	if surf-lowest > reach+b.eps {
		return Blade{}, ReasonOccluded
	}
	bl.TopZ = lowest - b.p.TopGapMM - b.slack
	if bl.TopZ < b.p.MinHeight() {
		return Blade{}, ReasonTooLow
	}
	for _, z := range b.p.bladeZones(bl, b.p.SideClearanceMM) {
		if _, hit := b.ix.anyTouching(z, b.eps); hit {
			return Blade{}, ReasonCollision
		}
	}
	mine := b.p.bladeZones(bl, 0)
	for _, o := range b.solidsExcept(nil, 0) {
		for _, z := range mine {
			if touchesConvex(z, o, b.eps) {
				return Blade{}, ReasonCrowded
			}
		}
	}
	return bl, ""
}

func (b *builder) addBlade(bl Blade) error {
	if len(b.pillars)+len(b.blades) == b.lim.MaxSupports {
		return fmt.Errorf("%w: more than %d supports needed", ErrLimit, b.lim.MaxSupports)
	}
	b.blades = append(b.blades, bl)
	return nil
}

// wallDistance returns the distance from sample s along unit direction
// (dx, dy) to the model's cross-section one layer below s, and false when
// it is farther than MaxBridgeMM.
func (b *builder) wallDistance(s r3.Vec, dx, dy float64) (float64, bool) {
	h := s.Z - b.p.LayerHeightMM
	if h <= 0 {
		return 0, false
	}
	segs := b.sliceNear(s.X, s.Y, h, b.p.MaxBridgeMM)
	return firstHit(segs, s.X, s.Y, dx, dy, b.p.MaxBridgeMM)
}

// bladeCovers reports whether blades hold demand sample s, on a surface
// tilted tiltDeg from horizontal. Only blades whose held surface is within
// the coverage rule's height band of s count:
//
//   - held: s lies over a blade's top face;
//   - bridged: the surface is tilted at most MaxBridgeTiltDeg, and along a
//     blade's axis or across it, the nearest anchors on both sides of s are
//     at most MaxBridgeMM apart, at least one of them a blade's top face.
//     The other anchors are the model's cross-section one layer below s.
func (b *builder) bladeCovers(s r3.Vec, tiltDeg float64) bool {
	maxB := b.p.MaxBridgeMM
	var near []Blade
	for _, bl := range b.blades {
		if math.Abs(s.Z-bl.SurfaceZ) > b.dzMax+b.eps {
			continue
		}
		if math.Hypot(s.X-bl.X, s.Y-bl.Y) > maxB+bl.LengthMM/2+b.p.ContactWidthMM {
			continue
		}
		du, dv := bl.local(s.X, s.Y)
		if math.Abs(du) <= b.p.ContactWidthMM/2+b.eps && math.Abs(dv) <= bl.LengthMM/2+b.eps {
			return true
		}
		near = append(near, bl)
	}
	if len(near) == 0 || maxB <= 0 || tiltDeg > b.p.MaxBridgeTiltDeg {
		return false
	}
	var tops []segment2
	for _, bl := range near {
		r := bl.rect(b.p.ContactWidthMM/2, bl.LengthMM/2)
		for k := range 4 {
			p, q := r[k], r[(k+1)%4]
			tops = append(tops, segment2{p[0], p[1], q[0], q[1]})
		}
	}
	var walls []segment2
	if h := s.Z - b.p.LayerHeightMM; h > 0 {
		walls = b.sliceNear(s.X, s.Y, h, maxB)
	}
	hit := func(dx, dy float64) (float64, bool, bool) {
		dt, okT := firstHit(tops, s.X, s.Y, dx, dy, maxB)
		dw, okW := firstHit(walls, s.X, s.Y, dx, dy, maxB)
		switch {
		case okT && (!okW || dt <= dw):
			return dt, true, true
		case okW:
			return dw, false, true
		}
		return 0, false, false
	}
	for _, bl := range near {
		nx, ny := bl.across()
		for _, d := range [][2]float64{{bl.AxisX, bl.AxisY}, {nx, ny}} {
			fwd, bladeF, okF := hit(d[0], d[1])
			back, bladeB, okB := hit(-d[0], -d[1])
			if okF && okB && (bladeF || bladeB) && fwd+back <= maxB+b.eps {
				return true
			}
		}
	}
	return false
}

// bladeTouches reports whether solid c touches any accepted blade.
func (b *builder) bladeTouches(c convex) bool {
	for _, bl := range b.blades {
		for _, z := range b.p.bladeZones(bl, 0) {
			if touchesConvex(c, z, b.eps) {
				return true
			}
		}
	}
	return false
}
