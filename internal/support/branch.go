package support

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A branch is a support that cannot stand straight under its contact: its
// foot is somewhere else on the plate. From the plate up it is
//
//   - a foot cone from BaseWidthMM to PillarWidthMM across, BaseThicknessMM
//     tall, at (FootX, FootY);
//   - a vertical shaft PillarWidthMM across up to KneeZ;
//   - a lean, PillarWidthMM across, from (FootX, FootY, KneeZ) to the tip
//     start (X, Y, TopZ - TipHeightMM), at most MaxLeanDeg from vertical;
//   - the tip, as on a pillar.
//
// Branches never root on the model.

// minShaftMM is the shortest vertical shaft a branch gets between its foot
// and its knee, so every span of the sweep is long enough for decad's mitred
// joins.
const minBranchShaftMM = 1.0

// branchSides is the number of sides of the polygon a branch is swept with.
const branchSides = 16

// zoneSides is the number of sides of the polygons that enclose a branch's
// round sections in clearance checks. An 8-gon of circumradius r/cos(pi/8)
// encloses a circle of radius r.
const zoneSides = 8

// IsBranch reports whether pl is a branch rather than a straight pillar.
func (pl Pillar) IsBranch() bool { return pl.Origin == OriginBranch }

// path returns a branch's path points from the plate up and the radius at
// each.
func (p Params) path(pl Pillar) ([]r3.Vec, []float64) {
	foot := r3.NewVec(pl.FootX, pl.FootY, 0)
	tip := pl.TopZ - p.TipHeightMM
	return []r3.Vec{
			foot,
			foot.Add(r3.NewVec(0, 0, p.BaseThicknessMM)),
			r3.NewVec(pl.FootX, pl.FootY, pl.KneeZ),
			r3.NewVec(pl.X, pl.Y, tip),
			r3.NewVec(pl.X, pl.Y, pl.TopZ),
		}, []float64{
			p.BaseWidthMM / 2, p.PillarWidthMM / 2, p.PillarWidthMM / 2, p.PillarWidthMM / 2, p.ContactWidthMM / 2,
		}
}

// branchBody sweeps a branch as one decad body: a regular branchSides-gon of
// radius BaseWidthMM/2 drawn on the XY plane, swept with mitred joins along
// the branch's path (built from the origin, then moved to the foot) and
// scaled at each path point to that point's radius.
func (p Params) branchBody(ctx context.Context, w *sketch.World, doc *decad.Document, pl Pillar) (*decad.Body, error) {
	pts, radii := p.path(pl)
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	poly, err := s.CreatePolygon(0, 0, branchSides, radii[0])
	if err != nil {
		return nil, err
	}
	s.Fix(poly.Center)
	for _, v := range poly.Vertices {
		s.Fix(v)
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, fmt.Errorf("support: solving branch profile: %w", err)
	}
	segs := make([]decad.PathSegment, 0, len(pts)-1)
	scales := make([]units.Value, 0, len(pts)-1)
	for k := 1; k < len(pts); k++ {
		segs = append(segs, decad.LineTo{End: pts[k].Sub(pts[0])})
		scales = append(scales, units.Scalar(radii[k]/radii[0]))
	}
	path, err := decad.NewPath(r3.Vec{}, segs...)
	if err != nil {
		return nil, fmt.Errorf("support: branch path: %w", err)
	}
	body, err := doc.Sweep(ctx, s, s.Profiles()[0], path, decad.WithMitredJoins(), decad.WithSectionScale(scales...))
	if err != nil {
		return nil, fmt.Errorf("support: sweeping branch: %w", err)
	}
	mv, err := r3.Translation(pts[0])
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, mv)
}

// branchVolume returns a branch's volume as the sum of cone frustums along
// its path: within the mitre wedges it differs from the swept body by less
// than the wedges' volume.
func (p Params) branchVolume(pl Pillar) float64 {
	pts, radii := p.path(pl)
	var v float64
	for k := 1; k < len(pts); k++ {
		h := pts[k].Sub(pts[k-1]).Len()
		r1, r2 := radii[k-1], radii[k]
		v += math.Pi * h * (r1*r1 + r1*r2 + r2*r2) / 3
	}
	return v
}

// segmentZone returns a convex solid enclosing a cone frustum from a (radius
// ra) to b (radius rb), each end lengthened by ext along the axis: two
// zoneSides-gons in planes perpendicular to the axis.
func segmentZone(a, b r3.Vec, ra, rb, ext float64) convex {
	axis, _ := b.Sub(a).Normalize()
	a = a.Sub(axis.Scale(ext))
	b = b.Add(axis.Scale(ext))
	ref := r3.NewVec(1, 0, 0)
	if math.Abs(axis.X) > 0.9 {
		ref = r3.NewVec(0, 1, 0)
	}
	u, _ := ref.Cross(axis).Normalize()
	v := axis.Cross(u)
	grow := 1 / math.Cos(math.Pi/zoneSides)
	ring := func(c r3.Vec, r float64) []r3.Vec {
		out := make([]r3.Vec, zoneSides)
		for i := range zoneSides {
			th := 2 * math.Pi * float64(i) / zoneSides
			out[i] = c.Add(u.Scale(r * grow * math.Cos(th))).Add(v.Scale(r * grow * math.Sin(th)))
		}
		return out
	}
	lo, hi := ring(a, ra), ring(b, rb)
	cv := convex{verts: append(lo, hi...), normals: []r3.Vec{axis}, edges: []r3.Vec{}}
	for i := range zoneSides {
		j := (i + 1) % zoneSides
		edge := lo[j].Sub(lo[i])
		side := hi[i].Sub(lo[i])
		cv.edges = append(cv.edges, edge, side)
		cv.normals = append(cv.normals, edge.Cross(side))
	}
	return cv
}

// branchZones returns the solids around a branch that must not touch the
// model: foot, shaft and lean widened by margin, and the tip swept up by the
// top gap. With margin 0 they enclose the branch itself.
func (p Params) branchZones(pl Pillar, margin float64) []convex {
	pts, radii := p.path(pl)
	rs := p.PillarWidthMM / 2
	zones := []convex{
		frustum(pl.FootX, pl.FootY, 0, p.BaseWidthMM/2+margin, p.BaseThicknessMM, p.BaseWidthMM/2+margin),
		frustum(pl.FootX, pl.FootY, p.BaseThicknessMM, rs+margin, pl.KneeZ+rs, rs+margin),
		// The lean is lengthened by its radius at both ends, so the zone
		// also covers the mitre wedges at the knee and at the tip start.
		segmentZone(pts[2], pts[3], radii[2]+margin, radii[3]+margin, rs),
	}
	tip := frustum(pl.X, pl.Y, pts[3].Z, rs, pl.TopZ, p.ContactWidthMM/2)
	if margin == 0 {
		return append(zones, tip)
	}
	return append(zones, sweptUp(tip, p.TopGapMM))
}

// touchesConvex reports whether two convex solids share a point, by
// separating axes: both shapes' face normals and the cross products of their
// edge directions.
func touchesConvex(a, b convex, eps float64) bool {
	try := func(axis r3.Vec) bool {
		l := axis.Len()
		if l < 1e-12 {
			return false
		}
		n := axis.Scale(1 / l)
		amin, amax := project(a.verts, n)
		bmin, bmax := project(b.verts, n)
		return amax < bmin-eps || bmax < amin-eps
	}
	if slices.ContainsFunc(a.normals, try) || slices.ContainsFunc(b.normals, try) {
		return false
	}
	for _, ea := range a.edges {
		for _, eb := range b.edges {
			if try(ea.Cross(eb)) {
				return false
			}
		}
	}
	return true
}

// Reasons a branch could not be placed.
const (
	ReasonNoBranch = "no branch from the plate reaches it: every foot position within the lean limit collides or is crowded"
)

// supportSolids returns the solids that enclose an accepted support, for
// support-to-support checks.
func (p Params) supportSolids(pl Pillar) []convex {
	if pl.Tree > 0 {
		return p.memberZones(pl, 0)
	}
	if pl.IsBranch() {
		return p.branchZones(pl, 0)
	}
	base, shaft, contact := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
	tip := p.tipStart(pl)
	return []convex{
		frustum(pl.X, pl.Y, 0, base, p.BaseThicknessMM, base),
		frustum(pl.X, pl.Y, p.BaseThicknessMM, shaft, tip, shaft),
		frustum(pl.X, pl.Y, tip, shaft, pl.TopZ, contact),
	}
}

// branchPass gives each sample no pillar covers a branch, when one fits. A
// branch's tip goes under the sample (the sample's own position first, then
// its nearby positions); its foot is the nearest plate position, on a
// SpacingMM/4 grid around the tip and nearest first, from which a shaft and
// a lean of at most MaxLeanDeg reach the tip start with every zone clear of
// the model and of the other supports.
func (b *builder) branchPass(ctx context.Context, uncovered []Sample) ([]Sample, error) {
	if b.p.MaxLeanDeg <= 0 {
		return uncovered, nil
	}
	var still []Sample
	for _, u := range uncovered {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if b.covered(u.Point) {
			continue
		}
		ok, err := b.placeBranch(ctx, u.Point)
		if err != nil {
			return nil, err
		}
		if !ok {
			still = append(still, Sample{Point: u.Point, Reason: u.Reason + "; " + ReasonNoBranch})
		}
	}
	return still, nil
}

// placeBranch finds a tip for sample s and adds a single branch, standing on
// its own foot, for it. It reports whether it placed one. Branches join
// trunks afterwards, in mergePass.
func (b *builder) placeBranch(ctx context.Context, s r3.Vec) (bool, error) {
	for _, tip := range b.tipPositions(s) {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		surf, tri, ok := b.ix.lowestHit(tip[0], tip[1], s.Z-b.dzMax, b.eps)
		if !ok || b.kinds[tri] != overhang.Demand || math.Abs(surf-s.Z) > b.dzMax+b.eps {
			continue
		}
		top := b.ix.lowestOverAbove(tip[0], tip[1], b.p.ContactWidthMM/2, s.Z-b.dzMax) - b.p.TopGapMM - b.slack
		pl := Pillar{X: tip[0], Y: tip[1], SurfaceZ: surf, TopZ: top, Origin: OriginBranch}
		if !b.covers(pl, s) {
			continue
		}
		if found, ok := b.findFoot(pl); ok {
			return true, b.add(found)
		}
	}
	return false, nil
}

// tipPositions returns the sample's own position, then the points of a
// SpacingMM/4 grid within SpacingMM/2 of it, nearest first.
func (b *builder) tipPositions(s r3.Vec) [][2]float64 {
	q := b.p.SpacingMM / sampleDivisions
	out := [][2]float64{{s.X, s.Y}}
	type spot struct{ x, y, d float64 }
	var spots []spot
	cx, cy := math.Round(s.X/q), math.Round(s.Y/q)
	for dy := -2; dy <= 2; dy++ {
		for dx := -2; dx <= 2; dx++ {
			x, y := (cx+float64(dx))*q, (cy+float64(dy))*q
			if d := math.Hypot(x-s.X, y-s.Y); d > 0 && d <= b.p.SpacingMM/2 {
				spots = append(spots, spot{x, y, d})
			}
		}
	}
	slices.SortFunc(spots, func(a, c spot) int {
		return cmp.Or(cmp.Compare(a.d, c.d), cmp.Compare(a.y, c.y), cmp.Compare(a.x, c.x))
	})
	for _, sp := range spots {
		out = append(out, [2]float64{sp.x, sp.y})
	}
	return out
}

// findFoot looks for the nearest foot position for a branch whose tip is
// already set in pl.
func (b *builder) findFoot(pl Pillar) (Pillar, bool) {
	tipStart := pl.TopZ - b.p.TipHeightMM
	lowestKnee := b.p.BaseThicknessMM + minBranchShaftMM
	slope := math.Tan(b.p.MaxLeanDeg * math.Pi / 180)
	reach := (tipStart - lowestKnee) * slope
	if reach <= 0 {
		return Pillar{}, false
	}
	q := b.p.SpacingMM / sampleDivisions
	n := int(math.Ceil(reach / q))
	type spot struct{ x, y, d float64 }
	var spots []spot
	cx, cy := math.Round(pl.X/q), math.Round(pl.Y/q)
	for dy := -n; dy <= n; dy++ {
		for dx := -n; dx <= n; dx++ {
			x, y := (cx+float64(dx))*q, (cy+float64(dy))*q
			d := math.Hypot(x-pl.X, y-pl.Y)
			if d < q/2 || d > reach {
				continue
			}
			spots = append(spots, spot{x, y, d})
		}
	}
	slices.SortFunc(spots, func(a, c spot) int {
		return cmp.Or(cmp.Compare(a.d, c.d), cmp.Compare(a.y, c.y), cmp.Compare(a.x, c.x))
	})
	for _, sp := range spots {
		cand := pl
		cand.FootX, cand.FootY = sp.x, sp.y
		cand.KneeZ = tipStart - sp.d/slope
		if b.branchClear(cand) {
			return cand, true
		}
	}
	return Pillar{}, false
}

// branchClear reports whether a branch's zones are clear of the model and
// of every accepted support, the latter with SideClearanceMM between them.
func (b *builder) branchClear(pl Pillar) bool {
	for _, z := range b.p.branchZones(pl, b.p.SideClearanceMM) {
		if _, hit := b.ix.anyTouching(z, b.eps); hit {
			return false
		}
	}
	mine := b.p.branchZones(pl, b.p.SideClearanceMM)
	for _, theirs := range b.solidsExcept(nil, 0) {
		for _, z := range mine[:3] {
			if touchesConvex(z, theirs, b.eps) {
				return false
			}
		}
	}
	return true
}
