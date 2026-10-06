package support

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A tree is a trunk several branches share. The trunk stands on the plate
// at (FootX, FootY) and rises vertically to TopZ. Each member branch leaves
// it at its own knee: its path starts on the trunk's axis a stub below the
// knee (RootZ), rises to the knee, and leans from there to its tip as a
// single branch would. A tree is built as one body: the trunk sweep with
// every member sweep unioned onto it.

// Tree is one trunk and the bookkeeping its members need.
type Tree struct {
	FootX, FootY float64
	// TopZ is the trunk's top: the highest member knee.
	TopZ float64
	// Members counts the branches the trunk carries.
	Members int
}

// Tree limits. See the package documentation of tree.go.
const (
	// maxTreeMembers keeps a tree within the number of unions decad
	// chains onto one body (about 15 as of decad 85163ab).
	maxTreeMembers = 14
	// treeStubMM is the vertical start of a member's path, inside the
	// trunk: decad's sweep must start along the profile's normal.
	treeStubMM = 1.0
)

// trunkRadius returns the radius of a trunk carrying n branches: the
// radius whose cross-section equals n branch cross-sections.
func (p Params) trunkRadius(n int) float64 {
	return p.PillarWidthMM / 2 * math.Sqrt(float64(max(1, n)))
}

// trunkBaseRadius widens the trunk at the plate by the same margin a
// pillar's base has over its shaft.
func (p Params) trunkBaseRadius(n int) float64 {
	return p.trunkRadius(n) + (p.BaseWidthMM-p.PillarWidthMM)/2
}

// memberPath returns a tree member's path from its root up and the radius at
// each point.
func (p Params) memberPath(pl Pillar) ([]r3.Vec, []float64) {
	rs := p.PillarWidthMM / 2
	tip := pl.TopZ - p.TipHeightMM
	return []r3.Vec{
			r3.NewVec(pl.FootX, pl.FootY, pl.RootZ),
			r3.NewVec(pl.FootX, pl.FootY, pl.KneeZ),
			r3.NewVec(pl.X, pl.Y, tip),
			r3.NewVec(pl.X, pl.Y, pl.TopZ),
		}, []float64{
			rs, rs, rs, p.ContactWidthMM / 2,
		}
}

// trunkZones returns the solids around a trunk: its base and its shaft,
// widened by margin.
func (p Params) trunkZones(t Tree, margin float64) []convex {
	rb, r := p.trunkBaseRadius(t.Members), p.trunkRadius(t.Members)
	return []convex{
		frustum(t.FootX, t.FootY, 0, rb+margin, p.BaseThicknessMM, rb+margin),
		frustum(t.FootX, t.FootY, p.BaseThicknessMM, r+margin, t.TopZ+r, r+margin),
	}
}

// memberZones returns the solids around a member's lean and tip. The lean is
// widened by margin; the tip is swept up by the top gap when margin is not
// 0. The member's stub lies inside the trunk and has no zone of its own.
func (p Params) memberZones(pl Pillar, margin float64) []convex {
	pts, radii := p.memberPath(pl)
	rs := p.PillarWidthMM / 2
	zones := []convex{segmentZone(pts[1], pts[2], radii[1]+margin, radii[2]+margin, rs)}
	tip := frustum(pl.X, pl.Y, pts[2].Z, rs, pl.TopZ, p.ContactWidthMM/2)
	if margin == 0 {
		return append(zones, tip)
	}
	return append(zones, sweptUp(tip, p.TopGapMM))
}

// sweepAlong sweeps a regular branchSides-gon of radius radii[0] along pts
// with mitred joins, scaled to radii[k] at pts[k]. The path is built from
// the origin on the XY plane, so its first span must be vertical, and the
// body is then moved to pts[0].
func sweepAlong(ctx context.Context, w *sketch.World, doc *decad.Document, pts []r3.Vec, radii []float64) (*decad.Body, error) {
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
		return nil, fmt.Errorf("support: solving sweep profile: %w", err)
	}
	segs := make([]decad.PathSegment, 0, len(pts)-1)
	scales := make([]units.Value, 0, len(pts)-1)
	for k := 1; k < len(pts); k++ {
		segs = append(segs, decad.LineTo{End: pts[k].Sub(pts[0])})
		scales = append(scales, units.Scalar(radii[k]/radii[0]))
	}
	path, err := decad.NewPath(r3.Vec{}, segs...)
	if err != nil {
		return nil, fmt.Errorf("support: sweep path: %w", err)
	}
	var opts []decad.SweepOption
	if len(segs) > 1 {
		opts = append(opts, decad.WithMitredJoins())
	}
	opts = append(opts, decad.WithSectionScale(scales...))
	body, err := doc.Sweep(ctx, s, s.Profiles()[0], path, opts...)
	if err != nil {
		return nil, fmt.Errorf("support: sweeping: %w", err)
	}
	mv, err := r3.Translation(pts[0])
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, mv)
}

// TreeBody builds a tree as one decad body: the trunk swept from the plate to
// TopZ, each member swept from its root and unioned onto it in the order
// given, and, when the trunk is at least four extrusion widths wide, a bore
// cut out of the trunk below the lowest member root (decad's sealed-bore
// recipe: a thinner sweep kept inside the trunk at both ends).
func (p Params) TreeBody(ctx context.Context, w *sketch.World, doc *decad.Document, t Tree, members []Pillar) (*decad.Body, error) {
	r := p.trunkRadius(t.Members)
	trunk, err := sweepAlong(ctx, w, doc,
		[]r3.Vec{r3.NewVec(t.FootX, t.FootY, 0), r3.NewVec(t.FootX, t.FootY, p.BaseThicknessMM), r3.NewVec(t.FootX, t.FootY, t.TopZ)},
		[]float64{p.trunkBaseRadius(t.Members), r, r})
	if err != nil {
		return nil, fmt.Errorf("support: trunk: %w", err)
	}
	lowestRoot := math.Inf(1)
	for _, m := range members {
		pts, radii := p.memberPath(m)
		mb, err := sweepAlong(ctx, w, doc, pts, radii)
		if err != nil {
			return nil, fmt.Errorf("support: tree member: %w", err)
		}
		if trunk, err = decad.Union(ctx, trunk, mb); err != nil {
			return nil, fmt.Errorf("support: joining a member to its trunk: %w", err)
		}
		lowestRoot = math.Min(lowestRoot, m.RootZ)
	}
	bore, lo, hi := p.boreSpan(r, lowestRoot)
	if bore <= 0 {
		return trunk, nil
	}
	b, err := sweepAlong(ctx, w, doc, []r3.Vec{r3.NewVec(t.FootX, t.FootY, lo), r3.NewVec(t.FootX, t.FootY, hi)}, []float64{bore, bore})
	if err != nil {
		return nil, fmt.Errorf("support: trunk bore: %w", err)
	}
	if trunk, err = decad.Cut(ctx, trunk, b); err != nil {
		return nil, fmt.Errorf("support: cutting the trunk bore: %w", err)
	}
	return trunk, nil
}

// boreSpan returns the radius and height range of a trunk's bore, or a zero
// radius when the trunk gets none. The wall keeps two extrusion widths; the
// bore starts 1 mm above the base and ends a branch radius plus 1 mm below
// the lowest member root, and must be at least 2 mm long.
func (p Params) boreSpan(trunkR, lowestRoot float64) (float64, float64, float64) {
	if p.ExtrusionWidthMM <= 0 || 2*trunkR < 4*p.ExtrusionWidthMM {
		return 0, 0, 0
	}
	bore := trunkR - 2*p.ExtrusionWidthMM
	lo := p.BaseThicknessMM + 1
	hi := lowestRoot - p.PillarWidthMM/2 - 1
	if bore < p.MinFeatureMM/2 || hi-lo < 2 {
		return 0, 0, 0
	}
	return bore, lo, hi
}

// treeVolume estimates a tree's volume as the trunk's frustums plus each
// member's frustums outside the trunk, less the bore.
func (p Params) treeVolume(t Tree, members []Pillar) float64 {
	frust := func(h, r1, r2 float64) float64 { return math.Pi * h * (r1*r1 + r1*r2 + r2*r2) / 3 }
	r := p.trunkRadius(t.Members)
	v := frust(p.BaseThicknessMM, p.trunkBaseRadius(t.Members), r) + frust(t.TopZ-p.BaseThicknessMM, r, r)
	lowestRoot := math.Inf(1)
	for _, m := range members {
		pts, radii := p.memberPath(m)
		for k := 2; k < len(pts); k++ {
			v += frust(pts[k].Sub(pts[k-1]).Len(), radii[k-1], radii[k])
		}
		lowestRoot = math.Min(lowestRoot, m.RootZ)
	}
	if bore, lo, hi := p.boreSpan(r, lowestRoot); bore > 0 {
		v -= frust(hi-lo, bore, bore)
	}
	return v
}

// mergePass re-routes single branches onto trunks. Each single branch, in
// plan order, is taken out and offered to attach with its tip unchanged; if
// no tree or other single branch can take it, it goes back as it was. The
// branches all fitted as singles, and taking one out only frees room, so the
// pass never loses coverage.
func (b *builder) mergePass(ctx context.Context) error {
	for i := 0; i < len(b.pillars); i++ {
		if err := ctx.Err(); err != nil {
			return err
		}
		pl := b.pillars[i]
		if !pl.IsBranch() || pl.Tree != 0 {
			continue
		}
		b.removeAt(i)
		tip := pl
		tip.FootX, tip.FootY, tip.KneeZ = 0, 0, 0
		ok, err := b.attach(tip)
		if err != nil {
			return err
		}
		if !ok {
			b.insertAt(i, pl)
			continue
		}
		// attach appended the re-routed branch at the end; the pillar
		// now at index i has not been visited yet.
		i--
	}
	return nil
}

// removeAt takes pillar i out of the builder and re-indexes the cells.
func (b *builder) removeAt(i int) {
	b.pillars = slices.Delete(b.pillars, i, i+1)
	b.reindex()
}

// insertAt puts pl back at index i and re-indexes the cells.
func (b *builder) insertAt(i int, pl Pillar) {
	b.pillars = slices.Insert(b.pillars, i, pl)
	b.reindex()
}

func (b *builder) reindex() {
	clear(b.cells)
	for j, o := range b.pillars {
		c := b.cellOf(o.X, o.Y)
		b.cells[c] = append(b.cells[c], j)
	}
}

// attach tries to put a branch for tip pl onto an existing tree, or to turn
// an existing single branch into a two-member tree. Candidates are taken by
// the distance from their foot to the tip, nearest first. It reports whether
// it attached.
func (b *builder) attach(pl Pillar) (bool, error) {
	tipStart := pl.TopZ - b.p.TipHeightMM
	slope := math.Tan(b.p.MaxLeanDeg * math.Pi / 180)

	type cand struct {
		tree   int // 1-based tree, or 0 for a single branch
		branch int // index of a single branch in b.pillars
		fx, fy float64
		d      float64
	}
	var cands []cand
	for i, t := range b.trees {
		if t.Members >= maxTreeMembers {
			continue
		}
		cands = append(cands, cand{tree: i + 1, fx: t.FootX, fy: t.FootY, d: math.Hypot(pl.X-t.FootX, pl.Y-t.FootY)})
	}
	for i, o := range b.pillars {
		if o.IsBranch() && o.Tree == 0 {
			cands = append(cands, cand{branch: i, fx: o.FootX, fy: o.FootY, d: math.Hypot(pl.X-o.FootX, pl.Y-o.FootY)})
		}
	}
	slices.SortFunc(cands, func(a, c cand) int {
		return cmp.Or(cmp.Compare(a.d, c.d), cmp.Compare(a.fy, c.fy), cmp.Compare(a.fx, c.fx))
	})

	for _, c := range cands {
		knee := tipStart - c.d/slope
		root := knee - treeStubMM
		if root < b.p.BaseThicknessMM+1 {
			continue
		}
		next := pl
		next.Origin = OriginBranch
		next.FootX, next.FootY, next.KneeZ, next.RootZ = c.fx, c.fy, knee, root

		var t Tree
		var members []Pillar
		var memberIdx []int
		if c.tree > 0 {
			t = b.trees[c.tree-1]
			for i, o := range b.pillars {
				if o.Tree == c.tree {
					members = append(members, o)
					memberIdx = append(memberIdx, i)
				}
			}
		} else {
			first := b.pillars[c.branch]
			first.RootZ = first.KneeZ - treeStubMM
			if first.RootZ < b.p.BaseThicknessMM+1 {
				continue
			}
			t = Tree{FootX: c.fx, FootY: c.fy, TopZ: first.KneeZ, Members: 1}
			members = []Pillar{first}
			memberIdx = []int{c.branch}
		}
		if !b.kneeApart(knee, members) {
			continue
		}
		grown := t
		grown.Members++
		grown.TopZ = math.Max(t.TopZ, knee)
		if !b.treeClear(grown, append(slices.Clone(members), next), memberIdx, c.tree) {
			continue
		}

		treeID := c.tree
		if treeID == 0 {
			b.trees = append(b.trees, grown)
			treeID = len(b.trees)
			first := b.pillars[c.branch]
			first.RootZ = first.KneeZ - treeStubMM
			first.Tree = treeID
			b.pillars[c.branch] = first
		} else {
			b.trees[treeID-1] = grown
		}
		next.Tree = treeID
		return true, b.add(next)
	}
	return false, nil
}

// kneeApart reports whether a new member's knee keeps its stub and the start
// of its lean clear of every existing member's inside the trunk: the knees
// must differ by at least a stub plus a branch diameter. decad's unions can
// fail when members overlap each other inside a trunk.
func (b *builder) kneeApart(knee float64, members []Pillar) bool {
	gap := treeStubMM + b.p.PillarWidthMM
	for _, m := range members {
		if math.Abs(m.KneeZ-knee) < gap {
			return false
		}
	}
	return true
}

// treeClear reports whether a grown tree fits: its trunk and every member's
// lean and tip, with clearance, touch no model triangle; they keep
// SideClearanceMM from every support outside the tree; and the members'
// leans do not touch each other. memberIdx lists the existing members'
// indices in b.pillars; self is the tree's 1-based id, or 0 for a tree being
// made from a single branch.
func (b *builder) treeClear(t Tree, members []Pillar, memberIdx []int, self int) bool {
	sc := b.p.SideClearanceMM
	var mine []convex
	mine = append(mine, b.p.trunkZones(t, sc)...)
	for _, m := range members {
		mine = append(mine, b.p.memberZones(m, sc)...)
	}
	for _, z := range mine {
		if _, hit := b.ix.anyTouching(z, b.eps); hit {
			return false
		}
	}
	// Members of one tree must not touch each other outside the trunk.
	for i := range members {
		for j := i + 1; j < len(members); j++ {
			if touchesConvex(b.p.memberZones(members[i], 0)[0], b.p.memberZones(members[j], 0)[0], b.eps) &&
				!b.leansMeetInsideTrunk(t, members[i], members[j]) {
				return false
			}
		}
	}
	others := b.solidsExcept(memberIdx, self)
	for _, z := range mine {
		for _, o := range others {
			if touchesConvex(z, o, b.eps) {
				return false
			}
		}
	}
	return true
}

// leansMeetInsideTrunk reports whether two members' leans touch only within
// the trunk: their zones, cut to the part beyond the trunk's radius plus a
// branch radius from the axis, are apart.
func (b *builder) leansMeetInsideTrunk(t Tree, m1, m2 Pillar) bool {
	out := func(m Pillar) convex {
		pts, radii := b.p.memberPath(m)
		dir, _ := pts[2].Sub(pts[1]).Normalize()
		horiz := math.Hypot(dir.X, dir.Y)
		if horiz == 0 {
			return segmentZone(pts[1], pts[2], radii[1], radii[2], 0)
		}
		skip := (b.p.trunkRadius(t.Members) + b.p.PillarWidthMM/2) / horiz
		start := pts[1].Add(dir.Scale(math.Min(skip, pts[2].Sub(pts[1]).Len())))
		return segmentZone(start, pts[2], radii[1], radii[2], 0)
	}
	return !touchesConvex(out(m1), out(m2), b.eps)
}

// solidsExcept returns the solids of every accepted support except the
// pillars listed in skip and the tree with id self (0: none).
func (b *builder) solidsExcept(skip []int, self int) []convex {
	var out []convex
	for i, o := range b.pillars {
		if slices.Contains(skip, i) || (self > 0 && o.Tree == self) {
			continue
		}
		out = append(out, b.p.supportSolids(o)...)
	}
	for i, t := range b.trees {
		if i+1 == self {
			continue
		}
		out = append(out, b.p.trunkZones(t, 0)...)
	}
	return out
}
