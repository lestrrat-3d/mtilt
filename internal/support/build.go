package support

import (
	"cmp"
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
)

// ErrLimit is returned when building supports would exceed a work limit.
var ErrLimit = errors.New("support: work limit reached")

// Limits bound the work Build does.
type Limits struct {
	// MaxSupports is the most pillars Build places.
	MaxSupports int
	// MaxSamples is the most demand sample points Build generates.
	MaxSamples int
}

// Reasons a position cannot hold a pillar, as reported for uncovered
// samples.
const (
	ReasonOccluded  = "occluded: the vertical line from the plate meets other model geometry first"
	ReasonTooLow    = "too low: less room than the shortest pillar plus the top gap"
	ReasonCollision = "collision: the pillar or its clearance zone would touch the model"
	ReasonCrowded   = "crowded: the pillar base would overlap another pillar's base"
	ReasonNoSurface = "no model surface above this position"
)

// Sample is one support-demand sample point and, when no pillar covers it,
// why the position below it cannot hold one.
type Sample struct {
	Point  r3.Vec `json:"point_mm"`
	Reason string `json:"reason,omitempty"`
}

// Plan is the result of Build.
type Plan struct {
	// Pillars are sorted by Y, then X.
	Pillars []Pillar
	// DemandTriangles counts the triangles classified overhang.Demand.
	DemandTriangles int
	// Samples counts the demand sample points.
	Samples int
	// Uncovered lists the samples no pillar covers, sorted by Y, X, Z.
	// A plan with any uncovered sample must not be used.
	Uncovered []Sample

	samples []r3.Vec
}

// Grid resolutions, as steps per support spacing. See Build.
const (
	// sampleDivisions sets the demand sample grid.
	sampleDivisions = 4
	// nearbyDivisions sets the grid of nearby positions tried for a
	// sample.
	nearbyDivisions = 16
)

// slackFactor scales the numeric slack left between a pillar's swept gap
// zone and the surface above it, in units of the serialization tolerance.
// See Build.
const slackFactor = 4

// Build places pillars under the support demand of a placed model.
//
// Demand is every triangle overhang.Classify marks as Demand. Build samples
// it on a square grid of pitch SpacingMM/4 anchored at the origin (every
// grid point inside a demand triangle's XY projection, at the triangle's
// height there) plus every demand triangle's vertices.
//
// Coverage rule: a sample (x, y, z) is covered by a pillar at (px, py) whose
// held surface is at height sz when hypot(x-px, y-py) <= SpacingMM and
// |z - sz| <= SpacingMM * max(1, tan(ThresholdDeg)). The height condition
// keeps a pillar under one surface from counting for another surface
// stacked above it.
//
// Pillar positions come from three passes. Samples are visited in Y, X, Z
// order, and "nearby positions" are the points of a SpacingMM/16 grid
// anchored at the origin within SpacingMM of a sample, nearest first (ties
// by Y, then X).
//
//  1. Edge: each uncovered sample whose own position the model rules out
//     (occluded, too low, or a clearance collision; typically next to a
//     wall) gets the first accepted nearby position that covers it. Running this first keeps grid pillars from taking the
//     only room left beside a wall.
//  2. Grid: every node of a SpacingMM grid anchored at the origin, inside
//     the samples' XY bounding box, in Y-then-X order.
//  3. Fill: each sample still uncovered tries its own position, then the
//     nearby positions, and takes the first accepted one that covers it.
//
// A position is accepted when the vertical line through it first meets a
// demand triangle; the part's lowest point over the ContactWidthMM top square
// leaves room for MinHeight plus TopGapMM; no clearance zone (see
// clearanceZones) touches the model; and the base is apart from every
// accepted base. The top face is placed TopGapMM plus a numeric slack
// (slackFactor times tol.Serialization) below that lowest point.
//
// Samples that remain uncovered are listed in Plan.Uncovered with the reason
// their own position was refused. Build returns ErrLimit when the samples
// or pillars would exceed lim, and ctx.Err() when ctx is cancelled.
func Build(ctx context.Context, placed *mesh.Mesh, p Params, lim Limits, tol mesh.Tolerance) (*Plan, error) {
	b := &builder{
		ix:    newIndex(placed),
		p:     p,
		lim:   lim,
		eps:   tol.Length,
		slack: slackFactor * tol.Serialization,
		dzMax: p.SpacingMM * math.Max(1, math.Tan(p.ThresholdDeg*math.Pi/180)),
		kinds: make([]overhang.Kind, len(placed.Triangles)),
		cells: make(map[[2]int64][]int),
	}
	plan := &Plan{}
	for i := range placed.Triangles {
		b.kinds[i] = overhang.Classify(placed.Triangle(i), p.ThresholdDeg, tol.Plate)
		if b.kinds[i] == overhang.Demand {
			plan.DemandTriangles++
		}
	}
	if plan.DemandTriangles == 0 {
		return plan, nil
	}

	samples, err := b.samples(ctx)
	if err != nil {
		return nil, err
	}
	plan.samples = samples
	plan.Samples = len(samples)

	if err := b.edgePass(ctx, samples); err != nil {
		return nil, err
	}
	if err := b.gridPass(ctx, samples); err != nil {
		return nil, err
	}
	uncovered, err := b.fillPass(ctx, samples)
	if err != nil {
		return nil, err
	}
	plan.Uncovered = uncovered

	plan.Pillars = slices.Clone(b.pillars)
	slices.SortFunc(plan.Pillars, func(a, c Pillar) int { return cmp.Or(cmp.Compare(a.Y, c.Y), cmp.Compare(a.X, c.X)) })
	return plan, nil
}

type builder struct {
	ix    *index
	p     Params
	lim   Limits
	eps   float64
	slack float64
	dzMax float64
	kinds []overhang.Kind

	pillars []Pillar
	// cells buckets pillar indices by SpacingMM grid cell for coverage
	// and crowding lookups.
	cells map[[2]int64][]int
}

func (b *builder) cellOf(x, y float64) [2]int64 {
	return [2]int64{int64(math.Floor(x / b.p.SpacingMM)), int64(math.Floor(y / b.p.SpacingMM))}
}

// nearby calls fn with every pillar in the 3x3 cells around (x, y). With
// cells SpacingMM wide that includes every pillar within SpacingMM.
func (b *builder) nearby(x, y float64, fn func(Pillar) bool) bool {
	c := b.cellOf(x, y)
	for dy := int64(-1); dy <= 1; dy++ {
		for dx := int64(-1); dx <= 1; dx++ {
			for _, i := range b.cells[[2]int64{c[0] + dx, c[1] + dy}] {
				if fn(b.pillars[i]) {
					return true
				}
			}
		}
	}
	return false
}

func (b *builder) covers(pl Pillar, s r3.Vec) bool {
	return math.Hypot(s.X-pl.X, s.Y-pl.Y) <= b.p.SpacingMM+b.eps && math.Abs(s.Z-pl.SurfaceZ) <= b.dzMax+b.eps
}

func (b *builder) covered(s r3.Vec) bool {
	return b.nearby(s.X, s.Y, func(pl Pillar) bool { return b.covers(pl, s) })
}

func (b *builder) add(pl Pillar) error {
	if len(b.pillars) == b.lim.MaxSupports {
		return fmt.Errorf("%w: more than %d supports needed", ErrLimit, b.lim.MaxSupports)
	}
	b.pillars = append(b.pillars, pl)
	c := b.cellOf(pl.X, pl.Y)
	b.cells[c] = append(b.cells[c], len(b.pillars)-1)
	return nil
}

func (b *builder) samples(ctx context.Context) ([]r3.Vec, error) {
	q := b.p.SpacingMM / sampleDivisions
	seen := make(map[r3.Vec]struct{})
	var out []r3.Vec
	push := func(v r3.Vec) error {
		if _, ok := seen[v]; ok {
			return nil
		}
		if len(out) == b.lim.MaxSamples {
			return fmt.Errorf("%w: more than %d support-demand samples", ErrLimit, b.lim.MaxSamples)
		}
		seen[v] = struct{}{}
		out = append(out, v)
		return nil
	}
	for i, t := range b.ix.tris {
		if b.kinds[i] != overhang.Demand {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		for _, v := range t {
			if err := push(v); err != nil {
				return nil, err
			}
		}
		bb := b.ix.boxes[i]
		for gy := math.Ceil(bb.Min.Y / q); gy*q <= bb.Max.Y; gy++ {
			for gx := math.Ceil(bb.Min.X / q); gx*q <= bb.Max.X; gx++ {
				x, y := gx*q, gy*q
				z, ok := zAt(t, x, y, b.eps)
				if !ok {
					continue
				}
				if err := push(r3.NewVec(x, y, z)); err != nil {
					return nil, err
				}
			}
		}
	}
	slices.SortFunc(out, func(a, c r3.Vec) int {
		return cmp.Or(cmp.Compare(a.Y, c.Y), cmp.Compare(a.X, c.X), cmp.Compare(a.Z, c.Z))
	})
	return out, nil
}

// accept decides whether a pillar can stand at (x, y). It returns the pillar
// or the reason it cannot.
func (b *builder) accept(x, y float64, origin Origin) (Pillar, string) {
	surf, tri, ok := b.ix.lowestHit(x, y, 0, b.eps)
	if !ok {
		return Pillar{}, ReasonNoSurface
	}
	if b.kinds[tri] != overhang.Demand {
		return Pillar{}, ReasonOccluded
	}
	top := b.ix.lowestOver(x, y, b.p.ContactWidthMM/2) - b.p.TopGapMM - b.slack
	if top < b.p.MinHeight() {
		return Pillar{}, ReasonTooLow
	}
	pl := Pillar{X: x, Y: y, SurfaceZ: surf, TopZ: top, Origin: origin}
	for _, z := range b.p.clearanceZones(pl) {
		if _, hit := b.ix.anyTouching(z, b.eps); hit {
			return Pillar{}, ReasonCollision
		}
	}
	if b.crowded(pl) {
		return Pillar{}, ReasonCrowded
	}
	return pl, ""
}

// crowded reports whether pl's base square would touch or overlap an
// accepted pillar's base square.
func (b *builder) crowded(pl Pillar) bool {
	w := b.p.BaseWidthMM + b.eps
	return b.nearby(pl.X, pl.Y, func(o Pillar) bool {
		return math.Abs(o.X-pl.X) < w && math.Abs(o.Y-pl.Y) < w
	})
}

func (b *builder) gridPass(ctx context.Context, samples []r3.Vec) error {
	lo, hi := samples[0], samples[0]
	for _, s := range samples {
		lo = r3.NewVec(math.Min(lo.X, s.X), math.Min(lo.Y, s.Y), 0)
		hi = r3.NewVec(math.Max(hi.X, s.X), math.Max(hi.Y, s.Y), 0)
	}
	sp := b.p.SpacingMM
	for gy := math.Ceil(lo.Y / sp); gy*sp <= hi.Y; gy++ {
		if err := ctx.Err(); err != nil {
			return err
		}
		for gx := math.Ceil(lo.X / sp); gx*sp <= hi.X; gx++ {
			pl, reason := b.accept(gx*sp, gy*sp, OriginGrid)
			if reason != "" {
				continue
			}
			if err := b.add(pl); err != nil {
				return err
			}
		}
	}
	return nil
}

func (b *builder) edgePass(ctx context.Context, samples []r3.Vec) error {
	for _, s := range samples {
		if err := ctx.Err(); err != nil {
			return err
		}
		if b.covered(s) {
			continue
		}
		// Only samples the model itself keeps a pillar away from count
		// here; a position that is merely crowded by an earlier pillar is
		// left to the later passes.
		if _, reason := b.accept(s.X, s.Y, OriginFill); reason == "" || reason == ReasonCrowded {
			continue
		}
		if _, err := b.fillNear(s); err != nil {
			return err
		}
	}
	return nil
}

func (b *builder) fillPass(ctx context.Context, samples []r3.Vec) ([]Sample, error) {
	var uncovered []Sample
	for _, s := range samples {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if b.covered(s) {
			continue
		}
		pl, ownReason := b.accept(s.X, s.Y, OriginFill)
		if ownReason == "" && b.covers(pl, s) {
			if err := b.add(pl); err != nil {
				return nil, err
			}
			continue
		}
		if ownReason == "" {
			// The position holds a pillar, but for a different surface.
			ownReason = ReasonOccluded
		}
		placed, err := b.fillNear(s)
		if err != nil {
			return nil, err
		}
		if !placed {
			uncovered = append(uncovered, Sample{Point: s, Reason: ownReason})
		}
	}
	return uncovered, nil
}

// fillNear tries the nearby positions of s (see Build) and adds the first
// accepted pillar that covers s.
func (b *builder) fillNear(s r3.Vec) (bool, error) {
	q := b.p.SpacingMM / nearbyDivisions
	reach := nearbyDivisions
	cx, cy := math.Round(s.X/q), math.Round(s.Y/q)
	type spot struct{ x, y, d float64 }
	var spots []spot
	for dy := -reach; dy <= reach; dy++ {
		for dx := -reach; dx <= reach; dx++ {
			x, y := (cx+float64(dx))*q, (cy+float64(dy))*q
			d := math.Hypot(x-s.X, y-s.Y)
			if d > b.p.SpacingMM {
				continue
			}
			spots = append(spots, spot{x, y, d})
		}
	}
	slices.SortFunc(spots, func(a, c spot) int {
		return cmp.Or(cmp.Compare(a.d, c.d), cmp.Compare(a.y, c.y), cmp.Compare(a.x, c.x))
	})
	for _, sp := range spots {
		pl, reason := b.accept(sp.x, sp.y, OriginFill)
		if reason != "" || !b.covers(pl, s) {
			continue
		}
		return true, b.add(pl)
	}
	return false, nil
}
