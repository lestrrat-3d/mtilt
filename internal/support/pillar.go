// Package support plans and checks mtilt's breakaway supports: separate
// vertical round pillars rooted on the build plate, each one decad body.
// Planning runs on a tessellation of the placed model; Params.Body turns each
// planned pillar into a decad body.
//
// A pillar, from the plate up:
//
//   - a base disc BaseWidthMM across and BaseThicknessMM tall;
//   - a shaft PillarWidthMM across, up to TipHeightMM below the top;
//   - a tip that narrows from PillarWidthMM to ContactWidthMM across over
//     TipHeightMM;
//   - a ContactWidthMM-wide top face, TopGapMM (plus a numeric slack)
//     below the lowest point of the part above the square that encloses it.
//
// Planning and the clearance zones use the square that encloses each round
// section, so every check made on the squares holds for the round pillar.
//
// The pillar never touches the part: the top gap is left for the slicer to
// bridge or not, as its own settings decide.
package support

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Params are the support dimensions, in millimeters and degrees. Callers
// validate them before use (mtilt.Profile.Validate does).
type Params struct {
	ThresholdDeg    float64
	SpacingMM       float64
	ContactWidthMM  float64
	PillarWidthMM   float64
	TipHeightMM     float64
	BaseWidthMM     float64
	BaseThicknessMM float64
	TopGapMM        float64
	SideClearanceMM float64
	// PlateAnchorMM is the plate-anchor height (see package overhang).
	PlateAnchorMM float64
	// LayerHeightMM is the layer height; bridge anchors are looked for
	// one layer below a sample.
	LayerHeightMM float64
	// MaxBridgeMM is the longest span printed in the air between two
	// walls without support; 0 turns bridges off. MaxBridgeTiltDeg is the
	// steepest surface tilt from horizontal that counts as a bridge.
	MaxBridgeMM      float64
	MaxBridgeTiltDeg float64
	// MaxLeanDeg is the steepest a branch may lean from vertical; 0 turns
	// branches off.
	MaxLeanDeg float64
	// ExtrusionWidthMM and MinFeatureMM size trunk bores (see
	// Params.boreSpan).
	ExtrusionWidthMM float64
	MinFeatureMM     float64
}

// MinHeight is the shortest pillar these params can build: base plus tip.
func (p Params) MinHeight() float64 { return p.BaseThicknessMM + p.TipHeightMM }

// MinReachMM is the lowest surface height a pillar fits under: the shortest
// pillar plus the top gap.
func (p Params) MinReachMM() float64 { return p.MinHeight() + p.TopGapMM }

// minShaftMM is the shortest shaft section a pillar gets. A shaft that would
// be shorter is left out and the tip starts on the base, so a pillar near
// MinHeight never carries a sliver band that rounding left behind.
const minShaftMM = 1e-6

// tipStart returns the height where the tip begins.
func (p Params) tipStart(pl Pillar) float64 {
	tip := pl.TopZ - p.TipHeightMM
	if tip-p.BaseThicknessMM < minShaftMM {
		return p.BaseThicknessMM
	}
	return tip
}

// Origin records how a pillar position was chosen.
type Origin string

// Pillar origins.
const (
	// OriginGrid is a node of the support-spacing grid.
	OriginGrid Origin = "grid"
	// OriginFill is a position added to cover a sample the grid left
	// uncovered.
	OriginFill Origin = "fill"
	// OriginBranch is a branch: its foot stands elsewhere on the plate.
	OriginBranch Origin = "branch"
)

// Pillar is one support pillar, centered on (X, Y).
type Pillar struct {
	X, Y float64
	// SurfaceZ is the height at which the vertical line through (X, Y)
	// first meets the part: the support-demand surface the pillar holds.
	SurfaceZ float64
	// TopZ is the height of the pillar's top face.
	TopZ   float64
	Origin Origin
	// FootX, FootY and KneeZ place a branch's foot and the top of its
	// vertical shaft (see branch.go). They are zero for a straight pillar.
	FootX, FootY, KneeZ float64
	// Tree is the 1-based index into Plan.Trees of the trunk this branch
	// leaves, or 0. RootZ is where a tree member's path starts on the
	// trunk's axis (see tree.go).
	Tree  int
	RootZ float64
}

type level struct {
	z, half float64
}

// levels returns the pillar's sections from the plate up, each as a height
// and a half-width (the radius of the round section). Two
// consecutive sections at the same height form a horizontal ledge.
func (p Params) levels(pl Pillar) []level {
	base, shaft, contact := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
	tip := p.tipStart(pl)
	out := []level{{0, base}, {p.BaseThicknessMM, base}}
	if shaft < base {
		out = append(out, level{p.BaseThicknessMM, shaft})
	}
	if tip > p.BaseThicknessMM {
		out = append(out, level{tip, shaft})
	}
	return append(out, level{pl.TopZ, contact})
}

// Body builds the pillar as a decad body in doc: the stepped outline of
// levels, revolved a full turn about the pillar's axis. Each level's
// half-width becomes a radius, so the base, shaft and contact are round. The
// outline is drawn in a new sketch in w, revolved about the sketch's U axis,
// and placed upright at (X, Y); only the placed body stays live in doc.
func (p Params) Body(ctx context.Context, w *sketch.World, doc *decad.Document, pl Pillar) (*decad.Body, error) {
	if pl.IsBranch() {
		return p.branchBody(ctx, w, doc, pl)
	}
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	// U is height along the pillar axis, V is radius.
	outline := [][2]float64{{0, 0}}
	for _, l := range p.levels(pl) {
		outline = append(outline, [2]float64{l.z, l.half})
	}
	outline = append(outline, [2]float64{pl.TopZ, 0})
	pts := make([]*sketch.Point, len(outline))
	for i, o := range outline {
		pts[i] = s.CreatePoint(o[0], o[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, fmt.Errorf("support: solving pillar outline: %w", err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, fmt.Errorf("support: pillar outline gave %d profiles", len(profiles))
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := doc.Revolve(s, profiles[0], axis, decad.FullRevolution{})
	if err != nil {
		return nil, fmt.Errorf("support: revolving pillar: %w", err)
	}
	// The sketch's U (world X) becomes +Z; its V (world Y) stays Y.
	up, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(0, 0, 1), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(-1, 0, 0)}, r3.NewVec(pl.X, pl.Y, 0))
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, up)
}

// clearanceZones returns the solids that must not touch the part:
//
//   - the base pad and the shaft, each widened on every side by
//     SideClearanceMM (lateral clearance from walls);
//   - the tip, swept up by TopGapMM (the vertical gap below the surface it
//     holds). Lateral clearance is not applied within the tip's height, so
//     the narrowing tip can approach a sloped surface it holds.
func (p Params) clearanceZones(pl Pillar) []convex {
	if pl.Tree > 0 {
		return p.memberZones(pl, p.SideClearanceMM)
	}
	if pl.IsBranch() {
		return p.branchZones(pl, p.SideClearanceMM)
	}
	sc := p.SideClearanceMM
	base, shaft, contact := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
	tip := p.tipStart(pl)
	zones := []convex{frustum(pl.X, pl.Y, 0, base+sc, p.BaseThicknessMM, base+sc)}
	if tip > p.BaseThicknessMM {
		zones = append(zones, frustum(pl.X, pl.Y, p.BaseThicknessMM, shaft+sc, tip, shaft+sc))
	}
	return append(zones, sweptUp(frustum(pl.X, pl.Y, tip, shaft, pl.TopZ, contact), p.TopGapMM))
}

// Volume returns the volume in mm^3 of the round pillar Body builds for pl:
// the sum, over consecutive levels at different heights, of the cone
// frustum pi h (r1^2 + r1 r2 + r2^2) / 3.
func (p Params) Volume(pl Pillar) float64 {
	if pl.Tree > 0 {
		return 0 // counted with its tree: see PlanVolume
	}
	if pl.IsBranch() {
		return p.branchVolume(pl)
	}
	lv := p.levels(pl)
	var v float64
	for i := 1; i < len(lv); i++ {
		h := lv[i].z - lv[i-1].z
		if h <= 0 {
			continue
		}
		r1, r2 := lv[i-1].half, lv[i].half
		v += math.Pi * h * (r1*r1 + r1*r2 + r2*r2) / 3
	}
	return v
}

// PlanVolume returns the volume in mm^3 of every support in a plan: each
// straight pillar and single branch, and each tree with its members.
func (p Params) PlanVolume(plan *Plan) float64 {
	var v float64
	for _, pl := range plan.Pillars {
		v += p.Volume(pl)
	}
	for i, t := range plan.Trees {
		v += p.treeVolume(t, plan.Members(i+1))
	}
	return v
}

// ContactArea returns the area in mm^2 of a pillar's round top face.
func (p Params) ContactArea() float64 {
	r := p.ContactWidthMM / 2
	return math.Pi * r * r
}
