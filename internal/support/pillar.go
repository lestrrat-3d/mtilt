// Package support builds and checks mtilt's breakaway supports: separate
// vertical pillars rooted on the build plate, each a closed mesh of stacked
// axis-aligned square sections.
//
// A pillar, from the plate up:
//
//   - a base pad BaseWidthMM square and BaseThicknessMM tall;
//   - a shaft PillarWidthMM square, up to TipHeightMM below the top;
//   - a tip that narrows from PillarWidthMM to ContactWidthMM over
//     TipHeightMM;
//   - a ContactWidthMM square top face, TopGapMM (plus a numeric slack)
//     below the lowest point of the part above that square.
//
// The pillar never touches the part: the top gap is left for the slicer to
// bridge or not, as its own settings decide.
package support

import (
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
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
}

// MinHeight is the shortest pillar these params can build: base plus tip.
func (p Params) MinHeight() float64 { return p.BaseThicknessMM + p.TipHeightMM }

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
}

type level struct {
	z, half float64
}

// levels returns the pillar's square sections from the plate up. Two
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

// Mesh returns the pillar as a closed, outward-wound triangle mesh.
func (p Params) Mesh(pl Pillar) *mesh.Mesh {
	lv := p.levels(pl)
	m := &mesh.Mesh{}
	for _, l := range lv {
		h := l.half
		m.Vertices = append(m.Vertices,
			r3.NewVec(pl.X-h, pl.Y-h, l.z), r3.NewVec(pl.X+h, pl.Y-h, l.z),
			r3.NewVec(pl.X+h, pl.Y+h, l.z), r3.NewVec(pl.X-h, pl.Y+h, l.z),
		)
	}
	// Bottom cap, facing -Z.
	m.Triangles = append(m.Triangles, [3]uint32{0, 2, 1}, [3]uint32{0, 3, 2})
	for i := range len(lv) - 1 {
		lo, hi := uint32(4*i), uint32(4*(i+1))
		for k := range uint32(4) {
			a, b := lo+k, lo+(k+1)%4
			c, d := hi+(k+1)%4, hi+k
			m.Triangles = append(m.Triangles, [3]uint32{a, b, c}, [3]uint32{a, c, d})
		}
	}
	// Top cap, facing +Z.
	top := uint32(4 * (len(lv) - 1))
	m.Triangles = append(m.Triangles, [3]uint32{top, top + 1, top + 2}, [3]uint32{top, top + 2, top + 3})
	return m
}

// clearanceZones returns the solids that must not touch the part:
//
//   - the base pad and the shaft, each widened on every side by
//     SideClearanceMM (lateral clearance from walls);
//   - the tip, swept up by TopGapMM (the vertical gap below the surface it
//     holds). Lateral clearance is not applied within the tip's height, so
//     the narrowing tip can approach a sloped surface it holds.
func (p Params) clearanceZones(pl Pillar) []convex {
	sc := p.SideClearanceMM
	base, shaft, contact := p.BaseWidthMM/2, p.PillarWidthMM/2, p.ContactWidthMM/2
	tip := p.tipStart(pl)
	zones := []convex{frustum(pl.X, pl.Y, 0, base+sc, p.BaseThicknessMM, base+sc)}
	if tip > p.BaseThicknessMM {
		zones = append(zones, frustum(pl.X, pl.Y, p.BaseThicknessMM, shaft+sc, tip, shaft+sc))
	}
	return append(zones, sweptUp(frustum(pl.X, pl.Y, tip, shaft, pl.TopZ, contact), p.TopGapMM))
}
