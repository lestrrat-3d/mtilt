package support

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/r3"
)

// OcclusionClass summarizes how a placed model's occluded demand samples
// lie (see ClassifyOcclusion).
type OcclusionClass string

// Occlusion classes.
const (
	// OcclusionClear: every occluded sample has a clear sample near it, or
	// no sample is occluded.
	OcclusionClear OcclusionClass = "clear"
	// OcclusionBlocked: some occluded sample has no clear sample near it.
	OcclusionBlocked OcclusionClass = "blocked"
	// OcclusionUnknown: the samples exceeded Limits.MaxSamples, so nothing
	// was measured.
	OcclusionUnknown OcclusionClass = "unknown"
)

// Occlusion tests demand samples of one placed model against the model
// itself. Like the index it holds, it is not safe for concurrent use.
type Occlusion struct {
	b *builder
}

// NewOcclusion indexes and classifies placed for Occluded. It returns
// ctx.Err() when ctx is cancelled.
func NewOcclusion(ctx context.Context, placed *mesh.Mesh, p Params, tol mesh.Tolerance) (*Occlusion, error) {
	b, _, err := newBuilder(ctx, placed, p, Limits{}, tol)
	if err != nil {
		return nil, err
	}
	return &Occlusion{b: b}, nil
}

// Occluded reports whether the vertical line from the plate up through
// sample s fails to reach the surface s lies on. That is the case when the
// line meets nothing, when the first triangle it meets is not demand, or
// when it first meets demand more than the coverage rule's height
// difference (see Build) away from s.Z, so a pillar there would hold a
// different surface. A sample can be occluded and still be covered by a
// pillar placed beside it.
func (o *Occlusion) Occluded(s r3.Vec) bool {
	b := o.b
	surf, tri, ok := b.ix.lowestHit(s.X, s.Y, 0, b.eps)
	if !ok || b.kinds[tri] != overhang.Demand {
		return true
	}
	return math.Abs(surf-s.Z) > b.dzMax+b.eps
}

// ClassifyOcclusion returns OcclusionBlocked when some occluded sample has
// no clear sample that a pillar at the clear sample's position would cover
// under the coverage rule (see Build), and OcclusionClear otherwise.
// occluded[i] says whether samples[i] is occluded; the slices must have the
// same length.
//
// This is a heuristic. A clear sample nearby does not prove a pillar fits
// there, and an occluded sample does not make a plan fail when a pillar
// beside it covers it. ClassifyOcclusion returns ctx.Err() when ctx is
// cancelled.
func ClassifyOcclusion(ctx context.Context, samples []DemandSample, occluded []bool, p Params, tol mesh.Tolerance) (OcclusionClass, error) {
	if len(samples) != len(occluded) {
		return "", fmt.Errorf("support: %d samples but %d occlusion flags", len(samples), len(occluded))
	}
	b := &builder{p: p, eps: tol.Length, dzMax: p.coverDZ()}
	clearCells := make(map[[2]int64][]int)
	for i, s := range samples {
		if i%ctxCheckEvery == 0 {
			if err := ctx.Err(); err != nil {
				return "", err
			}
		}
		if !occluded[i] {
			c := b.cellOf(s.Point.X, s.Point.Y)
			clearCells[c] = append(clearCells[c], i)
		}
	}
	for i, s := range samples {
		if i%ctxCheckEvery == 0 {
			if err := ctx.Err(); err != nil {
				return "", err
			}
		}
		if !occluded[i] {
			continue
		}
		if !b.nearClear(clearCells, samples, s.Point) {
			return OcclusionBlocked, nil
		}
	}
	return OcclusionClear, nil
}

// nearClear reports whether a pillar at one of the clear samples in the 3x3
// cells around s, holding that sample's surface, would cover s.
func (b *builder) nearClear(clearCells map[[2]int64][]int, samples []DemandSample, s r3.Vec) bool {
	c := b.cellOf(s.X, s.Y)
	for dy := int64(-1); dy <= 1; dy++ {
		for dx := int64(-1); dx <= 1; dx++ {
			for _, j := range clearCells[[2]int64{c[0] + dx, c[1] + dy}] {
				at := samples[j].Point
				if b.covers(Pillar{X: at.X, Y: at.Y, SurfaceZ: at.Z}, s) {
					return true
				}
			}
		}
	}
	return false
}

// OcclusionReport is the occlusion measurement of one placed model. The
// numbers are nil when Class is OcclusionUnknown.
type OcclusionReport struct {
	// Samples counts the demand samples Build would plan on.
	Samples *int `json:"samples"`
	// OccludedSamples counts the samples Occlusion.Occluded reports.
	OccludedSamples *int `json:"occluded_samples"`
	// OccludedGridAreaMM2 is the number of distinct XY positions of
	// occluded grid samples (DemandSample.Grid) times the sample pitch
	// squared.
	OccludedGridAreaMM2 *float64       `json:"occluded_grid_area_mm2"`
	Class               OcclusionClass `json:"class"`
}

// MeasureOcclusion samples placed's demand as Build does, tests every sample
// with Occlusion.Occluded, and classifies the result with ClassifyOcclusion.
// When the samples would exceed lim.MaxSamples, the report's class is
// OcclusionUnknown and its numbers are nil. MeasureOcclusion returns
// ctx.Err() when ctx is cancelled.
func MeasureOcclusion(ctx context.Context, placed *mesh.Mesh, p Params, lim Limits, tol mesh.Tolerance) (OcclusionReport, error) {
	b, demand, err := newBuilder(ctx, placed, p, lim, tol)
	if err != nil {
		return OcclusionReport{}, err
	}
	var samples []DemandSample
	if demand > 0 {
		samples, _, _, err = b.samples(ctx)
		switch {
		case errors.Is(err, ErrLimit):
			return OcclusionReport{Class: OcclusionUnknown}, nil
		case err != nil:
			return OcclusionReport{}, err
		}
	}
	occ := &Occlusion{b: b}
	occluded := make([]bool, len(samples))
	count := 0
	gridXY := make(map[[2]float64]struct{})
	for i, s := range samples {
		if i%ctxCheckEvery == 0 {
			if err := ctx.Err(); err != nil {
				return OcclusionReport{}, err
			}
		}
		if !occ.Occluded(s.Point) {
			continue
		}
		occluded[i] = true
		count++
		if s.Grid {
			gridXY[[2]float64{s.Point.X, s.Point.Y}] = struct{}{}
		}
	}
	class, err := ClassifyOcclusion(ctx, samples, occluded, p, tol)
	if err != nil {
		return OcclusionReport{}, err
	}
	q := p.SpacingMM / sampleDivisions
	n := len(samples)
	area := float64(len(gridXY)) * q * q
	return OcclusionReport{Samples: &n, OccludedSamples: &count, OccludedGridAreaMM2: &area, Class: class}, nil
}
