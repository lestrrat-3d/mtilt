package support

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/r3"
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
// difference (see Build) away from s.Z, so a straight pillar there would
// hold a different surface. An occluded sample can still be covered by a
// straight pillar placed beside it or by a branch.
func (o *Occlusion) Occluded(s r3.Vec) bool {
	b := o.b
	surf, tri, ok := b.ix.lowestHit(s.X, s.Y, 0, b.eps)
	if !ok || b.kinds[tri] != overhang.Demand {
		return true
	}
	return math.Abs(surf-s.Z) > b.dzMax+b.eps
}

// OcclusionReport is the occlusion measurement of one placed model. Every
// number is nil when the samples exceeded Limits.MaxSamples, so nothing was
// measured.
type OcclusionReport struct {
	// Samples counts the demand samples Build would plan on.
	Samples *int `json:"samples"`
	// OccludedSamples counts the samples Occlusion.Occluded reports: those
	// a straight pillar from the plate below them cannot reach.
	OccludedSamples *int `json:"occluded_samples"`
	// OccludedGridAreaMM2 is the number of distinct XY positions of
	// occluded grid samples (DemandSample.Grid) times the sample pitch
	// squared.
	OccludedGridAreaMM2 *float64 `json:"occluded_grid_area_mm2"`
}

// MeasureOcclusion samples placed's demand as Build does and tests every
// sample with Occlusion.Occluded. When the samples would exceed
// lim.MaxSamples, every number in the report is nil. MeasureOcclusion
// returns ctx.Err() when ctx is cancelled.
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
			return OcclusionReport{}, nil
		case err != nil:
			return OcclusionReport{}, err
		}
	}
	occ := &Occlusion{b: b}
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
		count++
		if s.Grid {
			gridXY[[2]float64{s.Point.X, s.Point.Y}] = struct{}{}
		}
	}
	q := p.SpacingMM / sampleDivisions
	n := len(samples)
	area := float64(len(gridXY)) * q * q
	return OcclusionReport{Samples: &n, OccludedSamples: &count, OccludedGridAreaMM2: &area}, nil
}
