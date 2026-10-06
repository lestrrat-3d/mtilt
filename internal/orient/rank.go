package orient

import (
	"cmp"
	"math"
	"slices"
)

// Weights are the objective's term weights. Each must be finite and
// non-negative.
type Weights struct {
	SupportDemand float64 `json:"support_demand"`
	Height        float64 `json:"height"`
	BedContact    float64 `json:"bed_contact"`
	Strength      float64 `json:"strength"`
}

// Scale holds the per-model quantities that turn metrics into dimensionless
// fractions. Both are the same for every orientation of one model.
type Scale struct {
	// SurfaceAreaMM2 is the model's total surface area.
	SurfaceAreaMM2 float64 `json:"surface_area_mm2"`
	// SizeMM is twice the largest distance from the model's volume
	// centroid to a vertex.
	SizeMM float64 `json:"size_mm"`
	// Elongation is the model's Principal.Elongation.
	Elongation float64 `json:"elongation"`
}

// Terms are the weighted, dimensionless objective terms of one candidate.
// Their sum is the score; lower is better.
type Terms struct {
	// SupportDemand is Weights.SupportDemand times the projected support
	// demand area over the surface area.
	SupportDemand float64 `json:"support_demand"`
	// Height is Weights.Height times the build height over the model size.
	Height float64 `json:"height"`
	// BedContact is minus Weights.BedContact times the bed-contact area
	// over the surface area, so more contact lowers the score.
	BedContact float64 `json:"bed_contact"`
	// Strength is Weights.Strength times the elongation times the squared
	// sine of the long axis's elevation: 0 when the long axis lies flat or
	// the model has no long axis, largest when a slender model stands up.
	// FDM parts are weakest across layer lines, and a long part standing up
	// puts every layer line across its length.
	Strength float64 `json:"strength"`
}

// Score returns the weighted terms of m and their sum.
func Score(m Metrics, w Weights, s Scale) (Terms, float64) {
	t := Terms{
		SupportDemand: w.SupportDemand * m.SupportDemandProjectedAreaMM2 / s.SurfaceAreaMM2,
		Height:        w.Height * m.HeightMM / s.SizeMM,
		BedContact:    -w.BedContact * m.BedContactAreaMM2 / s.SurfaceAreaMM2,
	}
	sin := math.Sin(m.LongAxisElevationDeg * math.Pi / 180)
	t.Strength = w.Strength * s.Elongation * sin * sin
	return t, t.SupportDemand + t.Height + t.BedContact + t.Strength
}

// ScoreQuantum is the resolution at which scores are compared. Two scores
// that round to the same multiple of it tie, so float rounding from a
// different triangle order cannot reorder candidates.
const ScoreQuantum = 1e-9

// Entry is one candidate as ranking sees it.
type Entry struct {
	ID       int
	Feasible bool
	Score    float64
}

// Rank orders entries: feasible before infeasible, then by score rounded to
// ScoreQuantum (ascending), then by ID (ascending). It returns the IDs in
// rank order.
func Rank(entries []Entry) []int {
	sorted := slices.Clone(entries)
	q := func(s float64) float64 { return math.Round(s / ScoreQuantum) }
	slices.SortFunc(sorted, func(a, b Entry) int {
		fa, fb := 0, 0
		if !a.Feasible {
			fa = 1
		}
		if !b.Feasible {
			fb = 1
		}
		return cmp.Or(cmp.Compare(fa, fb), cmp.Compare(q(a.Score), q(b.Score)), cmp.Compare(a.ID, b.ID))
	})
	ids := make([]int, len(sorted))
	for i, e := range sorted {
		ids[i] = e.ID
	}
	return ids
}
