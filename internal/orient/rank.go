package orient

import (
	"cmp"
	"math"
	"slices"
)

// CostWeights weight the two support cost terms. Each must be finite and
// non-negative.
type CostWeights struct {
	Volume  float64 `json:"volume"`
	Contact float64 `json:"contact"`
}

// Scale holds the per-model quantities that turn costs into dimensionless
// fractions. They are the same for every orientation of one model.
type Scale struct {
	// SurfaceAreaMM2 is the model's total surface area.
	SurfaceAreaMM2 float64 `json:"surface_area_mm2"`
	// SizeMM is twice the largest distance from the model's volume
	// centroid to a vertex.
	SizeMM float64 `json:"size_mm"`
	// Elongation is the model's Principal.Elongation.
	Elongation float64 `json:"elongation"`
}

// Cost is a support cost: how much support material an orientation needs,
// and how much of the part's surface the supports touch.
type Cost struct {
	VolumeMM3  float64 `json:"volume_mm3"`
	ContactMM2 float64 `json:"contact_mm2"`
	// TooLowMM2 is the projected area of overhang that dips below the
	// shortest pillar (and above the plate-anchor height): no pillar fits
	// under it, so an orientation with any is unsupportable. Only the
	// estimate sets it.
	TooLowMM2 float64 `json:"too_low_mm2"`
	// Score is Volume weight times VolumeMM3 / (surface area * size) plus
	// Contact weight times ContactMM2 / surface area: both terms are
	// dimensionless, so no two units are added. Lower is better.
	Score float64 `json:"score"`
}

// NewCost returns the cost of a support volume and contact area.
func NewCost(volumeMM3, contactMM2 float64, w CostWeights, s Scale) Cost {
	return Cost{
		VolumeMM3:  volumeMM3,
		ContactMM2: contactMM2,
		Score:      w.Volume*volumeMM3/(s.SurfaceAreaMM2*s.SizeMM) + w.Contact*contactMM2/s.SurfaceAreaMM2,
	}
}

// ScoreQuantum is the resolution at which scores are compared. Two scores
// that round to the same multiple of it tie, so float rounding from a
// different triangle order cannot reorder candidates.
const ScoreQuantum = 1e-9

func compare(a, b Entry) int {
	q := func(v, quantum float64) float64 { return math.Round(v / quantum) }
	fa, fb := 0, 0
	if !a.Feasible {
		fa = 1
	}
	if !b.Feasible {
		fb = 1
	}
	return cmp.Or(
		cmp.Compare(fa, fb),
		cmp.Compare(q(a.Score, ScoreQuantum), q(b.Score, ScoreQuantum)),
		cmp.Compare(q(b.FirstLayerAreaMM2, tieQuantum), q(a.FirstLayerAreaMM2, tieQuantum)),
		cmp.Compare(q(a.HeightMM, tieQuantum), q(b.HeightMM, tieQuantum)),
		cmp.Compare(q(a.ElevationDeg, tieQuantum), q(b.ElevationDeg, tieQuantum)),
		cmp.Compare(a.ID, b.ID),
	)
}

// Entry is one candidate as ranking sees it.
type Entry struct {
	ID       int
	Feasible bool
	Score    float64
	// FirstLayerAreaMM2, HeightMM and ElevationDeg break ties.
	FirstLayerAreaMM2 float64
	HeightMM          float64
	ElevationDeg      float64
}

// tieQuantum is the resolution at which tied candidates' first-layer areas
// (mm^2), heights (mm) and elevations (degrees) are compared.
const tieQuantum = 1e-6

// Less reports whether a ranks before b: feasible before infeasible; then
// lower score (rounded to ScoreQuantum); then larger first-layer area,
// lower height and lower long-axis elevation (each rounded to 1e-6); then
// lower ID.
func Less(a, b Entry) bool {
	return compare(a, b) < 0
}

// Rank orders entries by Less and returns their IDs in that order.
func Rank(entries []Entry) []int {
	sorted := slices.Clone(entries)
	slices.SortFunc(sorted, compare)
	ids := make([]int, len(sorted))
	for i, e := range sorted {
		ids[i] = e.ID
	}
	return ids
}
