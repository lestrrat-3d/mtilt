package orient

import (
	"cmp"
	"fmt"
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

// quantize rounds v to a multiple of quantum, in units of quantum.
func quantize(v, quantum float64) float64 { return math.Round(v / quantum) }

// rankKey is one ranking key: its name in selection notes and its order.
type rankKey struct {
	name    string
	compare func(a, b Entry) int
}

// rankKeys are the ranking keys in order of precedence.
var rankKeys = []rankKey{
	{"feasible", func(a, b Entry) int {
		fa, fb := 0, 0
		if !a.Feasible {
			fa = 1
		}
		if !b.Feasible {
			fb = 1
		}
		return cmp.Compare(fa, fb)
	}},
	{"score", func(a, b Entry) int {
		return cmp.Compare(quantize(a.Score, ScoreQuantum), quantize(b.Score, ScoreQuantum))
	}},
	{"first_layer", func(a, b Entry) int {
		return cmp.Compare(quantize(b.FirstLayerAreaMM2, tieQuantum), quantize(a.FirstLayerAreaMM2, tieQuantum))
	}},
	{"height", func(a, b Entry) int {
		return cmp.Compare(quantize(a.HeightMM, tieQuantum), quantize(b.HeightMM, tieQuantum))
	}},
	{"elevation", func(a, b Entry) int {
		return cmp.Compare(quantize(a.ElevationDeg, tieQuantum), quantize(b.ElevationDeg, tieQuantum))
	}},
	{"id", func(a, b Entry) int { return cmp.Compare(a.ID, b.ID) }},
}

// compareBy orders a and b by keys, and returns the name of the first key
// that tells them apart ("" when none does).
func compareBy(keys []rankKey, a, b Entry) (int, string) {
	for _, k := range keys {
		if c := k.compare(a, b); c != 0 {
			return c, k.name
		}
	}
	return 0, ""
}

func compare(a, b Entry) int {
	c, _ := compareBy(rankKeys, a, b)
	return c
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

// Rank orders entries and returns their IDs in that order: feasible before
// infeasible; then lower score (rounded to ScoreQuantum); then larger
// first-layer area, lower height and lower long-axis elevation (each
// rounded to 1e-6); then lower ID.
func Rank(entries []Entry) []int {
	sorted := slices.Clone(entries)
	slices.SortFunc(sorted, compare)
	ids := make([]int, len(sorted))
	for i, e := range sorted {
		ids[i] = e.ID
	}
	return ids
}

// FirstLayerShare is the fraction of the band's largest first-layer area a
// band member must keep to stay in the selection. Like the default support
// band, it is not calibrated by any print.
const FirstLayerShare = 0.75

// Select picks the winner among planned entries, whose scores are planned
// costs (non-negative, since the cost weights are). band must be at least 0.
//
// With S* the lowest score, the support band holds the entries whose score,
// rounded to ScoreQuantum, is at most S* * (1 + band) rounded the same way.
// Anchored on S*, membership does not depend on the other entries. Among the
// band, with A* the largest first-layer area, an entry whose area rounded
// to 1e-6 is under FirstLayerShare * A* rounded the same way drops out; the
// entry with A* always stays, so the set is never empty. The rest are ranked
// like Rank without its feasibility key, and the first wins. Band 0 keeps
// only the entries whose score ties S* at ScoreQuantum.
//
// notes holds one note per entry ID: "selected", "outside the support band:
// score %.6g > %.6g", "first layer %.1f mm2 under 75% of %.1f mm2", or
// "lost to candidate %d on %s", where %s is the first key that ranks the
// winner ahead (score, first_layer, height, elevation or id). With no
// entries, Select returns -1 and no notes.
func Select(entries []Entry, band float64) (int, map[int]string) {
	notes := make(map[int]string, len(entries))
	if len(entries) == 0 {
		return -1, notes
	}
	best := entries[0].Score
	for _, e := range entries[1:] {
		best = math.Min(best, e.Score)
	}
	limit := best * (1 + band)
	members := make([]Entry, 0, len(entries))
	for _, e := range entries {
		if quantize(e.Score, ScoreQuantum) > quantize(limit, ScoreQuantum) {
			notes[e.ID] = fmt.Sprintf("outside the support band: score %.6g > %.6g", e.Score, limit)
			continue
		}
		members = append(members, e)
	}

	var largest float64
	for _, e := range members {
		largest = math.Max(largest, e.FirstLayerAreaMM2)
	}
	cut := FirstLayerShare * largest
	kept := make([]Entry, 0, len(members))
	for _, e := range members {
		if quantize(e.FirstLayerAreaMM2, tieQuantum) < quantize(cut, tieQuantum) {
			notes[e.ID] = fmt.Sprintf("first layer %.1f mm2 under %.0f%% of %.1f mm2", e.FirstLayerAreaMM2, 100*FirstLayerShare, largest)
			continue
		}
		kept = append(kept, e)
	}

	keys := rankKeys[1:] // every entry is planned, so feasible
	slices.SortFunc(kept, func(a, b Entry) int {
		c, _ := compareBy(keys, a, b)
		return c
	})
	win := kept[0]
	notes[win.ID] = "selected"
	for _, e := range kept[1:] {
		_, key := compareBy(keys, win, e)
		notes[e.ID] = fmt.Sprintf("lost to candidate %d on %s", win.ID, key)
	}
	return win.ID, notes
}
