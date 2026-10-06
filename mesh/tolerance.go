package mesh

import "math"

// RelativeLength is the numeric length tolerance as a fraction of a model's
// size. See Tolerance.
const RelativeLength = 1e-9

// float32Ulp is the spacing of float32 values in [1, 2): 2^-23.
const float32Ulp = 1.0 / (1 << 23)

// Tolerance holds the numeric tolerances for one model. Every geometric
// comparison in mtilt that needs an epsilon takes it from here, so the
// tolerances scale with the model instead of being fixed constants.
//
// These are numeric tolerances only: they absorb floating-point rounding.
// Manufacturing clearances (the gap between a support and the part, the side
// clearance from walls) are profile values and never come from here.
type Tolerance struct {
	// Length is the distance, in millimeters, below which two positions or
	// a height are treated as equal. It is RelativeLength times the model's
	// scale (its largest absolute coordinate, or 1 mm when smaller).
	Length float64

	// Serialization is the largest coordinate change, in millimeters, that
	// writing the model as float32 (binary STL) can introduce: one float32
	// ulp at the model's largest absolute coordinate. Checks that run on a
	// re-read file allow this much difference.
	Serialization float64

	// Plate is the height, in millimeters, at or below which a vertex of a
	// placed model counts as lying on the build plate: four float32 ulps at
	// the model's scale. Binary STL stores float32, so a face that is flat
	// in the source CAD model arrives with vertex heights that differ by a
	// few ulps after rotation; Length alone would split it.
	Plate float64
}

// ToleranceFor returns the tolerances for geometry inside b.
func ToleranceFor(b Box) Tolerance {
	scale := 1.0
	for _, c := range []float64{b.Min.X, b.Min.Y, b.Min.Z, b.Max.X, b.Max.Y, b.Max.Z} {
		scale = math.Max(scale, math.Abs(c))
	}
	return Tolerance{
		Length:        RelativeLength * scale,
		Serialization: float32Ulp * scale,
		Plate:         4 * float32Ulp * scale,
	}
}
