// Package overhang classifies triangles of a placed model (build plate at
// Z = 0, build direction +Z) by whether they need support.
//
// The overhang angle of a downward-facing triangle is the angle between its
// plane and the horizontal plane: a downward horizontal ceiling is 0 degrees
// and a vertical wall is 90 degrees. A downward-facing triangle whose angle
// is below the threshold needs support, unless it lies on the build plate.
// Upward-facing and vertical triangles never need support.
package overhang

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Kind is the support class of one triangle.
type Kind int

// The support classes.
const (
	// None needs no support: upward-facing, vertical, or steeper than the
	// threshold.
	None Kind = iota
	// BedContact is downward-facing with every vertex on the build plate.
	BedContact
	// Demand is downward-facing, below the threshold, and not on the build
	// plate: it needs support.
	Demand
)

// Angle returns the overhang angle in degrees of a triangle with unit normal
// n: acos(-n.Z) for a downward-facing normal. It returns 90 for a normal
// with n.Z >= 0, which never needs support.
func Angle(n r3.Vec) float64 {
	if n.Z >= 0 {
		return 90
	}
	return math.Acos(math.Min(1, -n.Z)) * 180 / math.Pi
}

// Classify returns the support class of triangle t. thresholdDeg is the
// overhang threshold in degrees; bedTol is the height in millimeters at or
// below which a vertex counts as on the build plate. A triangle with no
// normal (zero area) is None.
func Classify(t [3]r3.Vec, thresholdDeg, bedTol float64) Kind {
	n, ok := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
	if !ok || n.Z >= 0 {
		return None
	}
	if t[0].Z <= bedTol && t[1].Z <= bedTol && t[2].Z <= bedTol {
		return BedContact
	}
	if Angle(n) < thresholdDeg {
		return Demand
	}
	return None
}
