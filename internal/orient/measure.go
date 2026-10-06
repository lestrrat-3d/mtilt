package orient

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/overhang"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
)

// Place rotates m by rot and then translates it onto the build plate. The
// translation is
//
//	t = (-(minX+maxX)/2, -(minY+maxY)/2, -minZ)
//
// computed from the rotated model's bounding box, so the model's lowest
// point sits at Z = 0 and its bounding box is centered on the Z axis. Place
// returns the placed mesh and the combined transform, rotation first.
func Place(m *mesh.Mesh, rot r3.Transform) (*mesh.Mesh, r3.Transform, error) {
	b := m.Transformed(rot).Bounds()
	shift, err := r3.Translation(r3.NewVec(-(b.Min.X+b.Max.X)/2, -(b.Min.Y+b.Max.Y)/2, -b.Min.Z))
	if err != nil {
		return nil, r3.Transform{}, err
	}
	full, err := rot.Then(shift)
	if err != nil {
		return nil, r3.Transform{}, err
	}
	placed := m.Transformed(full)
	// Rounding in the combined transform can leave the lowest vertex a few
	// ulps off the plate; the plate is defined as Z = 0, so snap it.
	minZ := placed.Bounds().Min.Z
	if minZ != 0 {
		fix, err := r3.Translation(r3.NewVec(0, 0, -minZ))
		if err != nil {
			return nil, r3.Transform{}, err
		}
		if full, err = full.Then(fix); err != nil {
			return nil, r3.Transform{}, err
		}
		placed = m.Transformed(full)
	}
	return placed, full, nil
}

// Metrics are the measurements of one placed orientation. Lengths are in
// millimeters and areas in square millimeters.
type Metrics struct {
	// HeightMM is the placed model's maximum Z.
	HeightMM float64 `json:"height_mm"`
	// FootprintMM is the X and Y extent of the placed model's bounding box.
	FootprintMM [2]float64 `json:"bounding_footprint_mm"`
	// SupportDemandAreaMM2 is the surface area of the triangles that need
	// support (overhang.Demand).
	SupportDemandAreaMM2 float64 `json:"support_demand_area_mm2"`
	// SupportDemandProjectedAreaMM2 is the same triangles' area projected
	// onto the build plate.
	SupportDemandProjectedAreaMM2 float64 `json:"support_demand_projected_area_mm2"`
	// BedContactAreaMM2 is the area of the triangles lying on the build
	// plate (overhang.BedContact). It is the model's actual contact
	// geometry, not its bounding-box footprint.
	BedContactAreaMM2 float64 `json:"bed_contact_area_mm2"`
	// CentroidOverContact reports whether the volume centroid, projected
	// onto the plate, falls inside the convex hull of the bed-contact
	// vertices. It is nil when the contact vertices span no area. This is
	// a geometric heuristic, not a stability simulation.
	CentroidOverContact *bool `json:"centroid_over_contact_hull"`
	// ContactHullMarginMM is the distance from the projected centroid to
	// the contact hull's edge: positive inside, negative outside. Nil with
	// CentroidOverContact.
	ContactHullMarginMM *float64 `json:"contact_hull_margin_mm"`
}

// Measure computes the metrics of a placed model. thresholdDeg is the
// overhang threshold and tol the model's numeric tolerances.
func Measure(placed *mesh.Mesh, thresholdDeg float64, tol mesh.Tolerance) Metrics {
	b := placed.Bounds()
	met := Metrics{
		HeightMM:    b.Max.Z,
		FootprintMM: [2]float64{b.Max.X - b.Min.X, b.Max.Y - b.Min.Y},
	}
	var contact []r3.Vec
	for i := range placed.Triangles {
		t := placed.Triangle(i)
		switch overhang.Classify(t, thresholdDeg, tol.Plate) {
		case overhang.Demand:
			met.SupportDemandAreaMM2 += mesh.TriangleArea(t)
			met.SupportDemandProjectedAreaMM2 += projectedArea(t)
		case overhang.BedContact:
			met.BedContactAreaMM2 += mesh.TriangleArea(t)
			contact = append(contact, t[0], t[1], t[2])
		case overhang.None:
		}
	}
	hull := convexHull(contact)
	if len(hull) >= 3 && polygonArea(hull) > tol.Length*tol.Length {
		c := placed.Centroid()
		margin := hullMargin(hull, c)
		inside := margin >= 0
		met.CentroidOverContact = &inside
		met.ContactHullMarginMM = &margin
	}
	return met
}

func projectedArea(t [3]r3.Vec) float64 {
	return math.Abs((t[1].X-t[0].X)*(t[2].Y-t[0].Y)-(t[1].Y-t[0].Y)*(t[2].X-t[0].X)) / 2
}

// convexHull returns the XY convex hull of pts, counter-clockwise, by
// Andrew's monotone chain.
func convexHull(pts []r3.Vec) []r3.Vec {
	if len(pts) < 3 {
		return nil
	}
	p := slices.Clone(pts)
	slices.SortFunc(p, func(a, b r3.Vec) int { return cmp.Or(cmp.Compare(a.X, b.X), cmp.Compare(a.Y, b.Y)) })
	p = slices.CompactFunc(p, func(a, b r3.Vec) bool { return a.X == b.X && a.Y == b.Y })
	if len(p) < 3 {
		return nil
	}
	cross := func(o, a, b r3.Vec) float64 { return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X) }
	hull := make([]r3.Vec, 0, 2*len(p))
	for _, q := range p {
		for len(hull) >= 2 && cross(hull[len(hull)-2], hull[len(hull)-1], q) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, q)
	}
	lower := len(hull) + 1
	for i := len(p) - 2; i >= 0; i-- {
		q := p[i]
		for len(hull) >= lower && cross(hull[len(hull)-2], hull[len(hull)-1], q) <= 0 {
			hull = hull[:len(hull)-1]
		}
		hull = append(hull, q)
	}
	return hull[:len(hull)-1]
}

func polygonArea(poly []r3.Vec) float64 {
	var a float64
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		a += p.X*q.Y - q.X*p.Y
	}
	return a / 2
}

// hullMargin returns the signed XY distance from c to the boundary of the
// counter-clockwise convex polygon hull: positive inside.
func hullMargin(hull []r3.Vec, c r3.Vec) float64 {
	margin := math.Inf(1)
	for i := range hull {
		p, q := hull[i], hull[(i+1)%len(hull)]
		ex, ey := q.X-p.X, q.Y-p.Y
		l := math.Hypot(ex, ey)
		if l == 0 {
			continue
		}
		// Distance to the edge's line, positive on the inner (left) side.
		margin = math.Min(margin, (ex*(c.Y-p.Y)-ey*(c.X-p.X))/l)
	}
	return margin
}
