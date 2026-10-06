// Package mesh holds mtilt's indexed triangle mesh, the geometry measurements
// taken from it, and the checks that decide whether a mesh is a supported
// input.
//
// Coordinates are float64 millimeters in a right-handed frame once a mesh has
// gone through unit conversion; the package itself attaches no unit to a
// coordinate. A triangle's normal is always computed from its winding (the
// right-hand rule over its vertex order). Normals stored in an input file are
// never read.
package mesh

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// Mesh is an indexed triangle mesh. Vertices holds each distinct position
// once; every Triangles entry holds three indices into Vertices, in winding
// order.
//
// A Mesh is a plain value: the functions in this package never modify one in
// place, and return a new Mesh instead.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]uint32
}

// ErrNonFinite is returned when a coordinate is NaN or infinite.
var ErrNonFinite = errors.New("mesh: non-finite coordinate")

// FromSoup builds an indexed mesh from a triangle soup: a list of triangles
// that each carry their own three vertex positions.
//
// Two positions merge into one vertex only when all three float64 coordinates
// are exactly equal (+0 and -0 compare equal). FromSoup never welds
// positions that merely lie close together, so a mesh whose triangles meet at
// near-but-unequal positions comes out with open edges, and Validate reports
// them.
//
// FromSoup returns ErrNonFinite when any coordinate is NaN or infinite.
func FromSoup(soup [][3]r3.Vec) (*Mesh, error) {
	index := make(map[r3.Vec]uint32, len(soup))
	m := &Mesh{
		Vertices:  make([]r3.Vec, 0, len(soup)/2+3),
		Triangles: make([][3]uint32, 0, len(soup)),
	}
	for ti, tri := range soup {
		var idx [3]uint32
		for k, p := range tri {
			if !finite(p) {
				return nil, fmt.Errorf("%w: triangle %d vertex %d is %v", ErrNonFinite, ti, k, p)
			}
			p = canonicalZero(p)
			i, ok := index[p]
			if !ok {
				i = uint32(len(m.Vertices))
				index[p] = i
				m.Vertices = append(m.Vertices, p)
			}
			idx[k] = i
		}
		m.Triangles = append(m.Triangles, idx)
	}
	return m, nil
}

// Soup returns the mesh as a triangle soup, in triangle order.
func (m *Mesh) Soup() [][3]r3.Vec {
	out := make([][3]r3.Vec, len(m.Triangles))
	for i := range m.Triangles {
		out[i] = m.Triangle(i)
	}
	return out
}

// Triangle returns the three vertex positions of triangle i in winding order.
func (m *Mesh) Triangle(i int) [3]r3.Vec {
	t := m.Triangles[i]
	return [3]r3.Vec{m.Vertices[t[0]], m.Vertices[t[1]], m.Vertices[t[2]]}
}

// Transformed returns a copy of m with every vertex mapped through t.
// Triangle indices and winding are copied unchanged, so a reflection in t
// would turn the mesh inside out; callers that need a proper rigid motion
// check t.IsReflection first.
func (m *Mesh) Transformed(t r3.Transform) *Mesh {
	out := &Mesh{
		Vertices:  make([]r3.Vec, len(m.Vertices)),
		Triangles: append([][3]uint32(nil), m.Triangles...),
	}
	for i, v := range m.Vertices {
		out.Vertices[i] = t.Apply(v)
	}
	return out
}

// Scaled returns a copy of m with every coordinate multiplied by k. It is the
// one place mtilt changes the size of a model, and it is only used to convert
// input units to millimeters. k must be positive and finite.
func (m *Mesh) Scaled(k float64) *Mesh {
	out := &Mesh{
		Vertices:  make([]r3.Vec, len(m.Vertices)),
		Triangles: append([][3]uint32(nil), m.Triangles...),
	}
	for i, v := range m.Vertices {
		out.Vertices[i] = v.Scale(k)
	}
	return out
}

// Box is an axis-aligned bounding box.
type Box struct {
	Min, Max r3.Vec
}

// Size returns the box's extent along each axis.
func (b Box) Size() r3.Vec { return b.Max.Sub(b.Min) }

// Diagonal returns the length of the box's diagonal.
func (b Box) Diagonal() float64 { return b.Size().Len() }

// Union returns the smallest box that holds both b and o.
func (b Box) Union(o Box) Box {
	return Box{
		Min: r3.NewVec(math.Min(b.Min.X, o.Min.X), math.Min(b.Min.Y, o.Min.Y), math.Min(b.Min.Z, o.Min.Z)),
		Max: r3.NewVec(math.Max(b.Max.X, o.Max.X), math.Max(b.Max.Y, o.Max.Y), math.Max(b.Max.Z, o.Max.Z)),
	}
}

// Bounds returns the axis-aligned bounding box of the mesh's vertices. The
// zero Box is returned for a mesh without vertices.
func (m *Mesh) Bounds() Box {
	if len(m.Vertices) == 0 {
		return Box{}
	}
	b := Box{Min: m.Vertices[0], Max: m.Vertices[0]}
	for _, v := range m.Vertices[1:] {
		b.Min = r3.NewVec(math.Min(b.Min.X, v.X), math.Min(b.Min.Y, v.Y), math.Min(b.Min.Z, v.Z))
		b.Max = r3.NewVec(math.Max(b.Max.X, v.X), math.Max(b.Max.Y, v.Y), math.Max(b.Max.Z, v.Z))
	}
	return b
}

// SurfaceArea returns the sum of the triangle areas.
func (m *Mesh) SurfaceArea() float64 {
	var sum float64
	for i := range m.Triangles {
		sum += TriangleArea(m.Triangle(i))
	}
	return sum
}

// Volume returns the signed volume enclosed by the mesh, computed with the
// divergence theorem. It is positive for a closed mesh whose normals point
// outward, and meaningless for an open one.
func (m *Mesh) Volume() float64 {
	var sum float64
	for i := range m.Triangles {
		t := m.Triangle(i)
		sum += t[0].Dot(t[1].Cross(t[2]))
	}
	return sum / 6
}

// Centroid returns the center of the solid volume the mesh encloses. It
// assumes a closed, outward-wound mesh with nonzero volume.
func (m *Mesh) Centroid() r3.Vec {
	var acc r3.Vec
	var vol float64
	for i := range m.Triangles {
		t := m.Triangle(i)
		v := t[0].Dot(t[1].Cross(t[2]))
		vol += v
		acc = acc.Add(t[0].Add(t[1]).Add(t[2]).Scale(v))
	}
	return acc.Scale(1 / (4 * vol))
}

// TriangleArea returns the area of t.
func TriangleArea(t [3]r3.Vec) float64 {
	return t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Len() / 2
}

// TriangleNormal returns the unit normal of t from its winding, and false
// when t has no direction (zero area).
func TriangleNormal(t [3]r3.Vec) (r3.Vec, bool) {
	return t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
}

func finite(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}

// canonicalZero turns -0 into +0 so that the vertex map, which compares keys
// with ==, and the stored position agree on one spelling.
func canonicalZero(v r3.Vec) r3.Vec {
	if v.X == 0 {
		v.X = 0
	}
	if v.Y == 0 {
		v.Y = 0
	}
	if v.Z == 0 {
		v.Z = 0
	}
	return v
}
