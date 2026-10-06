// Package orient generates candidate orientations for a model, measures each
// one, and ranks them. Generation, measurement and ranking are separate
// steps so a different search strategy can replace generation without
// touching the others.
package orient

import (
	"cmp"
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
)

// Source records how a candidate rotation was produced.
type Source string

// Candidate sources, in generation order.
const (
	SourceOriginal    Source = "original"
	SourceAxisAligned Source = "axis_aligned"
	SourcePlanarFace  Source = "planar_face"
	SourceLongAxis    Source = "long_axis"
	SourceTiltedLong  Source = "long_axis_tilted"
)

// MinElongation is the Principal.Elongation below which a body has no long
// axis worth laying down, and Generate adds no long-axis candidates.
const MinElongation = 0.1

// TiltsDeg are the angles, in degrees, by which each lay-flat long-axis
// candidate is also tilted up, in both directions.
var TiltsDeg = []float64{5, 10, 20, 30}

// PlanarClusterDeg is the angle in degrees within which two triangle normals
// join the same planar-face cluster.
const PlanarClusterDeg = 1.0

// maxClusters caps how many planar-face clusters generation tracks. See
// planarClusters.
const maxClusters = 1024

// rotationTol is the per-component tolerance under which two rotations count
// as the same candidate.
const rotationTol = 1e-9

// Candidate is one rotation to evaluate. Rotation turns the model about the
// origin; placement on the build plate happens afterwards (see Place).
type Candidate struct {
	ID       int
	Source   Source
	Rotation r3.Transform
	// FaceNormal is the input-frame unit normal that a planar-face
	// candidate turns to -Z, and FaceAreaMM2 the total area of the
	// triangles in that cluster. Both are zero for other sources.
	FaceNormal  r3.Vec
	FaceAreaMM2 float64
	// TiltDeg is the tilt of a long_axis_tilted candidate; 0 otherwise.
	TiltDeg float64
}

// Limits bound candidate generation.
type Limits struct {
	// MaxCandidates is the most candidates Generate returns.
	MaxCandidates int
	// MaxPlanarFaces is the most planar-face candidates Generate adds.
	MaxPlanarFaces int
}

// Generation reports what Generate produced.
type Generation struct {
	// Distinct is the number of distinct rotations generated before
	// MaxCandidates applied.
	Distinct int
	// Truncated is true when MaxCandidates dropped some of them.
	Truncated bool
}

// Generate returns the candidate rotations for m in a fixed order:
//
//  1. the original orientation (identity);
//  2. the 24 proper rotations that map coordinate axes onto coordinate axes,
//     in the order axisRotations lists them;
//  3. up to lim.MaxPlanarFaces rotations that turn a dominant planar
//     direction to -Z (face down on the plate), largest cluster area first;
//  4. when pr is not nil and pr.Elongation is at least MinElongation, the
//     four rotations that lay the long axis pr.Axes[0] along +X, rolled by
//     0, 90, 180 and 270 degrees about it, and each of those tilted about
//     the Y axis by every angle in TiltsDeg, positive then negative, so the
//     long axis rises that far from the plate.
//
// A rotation equal to an earlier one within rotationTol is dropped, so IDs
// are dense and the first producer of a rotation keeps it. No yaw sampling
// is added: the axis-aligned set already contains the quarter-turn yaws of
// each axis-aligned pose, and planar-face candidates keep the yaw the
// minimal rotation gives them.
func Generate(ctx context.Context, m *mesh.Mesh, lim Limits, pr *Principal) ([]Candidate, Generation, error) {
	var out []Candidate
	add := func(c Candidate) {
		for _, prev := range out {
			if prev.Rotation.Equal(c.Rotation, rotationTol) {
				return
			}
		}
		c.ID = len(out)
		out = append(out, c)
	}

	add(Candidate{Source: SourceOriginal, Rotation: r3.Identity()})
	for _, r := range axisRotations() {
		add(Candidate{Source: SourceAxisAligned, Rotation: r})
	}

	clusters, err := planarClusters(ctx, m)
	if err != nil {
		return nil, Generation{}, err
	}
	for i, c := range clusters {
		if i == lim.MaxPlanarFaces {
			break
		}
		add(Candidate{Source: SourcePlanarFace, Rotation: faceDown(c.normal), FaceNormal: c.normal, FaceAreaMM2: c.area})
	}
	if pr != nil && pr.Elongation >= MinElongation {
		for _, flat := range layFlat(*pr) {
			add(Candidate{Source: SourceLongAxis, Rotation: flat, TiltDeg: 0})
		}
		for _, flat := range layFlat(*pr) {
			for _, deg := range TiltsDeg {
				for _, sign := range []float64{1, -1} {
					add(Candidate{Source: SourceTiltedLong, Rotation: tilted(flat, sign*deg), TiltDeg: sign * deg})
				}
			}
		}
	}

	gen := Generation{Distinct: len(out)}
	if len(out) > lim.MaxCandidates {
		out = out[:lim.MaxCandidates]
		gen.Truncated = true
	}
	return out, gen, nil
}

// axisRotations returns the 24 rotations whose basis vectors are signed
// coordinate axes and whose determinant is +1. They are listed by axis
// permutation (XYZ, XZY, YXZ, YZX, ZXY, ZYX) and then by sign pattern, with
// the identity first.
func axisRotations() []r3.Transform {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	perms := [][3]int{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}
	var out []r3.Transform
	for _, p := range perms {
		for signs := range 8 {
			s := func(bit int) float64 {
				if signs&(1<<bit) != 0 {
					return -1
				}
				return 1
			}
			b := r3.Basis{EX: axes[p[0]].Scale(s(0)), EY: axes[p[1]].Scale(s(1)), EZ: axes[p[2]].Scale(s(2))}
			if b.EX.Cross(b.EY).Dot(b.EZ) < 0 {
				continue
			}
			t, err := r3.FromBasis(b, r3.Vec{})
			if err != nil {
				panic(err) // signed axes are orthonormal
			}
			out = append(out, t)
		}
	}
	return out
}

// faceDown returns the minimal rotation that turns unit direction n to -Z.
// When n already points along +Z the rotation is a half turn about X.
func faceDown(n r3.Vec) r3.Transform {
	down := r3.NewVec(0, 0, -1)
	c := n.Dot(down)
	v := n.Cross(down)
	s2 := v.Dot(v)
	var b r3.Basis
	switch {
	case s2 < 1e-24 && c > 0:
		return r3.Identity()
	case s2 < 1e-24:
		b = r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, -1, 0), EZ: r3.NewVec(0, 0, -1)}
	default:
		// Rodrigues: R = I + [v]x + [v]x^2 (1 - c) / |v|^2. The basis
		// vectors are R's columns.
		k := (1 - c) / s2
		col := func(e r3.Vec) r3.Vec {
			vx := v.Cross(e)
			return e.Add(vx).Add(v.Cross(vx).Scale(k))
		}
		b = r3.Basis{EX: col(r3.NewVec(1, 0, 0)), EY: col(r3.NewVec(0, 1, 0)), EZ: col(r3.NewVec(0, 0, 1))}
	}
	t, err := r3.FromBasis(b, r3.Vec{})
	if err != nil {
		panic(err) // a rotation matrix is orthonormal to rounding
	}
	return t
}

// layFlat returns the four rotations that map the long axis pr.Axes[0] to
// +X, with the other two principal axes mapped to (Y, Z), (Z, -Y), (-Y, -Z)
// and (-Z, Y).
func layFlat(pr Principal) []r3.Transform {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	targets := [][2]r3.Vec{{y, z}, {z, y.Scale(-1)}, {y.Scale(-1), z.Scale(-1)}, {z.Scale(-1), y}}
	out := make([]r3.Transform, 0, len(targets))
	for _, tg := range targets {
		// R = T * P^T, where P's columns are the principal axes and T's
		// columns are their targets. R's columns are R applied to X, Y, Z.
		col := func(e r3.Vec) r3.Vec {
			return x.Scale(pr.Axes[0].Dot(e)).Add(tg[0].Scale(pr.Axes[1].Dot(e))).Add(tg[1].Scale(pr.Axes[2].Dot(e)))
		}
		t, err := r3.FromBasis(r3.Basis{EX: col(x), EY: col(y), EZ: col(z)}, r3.Vec{})
		if err != nil {
			panic(err) // a product of two orthonormal bases is orthonormal
		}
		out = append(out, t)
	}
	return out
}

// tilted returns rot followed by a rotation of deg degrees about +Y. It turns
// +X out of the plate by |deg| degrees: toward -Z for positive deg, toward +Z
// for negative deg. The long axis is a line, so both signs give the same
// elevation; they differ in which end of the part ends up lower.
func tilted(rot r3.Transform, deg float64) r3.Transform {
	a := deg * math.Pi / 180
	c, s := math.Cos(a), math.Sin(a)
	tilt, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(c, 0, -s), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(s, 0, c)}, r3.Vec{})
	if err != nil {
		panic(err) // a rotation about Y is orthonormal
	}
	out, err := rot.Then(tilt)
	if err != nil {
		panic(err) // two rotations compose to a rotation
	}
	return out
}

type cluster struct {
	normal r3.Vec // the seed triangle's normal
	area   float64
}

// planarClusters groups triangles whose unit normals lie within
// PlanarClusterDeg of a cluster's seed normal, and returns the clusters by
// total area, largest first.
//
// Triangles are visited by decreasing area (ties by normal components), and
// each joins the first cluster it matches or seeds a new one. The visit
// order depends only on triangle geometry, so reordering triangles does not
// change the result. Once maxClusters clusters exist, a triangle that matches
// none of them is left out.
func planarClusters(ctx context.Context, m *mesh.Mesh) ([]cluster, error) {
	type tri struct {
		n    r3.Vec
		area float64
	}
	tris := make([]tri, 0, len(m.Triangles))
	for i := range m.Triangles {
		t := m.Triangle(i)
		n, ok := mesh.TriangleNormal(t)
		if !ok {
			continue
		}
		tris = append(tris, tri{n: n, area: mesh.TriangleArea(t)})
	}
	slices.SortFunc(tris, func(a, b tri) int {
		return cmp.Or(
			cmp.Compare(b.area, a.area),
			cmp.Compare(a.n.X, b.n.X),
			cmp.Compare(a.n.Y, b.n.Y),
			cmp.Compare(a.n.Z, b.n.Z),
		)
	})

	cosTol := math.Cos(PlanarClusterDeg * math.Pi / 180)
	var clusters []cluster
	for i, t := range tris {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		joined := false
		for k := range clusters {
			if clusters[k].normal.Dot(t.n) >= cosTol {
				clusters[k].area += t.area
				joined = true
				break
			}
		}
		if !joined && len(clusters) < maxClusters {
			clusters = append(clusters, cluster{normal: t.n, area: t.area})
		}
	}
	slices.SortStableFunc(clusters, func(a, b cluster) int { return cmp.Compare(b.area, a.area) })
	return clusters, nil
}
