// Package orient searches for the print orientation that needs the least
// support while keeping a long part's long axis close to flat. The long axis
// (principal.go), the support estimate and search (candidates.go, search.go),
// measurement (measure.go) and ranking (rank.go) are separate, so each can be
// improved on its own.
//
// An orientation is fully described, for support purposes, by the "down"
// direction: the unit vector in the input frame that ends up pointing at the
// build plate (-Z). Turning the part about the vertical axis afterwards moves
// no overhang and changes no support.
package orient

import (
	"cmp"
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
)

// Source records how a candidate down direction was produced.
type Source string

// Candidate sources.
const (
	// SourceOriginal is the input orientation: down is -Z.
	SourceOriginal Source = "original"
	// SourceAxis is one of the six coordinate directions.
	SourceAxis Source = "axis"
	// SourcePlanarFace puts a dominant flat face on the plate.
	SourcePlanarFace Source = "planar_face"
	// SourceSweep is a direction of the even sweep over the sphere.
	SourceSweep Source = "sweep"
	// SourceRefined is a direction local refinement moved to.
	SourceRefined Source = "refined"
)

// PlanarClusterDeg is the angle in degrees within which two triangle normals
// join the same planar-face cluster.
const PlanarClusterDeg = 1.0

// maxClusters caps how many planar-face clusters are tracked. See
// planarClusters.
const maxClusters = 1024

// Candidate is one orientation the search produced.
type Candidate struct {
	ID     int
	Source Source
	// Down is the input-frame unit direction that faces the plate.
	Down r3.Vec
	// Rotation is the minimal rotation that turns Down to -Z. Placement
	// on the plate happens afterwards (see Place).
	Rotation r3.Transform
	// FaceNormal and FaceAreaMM2 describe the face cluster of a
	// planar_face candidate; zero otherwise.
	FaceNormal  r3.Vec
	FaceAreaMM2 float64
	// ElevationDeg is the long axis's angle to the plate in this
	// orientation.
	ElevationDeg float64
	// Allowed is false when the elevation is over the tilt limit.
	Allowed bool
	// Estimate is the support cost estimate (see Estimator).
	Estimate Cost
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
