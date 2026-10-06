package mesh

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/r3"
)

// Status is the outcome of one check.
type Status string

// The three outcomes of a check. StatusUnchecked marks a property mtilt does
// not test; it never means the property holds.
const (
	StatusPassed    Status = "passed"
	StatusFailed    Status = "failed"
	StatusUnchecked Status = "unchecked"
)

// Names of the checks Validate reports, in report order.
const (
	CheckTriangleCount    = "triangle_count"
	CheckNondegenerate    = "nondegenerate_triangles"
	CheckClosed           = "closed"
	CheckEdgeManifold     = "edge_manifold"
	CheckVertexManifold   = "vertex_manifold"
	CheckConsistentWind   = "consistent_winding"
	CheckOutward          = "outward_orientation"
	CheckSingleComponent  = "single_component"
	CheckSelfIntersection = "self_intersection"
)

// Check is the outcome of one named check.
type Check struct {
	Name   string `json:"name"`
	Status Status `json:"status"`
	Detail string `json:"detail,omitempty"`
}

// Report lists the checks Validate ran, and the ones it did not.
type Report struct {
	Checks []Check `json:"checks"`
	// Components is the number of edge-connected triangle groups.
	Components int `json:"components"`
}

// Failed returns the checks whose status is StatusFailed.
func (r Report) Failed() []Check {
	var out []Check
	for _, c := range r.Checks {
		if c.Status == StatusFailed {
			out = append(out, c)
		}
	}
	return out
}

// Status returns the status of the named check, and false when the report
// has no check of that name.
func (r Report) Status(name string) (Status, bool) {
	for _, c := range r.Checks {
		if c.Name == name {
			return c.Status, true
		}
	}
	return "", false
}

// maxListed caps how many offending triangle or vertex indices a failed check
// lists in its detail text.
const maxListed = 5

// Validate runs every topology and geometry check mtilt implements on m and
// reports each outcome. Coordinates are known to be finite: FromSoup refuses
// anything else.
//
// The checks are:
//
//   - triangle_count: the mesh has at least four triangles (the fewest a
//     closed surface can have).
//   - nondegenerate_triangles: no triangle repeats a vertex index, and no
//     triangle's height over its longest edge is at most tol.Length.
//   - closed: every edge is shared by at least two triangles.
//   - edge_manifold: no edge is shared by more than two triangles.
//   - vertex_manifold: the triangles around each vertex form one fan
//     connected through shared edges.
//   - consistent_winding: the two triangles on each edge traverse it in
//     opposite directions.
//   - outward_orientation: the enclosed signed volume is positive, so the
//     winding puts normals on the outside. It is unchecked unless closed,
//     edge_manifold and consistent_winding all passed.
//   - single_component: all triangles are connected through shared edges.
//   - self_intersection: always unchecked. mtilt does not test whether
//     triangles cross each other.
//
// Validate returns ctx.Err() when ctx is cancelled during the checks.
func Validate(ctx context.Context, m *Mesh, tol Tolerance) (Report, error) {
	var r Report
	add := func(name string, bad []int, what string) {
		if len(bad) == 0 {
			r.Checks = append(r.Checks, Check{Name: name, Status: StatusPassed})
			return
		}
		r.Checks = append(r.Checks, Check{Name: name, Status: StatusFailed, Detail: listDetail(len(bad), what, bad)})
	}

	if len(m.Triangles) < 4 {
		r.Checks = append(r.Checks, Check{
			Name: CheckTriangleCount, Status: StatusFailed,
			Detail: fmt.Sprintf("%d triangles; a closed surface needs at least 4", len(m.Triangles)),
		})
	} else {
		r.Checks = append(r.Checks, Check{Name: CheckTriangleCount, Status: StatusPassed})
	}

	var degenerate []int
	for i, t := range m.Triangles {
		if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] || triangleHeight(m.Triangle(i)) <= tol.Length {
			degenerate = append(degenerate, i)
		}
	}
	add(CheckNondegenerate, degenerate, "degenerate triangle(s)")

	if err := ctx.Err(); err != nil {
		return Report{}, err
	}

	edges := buildEdges(m)
	var open, nonManifold, flipped []int
	for _, e := range edges.order {
		info := edges.byKey[e]
		switch {
		case len(info.tris) == 1:
			open = append(open, info.tris[0])
		case len(info.tris) > 2:
			nonManifold = append(nonManifold, info.tris[0])
		case info.forward != 1:
			// Two triangles on a manifold edge must traverse it once each way.
			flipped = append(flipped, info.tris[1])
		}
	}
	add(CheckClosed, open, "triangle(s) on an open edge")
	add(CheckEdgeManifold, nonManifold, "triangle(s) on an edge shared by more than two triangles")
	add(CheckVertexManifold, nonManifoldVertices(m, edges), "vertex(es) joining separate fans")
	add(CheckConsistentWind, flipped, "triangle(s) wound against a neighbor")

	if err := ctx.Err(); err != nil {
		return Report{}, err
	}

	switch {
	case len(open) > 0 || len(nonManifold) > 0 || len(flipped) > 0:
		r.Checks = append(r.Checks, Check{
			Name: CheckOutward, Status: StatusUnchecked,
			Detail: "needs a closed, edge-manifold, consistently wound mesh",
		})
	case m.Volume() > 0:
		r.Checks = append(r.Checks, Check{Name: CheckOutward, Status: StatusPassed})
	default:
		r.Checks = append(r.Checks, Check{
			Name: CheckOutward, Status: StatusFailed,
			Detail: fmt.Sprintf("signed volume %g is not positive; the winding puts normals inside", m.Volume()),
		})
	}

	r.Components = countComponents(m, edges)
	if r.Components == 1 {
		r.Checks = append(r.Checks, Check{Name: CheckSingleComponent, Status: StatusPassed})
	} else {
		r.Checks = append(r.Checks, Check{
			Name: CheckSingleComponent, Status: StatusFailed,
			Detail: fmt.Sprintf("%d edge-connected components", r.Components),
		})
	}

	r.Checks = append(r.Checks, Check{
		Name: CheckSelfIntersection, Status: StatusUnchecked,
		Detail: "mtilt does not test whether triangles cross each other",
	})
	return r, nil
}

func listDetail(n int, what string, idx []int) string {
	var sb strings.Builder
	fmt.Fprintf(&sb, "%d %s, first: ", n, what)
	for i, v := range idx {
		if i == maxListed {
			sb.WriteString(", ...")
			break
		}
		if i > 0 {
			sb.WriteString(", ")
		}
		fmt.Fprintf(&sb, "%d", v)
	}
	return sb.String()
}

// triangleHeight returns twice the triangle's area over its longest edge: its
// height over that edge. A sliver with three distinct but nearly collinear
// vertices has a tiny height even when its edges are long.
func triangleHeight(t [3]r3.Vec) float64 {
	longest := 0.0
	for k := range 3 {
		longest = max(longest, t[(k+1)%3].Sub(t[k]).Len())
	}
	if longest == 0 {
		return 0
	}
	return 2 * TriangleArea(t) / longest
}

type edgeInfo struct {
	tris []int
	// forward counts the triangles that traverse the edge from its lower to
	// its higher vertex index.
	forward int
}

type edgeTable struct {
	byKey map[uint64]*edgeInfo
	// order lists keys in first-seen order so reports do not depend on map
	// iteration order.
	order []uint64
}

func edgeKey(a, b uint32) uint64 {
	if a > b {
		a, b = b, a
	}
	return uint64(a)<<32 | uint64(b)
}

func buildEdges(m *Mesh) edgeTable {
	et := edgeTable{byKey: make(map[uint64]*edgeInfo, len(m.Triangles)*3/2)}
	for ti, t := range m.Triangles {
		for k := range 3 {
			a, b := t[k], t[(k+1)%3]
			if a == b {
				continue
			}
			key := edgeKey(a, b)
			info, ok := et.byKey[key]
			if !ok {
				info = &edgeInfo{}
				et.byKey[key] = info
				et.order = append(et.order, key)
			}
			info.tris = append(info.tris, ti)
			if a < b {
				info.forward++
			}
		}
	}
	return et
}

type unionFind []int32

func newUnionFind(n int) unionFind {
	u := make(unionFind, n)
	for i := range u {
		u[i] = int32(i)
	}
	return u
}

func (u unionFind) find(i int32) int32 {
	for u[i] != i {
		u[i] = u[u[i]]
		i = u[i]
	}
	return i
}

func (u unionFind) union(a, b int32) {
	ra, rb := u.find(a), u.find(b)
	if ra == rb {
		return
	}
	if ra < rb {
		u[rb] = ra
		return
	}
	u[ra] = rb
}

// nonManifoldVertices groups each vertex's triangle corners by the manifold
// edges they share; a vertex whose corners fall into more than one group sits
// where separate fans touch.
func nonManifoldVertices(m *Mesh, et edgeTable) []int {
	corners := newUnionFind(3 * len(m.Triangles))
	corner := func(tri int, v uint32) int32 {
		t := m.Triangles[tri]
		for k := range 3 {
			if t[k] == v {
				return int32(3*tri + k)
			}
		}
		return -1
	}
	for _, key := range et.order {
		info := et.byKey[key]
		if len(info.tris) != 2 {
			continue
		}
		a, b := uint32(key>>32), uint32(key)
		t1, t2 := info.tris[0], info.tris[1]
		corners.union(corner(t1, a), corner(t2, a))
		corners.union(corner(t1, b), corner(t2, b))
	}
	first := make([]int32, len(m.Vertices))
	for i := range first {
		first[i] = -1
	}
	var bad []int
	flagged := make(map[uint32]struct{})
	for ti, t := range m.Triangles {
		for k := range 3 {
			v := t[k]
			root := corners.find(int32(3*ti + k))
			if first[v] == -1 {
				first[v] = root
				continue
			}
			if first[v] == root {
				continue
			}
			if _, ok := flagged[v]; !ok {
				flagged[v] = struct{}{}
				bad = append(bad, int(v))
			}
		}
	}
	return bad
}

func countComponents(m *Mesh, et edgeTable) int {
	if len(m.Triangles) == 0 {
		return 0
	}
	u := newUnionFind(len(m.Triangles))
	for _, key := range et.order {
		tris := et.byKey[key].tris
		for _, t := range tris[1:] {
			u.union(int32(tris[0]), int32(t))
		}
	}
	roots := make(map[int32]struct{})
	for i := range m.Triangles {
		roots[u.find(int32(i))] = struct{}{}
	}
	return len(roots)
}

// ComponentCount returns the number of edge-connected triangle groups in m.
func ComponentCount(m *Mesh) int {
	return countComponents(m, buildEdges(m))
}
