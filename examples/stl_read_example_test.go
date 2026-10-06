package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
)

func Example_stl_read() {
	// A tetrahedron with one facet wound the wrong way. Stored normals are
	// ignored; the winding decides which way a facet faces.
	const src = `solid tetra
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 0 10 0
  vertex 10 0 0
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 10 0 0
  vertex 0 0 10
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 0 0 0
  vertex 0 0 10
  vertex 0 10 0
 endloop
endfacet
facet normal 0 0 0
 outer loop
  vertex 10 0 0
  vertex 0 0 10
  vertex 0 10 0
 endloop
endfacet
endsolid tetra
`
	file, err := stl.Read(strings.NewReader(src), stl.Limits{})
	if err != nil {
		fmt.Printf("failed to read STL: %s\n", err)
		return
	}
	m, err := mesh.FromSoup(file.Triangles)
	if err != nil {
		fmt.Printf("failed to build mesh: %s\n", err)
		return
	}
	rep, err := mesh.Validate(context.Background(), m, mesh.ToleranceFor(m.Bounds()))
	if err != nil {
		fmt.Printf("failed to validate: %s\n", err)
		return
	}
	fmt.Printf("%s, %d triangles, %d vertices\n", file.Format, len(m.Triangles), len(m.Vertices))
	for _, c := range rep.Checks {
		fmt.Printf("%s %s\n", c.Status, c.Name)
	}
	// Output:
	// ascii, 4 triangles, 4 vertices
	// passed triangle_count
	// passed nondegenerate_triangles
	// passed closed
	// passed edge_manifold
	// passed vertex_manifold
	// failed consistent_winding
	// unchecked outward_orientation
	// passed single_component
	// unchecked self_intersection
}
