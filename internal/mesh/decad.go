package mesh

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// FromBody tessellates a decad body with chord tolerance tolMM and returns
// the mesh and its proven bound in millimeters: no point of the body's true
// surface lies farther than the bound from the mesh. Planar faces
// triangulate exactly, so a body with only planar faces has bound 0 up to the
// displacement its placements carried.
//
// The boundary proof is requested (decad.VerifyBoundary); a mesh without it
// is an error, so the mesh is watertight and consistently wound.
func FromBody(ctx context.Context, body *decad.Body, tolMM float64) (*Mesh, float64, error) {
	dm, err := body.Tessellate(ctx, units.Millimeters(tolMM), decad.WithVerification(decad.VerifyBoundary))
	if err != nil {
		return nil, 0, fmt.Errorf("mesh: tessellating body: %w", err)
	}
	if !dm.BoundaryVerified() {
		return nil, 0, fmt.Errorf("mesh: decad could not prove the tessellated boundary watertight")
	}
	bound, err := dm.Bound().In(units.Millimeter)
	if err != nil {
		return nil, 0, fmt.Errorf("mesh: reading tessellation bound: %w", err)
	}
	m := &Mesh{Vertices: dm.Vertices()}
	tris := dm.Triangles()
	m.Triangles = make([][3]uint32, len(tris))
	for i, t := range tris {
		m.Triangles[i] = [3]uint32{uint32(t[0]), uint32(t[1]), uint32(t[2])}
	}
	return m, bound, nil
}
