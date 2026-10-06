package fixture

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Bodies builds the fixture shapes as decad bodies in one document. The
// shapes and dimensions match the triangle-soup builders in fixture.go.
type Bodies struct {
	World *sketch.World
	Doc   *decad.Document
}

// NewBodies returns a builder with a fresh sketch world and document.
func NewBodies() *Bodies {
	return &Bodies{World: sketch.NewWorld(), Doc: decad.New()}
}

// Prism extrudes a simple polygon in the XZ plane from Y = y0 to Y = y1, like
// the soup builder Prism.
func (b *Bodies) Prism(ctx context.Context, poly [][2]float64, y0, y1 float64) (*decad.Body, error) {
	s, err := b.World.CreateSketch(b.World.XY())
	if err != nil {
		return nil, err
	}
	pts := make([]*sketch.Point, len(poly))
	for i, p := range poly {
		pts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	profiles := s.Profiles()
	if len(profiles) != 1 {
		return nil, fmt.Errorf("fixture: polygon gave %d profiles", len(profiles))
	}
	body, err := b.Doc.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(y1 - y0), Dir: decad.Along})
	if err != nil {
		return nil, err
	}
	// The sketch's (u, v) become world (X, Z); the extrusion along +Z
	// becomes -Y, shifted so it spans [y0, y1].
	turn, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)}, r3.NewVec(0, y1, 0))
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, turn)
}

// Box returns the axis-aligned box from lo to hi.
func (b *Bodies) Box(ctx context.Context, lo, hi r3.Vec) (*decad.Body, error) {
	return b.Prism(ctx, [][2]float64{{lo.X, lo.Z}, {hi.X, lo.Z}, {hi.X, hi.Z}, {lo.X, hi.Z}}, lo.Y, hi.Y)
}

// Cube returns a 20 mm cube with one corner at the origin.
func (b *Bodies) Cube(ctx context.Context) (*decad.Body, error) {
	return b.Box(ctx, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 20))
}

// ObliqueCuboid returns the 40 x 20 x 10 mm cuboid rotated 37 degrees about
// (1, 2, 3), as the soup builder ObliqueCuboid.
func (b *Bodies) ObliqueCuboid(ctx context.Context) (*decad.Body, error) {
	box, err := b.Box(ctx, r3.NewVec(-20, -10, -5), r3.NewVec(20, 10, 5))
	if err != nil {
		return nil, err
	}
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	if err != nil {
		return nil, err
	}
	return box.Placed(ctx, rot)
}

// Bracket returns the Γ-shaped bracket of the soup builder Bracket.
func (b *Bodies) Bracket(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, bracketProfile, 0, 20)
}

// Bridge returns the Π-shaped part of the soup builder Bridge.
func (b *Bodies) Bridge(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, bridgeProfile, 0, 20)
}

// Occluded returns the ⊐-shaped part of the soup builder Occluded.
func (b *Bodies) Occluded(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, occludedProfile, 0, 20)
}

// Stick returns an 8 x 8 x 100 mm square stick standing on its end: its long
// axis is Z.
func (b *Bodies) Stick(ctx context.Context) (*decad.Body, error) {
	return b.Box(ctx, r3.NewVec(0, 0, 0), r3.NewVec(8, 8, 100))
}

// Rod returns a round rod, 8 mm across and 100 mm long, standing on its end:
// its axis is Z.
func (b *Bodies) Rod(ctx context.Context) (*decad.Body, error) {
	s, err := b.World.CreateSketch(b.World.XY())
	if err != nil {
		return nil, err
	}
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	circle := s.CreateCircle(c, 4)
	s.FixEntity(circle)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return b.Doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(100), Dir: decad.Along})
}

// Nail returns a 30 x 30 x 4 mm flange with an 8 x 8 mm stick rising from its
// center to Z = 104: a long part that stands on a wide base. Standing up it
// needs no supports; lying down, its stick needs supports under its whole
// length.
func (b *Bodies) Nail(ctx context.Context) (*decad.Body, error) {
	flange, err := b.Box(ctx, r3.NewVec(-15, -15, 0), r3.NewVec(15, 15, 4))
	if err != nil {
		return nil, err
	}
	// The stick starts inside the flange so the two overlap instead of
	// meeting face to face.
	stick, err := b.Box(ctx, r3.NewVec(-4, -4, 2), r3.NewVec(4, 4, 104))
	if err != nil {
		return nil, err
	}
	return decad.Union(ctx, flange, stick)
}
