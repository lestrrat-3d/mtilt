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
	body, err := decad.Extrude(ctx, b.Doc, s, profiles[0], decad.Distance{D: units.Millimeters(y1 - y0), Dir: decad.Along})
	if err != nil {
		return nil, err
	}
	// The sketch's (u, v) become world (X, Z); the extrusion along +Z
	// becomes -Y, shifted so it spans [y0, y1].
	turn, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)}, r3.NewVec(0, y1, 0))
	if err != nil {
		return nil, err
	}
	return decad.Transform(ctx, body, turn)
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
	return decad.Transform(ctx, box, rot)
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
	return decad.Extrude(ctx, b.Doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(100), Dir: decad.Along})
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

// PlateBox returns the 100 x 40 x 5 mm plate of the soup builder PlateBox.
func (b *Bodies) PlateBox(ctx context.Context) (*decad.Body, error) {
	return b.Box(ctx, r3.NewVec(0, 0, 0), r3.NewVec(100, 40, 5))
}

// ChannelPlate returns the channelled plate of the soup builder
// ChannelPlate. Lying flat, the channel's ceiling at Z = 3 needs support.
func (b *Bodies) ChannelPlate(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, channelProfile, 0, 60)
}

// RecessPlate returns ChannelPlate with the channel 2 mm high and the
// groove 2 mm deep. Lying flat on either side, the channel's ceiling is
// 2 mm above the plate: too low for the shortest pillar of the example
// profile.
func (b *Bodies) RecessPlate(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, recessProfile, 0, 60)
}

// LPlate returns the L-shaped part of the soup builder LPlate.
func (b *Bodies) LPlate(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, lPlateProfile, 0, 40)
}

// ArmedStick returns the 8 x 8 x 100 mm stick of Stick with two flat arms,
// 2 mm thick along Y (Y 3..5) and spanning Z 40..60, reaching out to X = 48
// and X = -40. Each arm overlaps the stick by 1 mm. The part is both long
// (elongation over 0.1) and flat (flatness under 0.2).
func (b *Bodies) ArmedStick(ctx context.Context) (*decad.Body, error) {
	stick, err := b.Stick(ctx)
	if err != nil {
		return nil, err
	}
	right, err := b.Box(ctx, r3.NewVec(7, 3, 40), r3.NewVec(48, 5, 60))
	if err != nil {
		return nil, err
	}
	left, err := b.Box(ctx, r3.NewVec(-40, 3, 40), r3.NewVec(1, 5, 60))
	if err != nil {
		return nil, err
	}
	one, err := decad.Union(ctx, stick, right)
	if err != nil {
		return nil, err
	}
	return decad.Union(ctx, one, left)
}

// StackedShelves returns the two-shelf part of the soup builder
// StackedShelves.
func (b *Bodies) StackedShelves(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, stackedShelvesProfile, 0, 20)
}

// Slot returns a part with a 3 mm wide slot under a beam, 20 mm deep along
// Y: posts at X 0..10 and X 13..23, joined by a beam at Z 30..40. The slot's
// ceiling is a 3 mm span between two walls.
func (b *Bodies) Slot(ctx context.Context) (*decad.Body, error) {
	return b.Prism(ctx, [][2]float64{{0, 0}, {10, 0}, {10, 30}, {13, 30}, {13, 0}, {23, 0}, {23, 40}, {0, 40}}, 0, 20)
}
