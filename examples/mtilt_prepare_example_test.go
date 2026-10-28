package examples_test

import (
	"context"
	"fmt"
	"os"
	"path/filepath"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// hookPlate builds a wall hook's mounting plate, 60 mm deep: a 70 x 4 mm
// plate with a hook along one edge, whose arm reaches 12 mm past the plate's
// edge at Z = 8 and ends in a lip. The profile is drawn in the XY plane,
// extruded along Z, and turned so the profile stands in the XZ plane.
func hookPlate(ctx context.Context, w *sketch.World, doc *decad.Document) (*decad.Body, error) {
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	outline := [][2]float64{{0, 0}, {70, 0}, {70, 8}, {82, 8}, {82, 15}, {78, 15}, {78, 12}, {66, 12}, {66, 4}, {0, 4}}
	pts := make([]*sketch.Point, len(outline))
	for i, o := range outline {
		pts[i] = s.CreatePoint(o[0], o[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	body, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
	if err != nil {
		return nil, err
	}
	stand, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)}, r3.NewVec(0, 60, 0))
	if err != nil {
		return nil, err
	}
	return decad.Transform(ctx, body, stand)
}

func Example_mtilt_prepare() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	body, err := hookPlate(ctx, w, doc)
	if err != nil {
		fmt.Printf("failed to build hook plate: %s\n", err)
		return
	}

	// mtilt chooses the orientation. The part is a plate, so it lies flat;
	// upside down it would rest on the hook's lip, so it stays as drawn and
	// pillars hold the hook's arm past the plate's edge.
	res, err := mtilt.Prepare(ctx, body, mtilt.Options{Profile: mtilt.ExampleProfile()})
	if err != nil {
		fmt.Printf("failed to prepare: %s\n", err)
		return
	}
	sel := res.Report.Candidates[*res.Report.Selected]
	fmt.Printf("selected: candidate %d (%s), down (%.0f, %.0f, %.0f), height %.2f mm\n",
		sel.ID, sel.Source, sel.Down[0], sel.Down[1], sel.Down[2], sel.Metrics.HeightMM)
	fmt.Printf("support bodies: %d\n", len(res.Supports))
	if len(res.Supports) == 0 {
		return
	}
	first := res.Report.Supports[0]
	fmt.Printf("first: %s at (%.2f, %.2f), top %.2f mm under a surface at %.2f mm\n",
		first.ID, first.CenterMM[0], first.CenterMM[1], first.TopZMM, first.SurfaceZMM)

	// The model and the supports are decad bodies in one frame; write each
	// one with decad's exporters.
	dir, err := os.MkdirTemp("", ".tmp-mtilt-example-*")
	if err != nil {
		fmt.Printf("failed to create directory: %s\n", err)
		return
	}
	defer os.RemoveAll(dir)
	write := func(name string, b *decad.Body) error {
		f, err := os.Create(filepath.Join(dir, name))
		if err != nil {
			return err
		}
		defer f.Close()
		return export.STL(ctx, f, b, units.Millimeters(0.01))
	}
	if err := write("model.stl", res.Model); err != nil {
		fmt.Printf("failed to write model: %s\n", err)
		return
	}
	for i, s := range res.Supports {
		if err := write(res.Report.Supports[i].ID+".stl", s); err != nil {
			fmt.Printf("failed to write support: %s\n", err)
			return
		}
	}
	files, err := os.ReadDir(dir)
	if err != nil {
		fmt.Printf("failed to list files: %s\n", err)
		return
	}
	fmt.Printf("files written: %d\n", len(files))
	// Output:
	// selected: candidate 0 (original), down (0, 0, -1), height 15.00 mm
	// support bodies: 46
	// first: support-0001 at (31.25, -30.00), top 7.80 mm under a surface at 8.00 mm
	// files written: 47
}
