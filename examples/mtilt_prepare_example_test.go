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

// bracket builds a Γ-shaped bracket: a 10 x 40 mm post with a 30 mm arm at
// the top, 20 mm deep. The profile is drawn in the XY plane, extruded along
// Z, and turned so the profile stands in the XZ plane.
func bracket(ctx context.Context, w *sketch.World, doc *decad.Document) (*decad.Body, error) {
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	outline := [][2]float64{{0, 0}, {10, 0}, {10, 30}, {40, 30}, {40, 40}, {0, 40}}
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
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		return nil, err
	}
	stand, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)}, r3.NewVec(0, 20, 0))
	if err != nil {
		return nil, err
	}
	return body.Placed(ctx, stand)
}

func Example_mtilt_prepare() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	body, err := bracket(ctx, w, doc)
	if err != nil {
		fmt.Printf("failed to build bracket: %s\n", err)
		return
	}

	// Keep the bracket upright so its arm, 30 mm above the plate, has to be
	// supported.
	res, err := mtilt.Prepare(ctx, body, mtilt.Options{Profile: mtilt.ExampleProfile(), KeepOrientation: true})
	if err != nil {
		fmt.Printf("failed to prepare: %s\n", err)
		return
	}
	first := res.Report.Supports[0]
	fmt.Printf("support bodies: %d\n", len(res.Supports))
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
	// support bodies: 41
	// first: support-0001 at (-7.75, -10.00), top 29.80 mm under a surface at 30.00 mm
	// files written: 42
}
