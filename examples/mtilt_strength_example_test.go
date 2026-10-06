package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_mtilt_strength() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	// A round rod, 8 mm across and 100 mm long, standing on its end.
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.FixEntity(s.CreateCircle(c, 4))
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(100), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// FDM parts are weakest across layer lines, so mtilt scores a long part
	// standing up as weak and lays it down.
	res, err := mtilt.Prepare(ctx, rod, mtilt.Options{Profile: mtilt.ExampleProfile()})
	if err != nil {
		fmt.Printf("failed to prepare: %s\n", err)
		return
	}
	given := res.Report.Candidates[0]
	best := res.Report.Candidates[*res.Report.Selected]
	fmt.Printf("elongation: %.2f\n", res.Report.Input.Elongation)
	fmt.Printf("as given: long axis at %.0f degrees, strength term %.2f\n", given.Metrics.LongAxisElevationDeg, given.Terms.Strength)
	fmt.Printf("selected: long axis at %.0f degrees, height %.1f mm, %d supports\n",
		best.Metrics.LongAxisElevationDeg, best.Metrics.HeightMM, len(res.Supports))
	// Output:
	// elongation: 0.99
	// as given: long axis at 90 degrees, strength term 0.50
	// selected: long axis at 0 degrees, height 8.0 mm, 0 supports
}
