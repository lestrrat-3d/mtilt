package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/mtilt"
)

func Example_mtilt_analyze() {
	// The cuboid is 40 x 20 x 10 mm, rotated so that no face is level.
	m, err := loadSTL("../testdata/oblique-cuboid.stl")
	if err != nil {
		fmt.Printf("failed to load mesh: %s\n", err)
		return
	}
	a, err := mtilt.Analyze(context.Background(), mtilt.Input{Mesh: m, Unit: mtilt.UnitMillimeter}, mtilt.Options{
		Profile: mtilt.ExampleProfile(),
	})
	if err != nil {
		fmt.Printf("failed to analyze: %s\n", err)
		return
	}
	orig := a.Report.Candidates[0]
	best := a.Report.Candidates[a.Ranking[0]]
	fmt.Printf("as given: rank %d, demand %.0f mm2, contact %.0f mm2\n",
		orig.Rank, orig.Metrics.SupportDemandProjectedAreaMM2, orig.Metrics.BedContactAreaMM2)
	fmt.Printf("best: %s, demand %.0f mm2, contact %.0f mm2, height %.1f mm\n",
		best.Source, best.Metrics.SupportDemandProjectedAreaMM2, best.Metrics.BedContactAreaMM2, best.Metrics.HeightMM)
	// Output:
	// as given: rank 23, demand 742 mm2, contact 0 mm2
	// best: planar_face, demand 0 mm2, contact 800 mm2, height 10.0 mm
}
