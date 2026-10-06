package examples_test

import (
	"context"
	"errors"
	"fmt"
	"os"

	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
)

// loadSTL reads one of the repository's test fixtures into a mesh.
func loadSTL(path string) (*mesh.Mesh, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	file, err := stl.Read(f, stl.Limits{})
	if err != nil {
		return nil, err
	}
	return mesh.FromSoup(file.Triangles)
}

func Example_mtilt_prepare() {
	// The bracket's arm overhangs 30 mm above the plate. Keeping the input
	// orientation forces mtilt to support it instead of laying it flat.
	m, err := loadSTL("../testdata/bracket.stl")
	if err != nil {
		fmt.Printf("failed to load mesh: %s\n", err)
		return
	}
	res, err := mtilt.Prepare(context.Background(), mtilt.Input{Mesh: m, Unit: mtilt.UnitMillimeter}, mtilt.Options{
		Profile:         mtilt.ExampleProfile(),
		KeepOrientation: true,
	})
	if err != nil {
		fmt.Printf("failed to prepare: %s\n", err)
		return
	}

	failed := 0
	for _, c := range res.Report.Validation {
		if c.Status == mesh.StatusFailed {
			failed++
		}
	}
	first := res.Report.Supports[0]
	fmt.Printf("support bodies: %d\n", len(res.Supports))
	fmt.Printf("first: %s at (%.2f, %.2f), top %.2f mm under a surface at %.2f mm\n",
		first.ID, first.CenterMM[0], first.CenterMM[1], first.TopZMM, first.SurfaceZMM)
	fmt.Printf("failed checks: %d\n", failed)
	// Output:
	// support bodies: 41
	// first: support-0001 at (-7.75, -10.00), top 29.80 mm under a surface at 30.00 mm
	// failed checks: 0
}

func Example_mtilt_prepare_failure() {
	// The top arm of this part hangs over its own base, so no pillar
	// standing on the plate can reach it.
	m, err := loadSTL("../testdata/occluded.stl")
	if err != nil {
		fmt.Printf("failed to load mesh: %s\n", err)
		return
	}
	_, err = mtilt.Prepare(context.Background(), mtilt.Input{Mesh: m, Unit: mtilt.UnitMillimeter}, mtilt.Options{
		Profile:         mtilt.ExampleProfile(),
		KeepOrientation: true,
	})
	var fe *mtilt.FailureError
	if !errors.As(err, &fe) {
		fmt.Printf("expected a failure report, got %v\n", err)
		return
	}
	att := fe.Report.Candidates[0].Attempt
	fmt.Println(errors.Is(err, mtilt.ErrNoFeasibleCandidate))
	fmt.Printf("%d of %d demand samples uncovered\n", att.UncoveredCount, att.Samples)
	fmt.Println(att.Uncovered[0].Reason)
	// Output:
	// true
	// 651 of 651 demand samples uncovered
	// occluded: the vertical line from the plate meets other model geometry first
}
