package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/sketch"
)

func Example_mtilt_blades() {
	ctx := context.Background()
	// The bracket of Example_mtilt_prepare, only 4 mm deep: its arm's
	// underside is a 30 x 4 mm strip with the post at one end.
	body, err := bracket(ctx, sketch.NewWorld(), decad.New(), 4)
	if err != nil {
		fmt.Printf("failed to build bracket: %s\n", err)
		return
	}

	res, err := mtilt.Prepare(ctx, body, mtilt.Options{Profile: mtilt.ExampleProfile(), KeepOrientation: true})
	if err != nil {
		fmt.Printf("failed to prepare: %s\n", err)
		return
	}
	// A straight strip gets blades instead of a field of pillars: the
	// post anchors one end, a blade stands at the free end, and one more
	// keeps every span under the profile's 10 mm max_bridge_mm.
	fmt.Printf("support bodies: %d\n", len(res.Supports))
	for _, sr := range res.Report.Supports {
		fmt.Printf("%s: %s at x %.1f, %.0f mm long, top %.2f mm\n", sr.ID, sr.Kind, sr.CenterMM[0], *sr.LengthMM, sr.TopZMM)
	}
	// Output:
	// support bodies: 3
	// support-0001: blade at x -0.5, 4 mm long, top 29.80 mm
	// support-0002: blade at x 9.5, 4 mm long, top 29.80 mm
	// support-0003: blade at x 19.5, 4 mm long, top 29.80 mm
}
