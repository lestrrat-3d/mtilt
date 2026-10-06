package main

import (
	"context"
	"testing"
)

// TestRenderImage renders the cheapest shot and checks the image size and the
// summary, so the CI gallery job catches a broken render path.
func TestRenderImage(t *testing.T) {
	var s shot
	for _, c := range shots() {
		if c.name == "oblique-cuboid" {
			s = c
		}
	}
	img, summary, err := renderImage(context.Background(), s)
	if err != nil {
		t.Fatal(err)
	}
	if got := img.Bounds().Size(); got.X != 2*paneWidth+gap || got.Y != paneHeight {
		t.Fatalf("image is %v", got)
	}
	if want := "planar_face, long axis 0.0 deg, height 10.0 mm, 0 supports"; summary != want {
		t.Fatalf("summary %q, want %q", summary, want)
	}
}
