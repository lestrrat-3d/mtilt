// Command gallery renders mtilt's README images with solidlens.
//
// It lives in its own module so solidlens stays out of the mtilt library's
// dependency list. Run it from this directory with `go run .`; it prepares
// every shot's fixture body with mtilt and writes one PNG per shot under the
// repository's docs/images. Each image has two panes: on the left the body as
// given, set on the build plate; on the right the model as mtilt oriented it,
// with its support bodies.
//
// Flags:
//
//   - -out <dir> writes under that directory instead of ../docs/images.
//   - -only <names> renders just the named shots, comma separated.
//   - -list prints the shot names and exits.
package main

import (
	"context"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"math"
	"os"
	"path/filepath"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
	"golang.org/x/image/font"
	"golang.org/x/image/font/basicfont"
	"golang.org/x/image/math/fixed"
)

// shot is one README image: a fixture and the options it is prepared with.
type shot struct {
	name string
	body func(*fixture.Bodies, context.Context) (*decad.Body, error)
	opts mtilt.Options
}

func shots() []shot {
	keep := mtilt.Options{Profile: mtilt.ExampleProfile(), KeepOrientation: true}
	search := mtilt.Options{Profile: mtilt.ExampleProfile()}
	return []shot{
		{name: "oblique-cuboid", body: (*fixture.Bodies).ObliqueCuboid, opts: search},
		{name: "stick", body: (*fixture.Bodies).Stick, opts: search},
		{name: "nail", body: (*fixture.Bodies).Nail, opts: search},
		{name: "bracket-kept", body: (*fixture.Bodies).Bracket, opts: keep},
		{name: "bridge-kept", body: (*fixture.Bodies).Bridge, opts: keep},
		{name: "occluded-kept", body: (*fixture.Bodies).Occluded, opts: keep},
	}
}

// Pane and image sizes in pixels.
const (
	paneWidth  = 480
	paneHeight = 400
	gap        = 8
)

// chord is the tessellation tolerance for display meshes.
var chord = units.Millimeters(0.02)

var (
	background   = solidlens.RGB(0.82, 0.86, 0.93)
	modelColor   = solidlens.RGB(0.18, 0.47, 1)
	supportColor = solidlens.RGB(1, 0.68, 0.08)
	plateColor   = solidlens.RGB(0.32, 0.34, 0.4)
	edgeColor    = solidlens.RGB(0.05, 0.08, 0.15)
)

func main() {
	out := flag.String("out", filepath.Join("..", "docs", "images"), "directory to write images into")
	only := flag.String("only", "", "comma-separated shot names to render (default all)")
	list := flag.Bool("list", false, "print the shot names and exit")
	flag.Parse()

	if *list {
		for _, s := range shots() {
			fmt.Println(s.name)
		}
		return
	}
	var want []string
	if *only != "" {
		want = strings.Split(*only, ",")
	}
	ctx := context.Background()
	if err := os.MkdirAll(*out, 0o755); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	for _, s := range shots() {
		if want != nil && !slices.Contains(want, s.name) {
			continue
		}
		path := filepath.Join(*out, s.name+".png")
		summary, err := render(ctx, s, path)
		if err != nil {
			fmt.Fprintf(os.Stderr, "%s: %v\n", s.name, err)
			os.Exit(1)
		}
		fmt.Printf("%s: %s\n", path, summary)
	}
}

// render prepares one shot and writes its two-pane image to path. It returns
// a one-line summary of what mtilt chose.
func render(ctx context.Context, s shot, path string) (string, error) {
	img, summary, err := renderImage(ctx, s)
	if err != nil {
		return "", err
	}
	f, err := os.Create(path)
	if err != nil {
		return "", err
	}
	if err := png.Encode(f, img); err != nil {
		f.Close()
		return "", err
	}
	return summary, f.Close()
}

func renderImage(ctx context.Context, s shot) (*image.RGBA, string, error) {
	fb := fixture.NewBodies()
	body, err := s.body(fb, ctx)
	if err != nil {
		return nil, "", fmt.Errorf("building fixture: %w", err)
	}
	given, err := tessellate(ctx, body)
	if err != nil {
		return nil, "", err
	}
	res, err := mtilt.Prepare(ctx, body, s.opts)
	if err != nil {
		return nil, "", fmt.Errorf("preparing: %w", err)
	}
	model, err := tessellate(ctx, res.Model)
	if err != nil {
		return nil, "", err
	}
	supports := make([]*decad.Mesh, len(res.Supports))
	for i, b := range res.Supports {
		if supports[i], err = tessellate(ctx, b); err != nil {
			return nil, "", err
		}
	}

	// Both panes share one camera distance, so the two show the part at
	// one scale; each looks at the middle of its own assembly.
	left := onPlate(given)
	all := slices.Clone(model.Vertices())
	for _, m := range supports {
		all = append(all, m.Vertices()...)
	}
	radius := math.Max(boundingRadius(left.verts), boundingRadius(all))

	leftImg, err := solidlens.Render(ctx, scene(radius, center(left.verts), solidModel(left, modelColor)), solidlens.Settings{Width: paneWidth, Height: paneHeight})
	if err != nil {
		return nil, "", err
	}
	models := []solidlens.Model{solidModel(model, modelColor)}
	for _, m := range supports {
		models = append(models, solidModel(m, supportColor))
	}
	rightImg, err := solidlens.Render(ctx, scene(radius, center(all), models...), solidlens.Settings{Width: paneWidth, Height: paneHeight})
	if err != nil {
		return nil, "", err
	}

	img := image.NewRGBA(image.Rect(0, 0, 2*paneWidth+gap, paneHeight))
	draw.Draw(img, img.Bounds(), &image.Uniform{C: color.White}, image.Point{}, draw.Src)
	draw.Draw(img, image.Rect(0, 0, paneWidth, paneHeight), leftImg, image.Point{}, draw.Src)
	draw.Draw(img, image.Rect(paneWidth+gap, 0, 2*paneWidth+gap, paneHeight), rightImg, image.Point{}, draw.Src)

	sel := res.Report.Candidates[*res.Report.Selected]
	given0 := res.Report.Candidates[0]
	label(img, 0, fmt.Sprintf("as given: long axis %.0f deg, height %.0f mm", given0.Metrics.LongAxisElevationDeg, given0.Metrics.HeightMM))
	label(img, paneWidth+gap, fmt.Sprintf("mtilt: long axis %.0f deg, height %.0f mm, %d supports",
		sel.Metrics.LongAxisElevationDeg, sel.Metrics.HeightMM, len(res.Supports)))
	summary := fmt.Sprintf("%s, long axis %.1f deg, height %.1f mm, %d supports",
		sel.Source, sel.Metrics.LongAxisElevationDeg, sel.Metrics.HeightMM, len(res.Supports))
	return img, summary, nil
}

func tessellate(ctx context.Context, b *decad.Body) (*decad.Mesh, error) {
	return b.Tessellate(ctx, chord, decad.WithVerification(decad.VerifyNone))
}

// meshSource adapts plain vertex and triangle slices to
// solidlens.TriangleSource.
type meshSource struct {
	verts []r3.Vec
	tris  [][3]int
}

func (m meshSource) Vertices() []r3.Vec  { return m.verts }
func (m meshSource) Triangles() [][3]int { return m.tris }

// onPlate moves a mesh, unrotated, so its lowest point is on the plate and
// its bounding box is centered on the Z axis: the same placement mtilt gives
// a model it keeps in its given orientation.
func onPlate(m *decad.Mesh) meshSource {
	vs := m.Vertices()
	lo, hi := bounds(vs)
	shift := r3.NewVec(-(lo.X+hi.X)/2, -(lo.Y+hi.Y)/2, -lo.Z)
	out := make([]r3.Vec, len(vs))
	for i, v := range vs {
		out[i] = v.Add(shift)
	}
	return meshSource{verts: out, tris: m.Triangles()}
}

func bounds(vs []r3.Vec) (r3.Vec, r3.Vec) {
	lo, hi := vs[0], vs[0]
	for _, v := range vs[1:] {
		lo = r3.NewVec(math.Min(lo.X, v.X), math.Min(lo.Y, v.Y), math.Min(lo.Z, v.Z))
		hi = r3.NewVec(math.Max(hi.X, v.X), math.Max(hi.Y, v.Y), math.Max(hi.Z, v.Z))
	}
	return lo, hi
}

// label writes text in the top-left corner of the pane starting at x.
func label(img *image.RGBA, x int, text string) {
	d := font.Drawer{
		Dst:  img,
		Src:  image.NewUniform(color.RGBA{R: 20, G: 24, B: 40, A: 255}),
		Face: basicfont.Face7x13,
		Dot:  fixed.P(x+10, 20),
	}
	d.DrawString(text)
}

// center returns the middle of the bounding box of vs.
func center(vs []r3.Vec) r3.Vec {
	lo, hi := bounds(vs)
	return lo.Add(hi).Scale(0.5)
}

// boundingRadius is half the diagonal of a placed mesh's bounding box,
// measured from the point the camera looks at.
func boundingRadius(vs []r3.Vec) float64 {
	lo, hi := bounds(vs)
	return hi.Sub(lo).Len() / 2
}

func solidModel(src solidlens.TriangleSource, c solidlens.Color) solidlens.Model {
	return solidlens.Model{
		Mesh:     src,
		Material: solidlens.Material{Color: c, Ambient: 0.35},
		Edges:    solidlens.Edges{Enabled: true, Color: edgeColor, Width: 1, CreaseAngle: 30},
	}
}

// scene frames a placed assembly that fits in a sphere of radius r around
// target: the camera looks at target from the front right and above, and a
// square plate 3r wide is drawn just under Z = 0.
func scene(r float64, target r3.Vec, models ...solidlens.Model) solidlens.Scene {
	const fov = 35.0
	dist := r / math.Sin(fov/2*math.Pi/180) * 1.05
	dir, _ := r3.NewVec(1, -1.6, 1).Normalize()
	half := 1.5 * r
	plate := meshSource{
		verts: []r3.Vec{
			r3.NewVec(-half, -half, -0.6), r3.NewVec(half, -half, -0.6),
			r3.NewVec(half, half, -0.6), r3.NewVec(-half, half, -0.6),
			r3.NewVec(-half, -half, -0.05), r3.NewVec(half, -half, -0.05),
			r3.NewVec(half, half, -0.05), r3.NewVec(-half, half, -0.05),
		},
		tris: [][3]int{
			{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
			{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
		},
	}
	all := append([]solidlens.Model{{Mesh: plate, Material: solidlens.Material{Color: plateColor, Ambient: 0.5}}}, models...)
	return solidlens.Scene{
		Camera: solidlens.Camera{
			Position: target.Add(dir.Scale(dist)),
			Target:   target,
			Up:       r3.NewVec(0, 0, 1),
			FOV:      fov,
		},
		Models: all,
		DirectionalLights: []solidlens.DirectionalLight{
			{Direction: r3.NewVec(-0.7, 0.12, -1), Color: solidlens.RGB(1, 1, 1), Intensity: 1.2},
			{Direction: r3.NewVec(0.6, -0.2, -0.6), Color: solidlens.RGB(0.25, 0.55, 1), Intensity: 0.4},
		},
		Background: background,
	}
}
