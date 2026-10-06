package fixture

import (
	"bytes"
	"strings"

	"github.com/lestrrat-3d/mtilt/stl"
	"github.com/lestrrat-3d/r3"
)

// File is one file in testdata/ and the bytes it must hold.
type File struct {
	Name string
	Data []byte
}

// Files returns every file in testdata/, built from the shapes in this
// package. A test compares the committed files against it, so a change to a
// shape or to the STL writer shows up as a test failure until the files are
// regenerated.
func Files() []File {
	binaryFile := func(tris [][3]r3.Vec, header string) []byte {
		var buf bytes.Buffer
		if err := stl.WriteBinary(&buf, tris, header); err != nil {
			panic(err) // fixed, valid shapes
		}
		return buf.Bytes()
	}

	cube := Cube()

	solidHeader := binaryFile(cube, "")
	copy(solidHeader, "solid binary file whose header starts with solid")

	truncated := binaryFile(cube, "mtilt fixture: truncated")
	truncated = truncated[:len(truncated)-20]

	flipped := append([][3]r3.Vec(nil), cube...)
	flipped[0] = [3]r3.Vec{flipped[0][0], flipped[0][2], flipped[0][1]}

	open := append([][3]r3.Vec(nil), cube[1:]...)

	twoCubes := append([][3]r3.Vec(nil), cube...)
	twoCubes = append(twoCubes, Box(r3.NewVec(40, 0, 0), r3.NewVec(60, 20, 20))...)

	nonFinite := strings.Replace(ASCII("nonfinite", cube), "vertex 0 0 0", "vertex nan 0 0", 1)

	malformed := strings.Replace(ASCII("malformed", cube), "outer loop", "outer lop", 1)

	return []File{
		{Name: "cube.stl", Data: []byte(ASCII("cube", cube))},
		{Name: "cube-split-faces.stl", Data: []byte(ASCII("cube-split-faces", CubeSplitFaces()))},
		{Name: "oblique-cuboid.stl", Data: binaryFile(ObliqueCuboid(), "mtilt fixture: oblique cuboid")},
		{Name: "bracket.stl", Data: binaryFile(Bracket(), "mtilt fixture: bracket")},
		{Name: "bridge.stl", Data: binaryFile(Bridge(), "mtilt fixture: bridge")},
		{Name: "occluded.stl", Data: binaryFile(Occluded(), "mtilt fixture: occluded overhang")},
		{Name: "solid-header-binary.stl", Data: solidHeader},
		{Name: "truncated-binary.stl", Data: truncated},
		{Name: "malformed-ascii.stl", Data: []byte(malformed)},
		{Name: "nonfinite-ascii.stl", Data: []byte(nonFinite)},
		{Name: "inconsistent-winding.stl", Data: []byte(ASCII("inconsistent-winding", flipped))},
		{Name: "open-mesh.stl", Data: []byte(ASCII("open-mesh", open))},
		{Name: "two-components.stl", Data: []byte(ASCII("two-components", twoCubes))},
	}
}
