package stl

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/r3"
)

// WriteBinary writes tris as a binary STL file.
//
// header is copied into the 80-byte header and padded with spaces. It must
// be at most 80 bytes and must not begin with "solid" in any letter case:
// readers that sniff that prefix would take the file for ASCII.
//
// Each facet normal is computed from the vertex winding. Coordinates are
// rounded to float32, the only precision binary STL stores. A coordinate
// outside the float32 range is ErrNonFinite, and nothing past the failing
// triangle is written.
func WriteBinary(w io.Writer, tris [][3]r3.Vec, header string) error {
	if len(header) > headerSize {
		return fmt.Errorf("stl: header is %d bytes, limit is %d", len(header), headerSize)
	}
	if hasSolidPrefix([]byte(header)) {
		return fmt.Errorf("stl: binary header must not begin with \"solid\"")
	}
	if len(tris) > math.MaxUint32 {
		return fmt.Errorf("%w: %d triangles do not fit a binary STL count", ErrLimit, len(tris))
	}

	bw := bufio.NewWriter(w)
	hdr := bytes.Repeat([]byte{' '}, headerSize)
	copy(hdr, header)
	if _, err := bw.Write(hdr); err != nil {
		return err
	}
	var buf [triangleSize]byte
	binary.LittleEndian.PutUint32(buf[:4], uint32(len(tris)))
	if _, err := bw.Write(buf[:4]); err != nil {
		return err
	}
	for i, t := range tris {
		n, ok := t[1].Sub(t[0]).Cross(t[2].Sub(t[0])).Normalize()
		if !ok {
			n = r3.Vec{}
		}
		vals := [12]float64{n.X, n.Y, n.Z, t[0].X, t[0].Y, t[0].Z, t[1].X, t[1].Y, t[1].Z, t[2].X, t[2].Y, t[2].Z}
		for k, v := range vals {
			f := float32(v)
			if math.IsInf(float64(f), 0) || math.IsNaN(float64(f)) {
				return fmt.Errorf("%w: triangle %d value %g does not fit float32", ErrNonFinite, i, v)
			}
			binary.LittleEndian.PutUint32(buf[4*k:], math.Float32bits(f))
		}
		buf[48], buf[49] = 0, 0
		if _, err := bw.Write(buf[:]); err != nil {
			return err
		}
	}
	return bw.Flush()
}
