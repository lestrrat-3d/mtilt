// Package stl reads ASCII and binary STL files into float64 triangle soups,
// and writes binary STL files.
//
// The reader is bounded: it reads at most Limits.MaxBytes bytes and returns at
// most Limits.MaxTriangles triangles, and the triangle count a binary file
// declares never sizes an allocation before the file's length confirms it.
//
// STL carries no unit. The triangles Read returns are in whatever unit the
// file's author used; the caller must state that unit before treating the
// coordinates as lengths.
package stl

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/r3"
)

// Format is the encoding of an STL file.
type Format string

// The two STL encodings.
const (
	FormatASCII  Format = "ascii"
	FormatBinary Format = "binary"
)

// Errors Read returns. Each is wrapped with detail about where the input
// failed.
var (
	// ErrMalformed reports input that is neither valid binary nor valid
	// ASCII STL.
	ErrMalformed = errors.New("stl: malformed input")
	// ErrTruncated reports a binary file shorter than its declared triangle
	// count requires, or an ASCII file that ends inside a facet.
	ErrTruncated = errors.New("stl: truncated input")
	// ErrNonFinite reports a coordinate that is NaN, infinite, or outside
	// the float64 range.
	ErrNonFinite = errors.New("stl: non-finite coordinate")
	// ErrLimit reports input larger than a configured limit.
	ErrLimit = errors.New("stl: input exceeds limit")
)

const (
	headerSize   = 80
	countSize    = 4
	triangleSize = 50
)

// Limits bound the work Read does. A zero field means the default.
type Limits struct {
	// MaxBytes is the largest input Read accepts. Default 512 MiB.
	MaxBytes int64
	// MaxTriangles is the largest triangle count Read accepts. Default
	// 5,000,000.
	MaxTriangles int
}

// Default limits.
const (
	DefaultMaxBytes     = 512 << 20
	DefaultMaxTriangles = 5_000_000
)

func (l Limits) withDefaults() Limits {
	if l.MaxBytes <= 0 {
		l.MaxBytes = DefaultMaxBytes
	}
	if l.MaxTriangles <= 0 {
		l.MaxTriangles = DefaultMaxTriangles
	}
	return l
}

// File is a decoded STL file.
type File struct {
	Format Format
	// Name is the ASCII solid name, or the binary header with trailing
	// NUL bytes and spaces removed.
	Name string
	// Triangles holds each facet's three vertices in file order. Stored
	// facet normals are not kept.
	Triangles [][3]r3.Vec
	// NormalsDisagreeing counts facets whose stored normal is nonzero and
	// points against the normal computed from the vertex winding. mtilt
	// uses the computed normal; this count is diagnostic only.
	NormalsDisagreeing int
}

// Read decodes one STL file from r.
//
// The format is decided by size first: input of at least 84 bytes whose
// length equals 84 + 50 * (the little-endian count at byte 80) is binary,
// even when its header begins with "solid". Otherwise input that begins with
// "solid" (after optional whitespace, any letter case) is parsed as ASCII.
// Anything else of at least 84 bytes is a binary file whose length disagrees
// with its count: ErrTruncated when shorter, ErrMalformed when longer.
//
// ASCII input must hold exactly one solid; text after "endsolid" and its
// name is ErrMalformed. Coordinates are parsed as float64. A NaN or infinite
// coordinate, in either format, is ErrNonFinite.
func Read(r io.Reader, lim Limits) (*File, error) {
	lim = lim.withDefaults()
	data, err := io.ReadAll(io.LimitReader(r, lim.MaxBytes+1))
	if err != nil {
		return nil, fmt.Errorf("stl: reading input: %w", err)
	}
	if int64(len(data)) > lim.MaxBytes {
		return nil, fmt.Errorf("%w: input is larger than %d bytes", ErrLimit, lim.MaxBytes)
	}
	return Decode(data, lim)
}

// Decode decodes one STL file held in data. It applies the same rules as
// Read, except that the byte limit is not checked.
func Decode(data []byte, lim Limits) (*File, error) {
	lim = lim.withDefaults()
	if len(data) >= headerSize+countSize {
		count := int64(binary.LittleEndian.Uint32(data[headerSize:]))
		if int64(len(data)) == headerSize+countSize+triangleSize*count {
			return decodeBinary(data, int(count), lim)
		}
	}
	if hasSolidPrefix(data) {
		return decodeASCII(data, lim)
	}
	if len(data) < headerSize+countSize {
		return nil, fmt.Errorf("%w: %d bytes is too short for binary STL and does not start with \"solid\"", ErrMalformed, len(data))
	}
	count := int64(binary.LittleEndian.Uint32(data[headerSize:]))
	want := headerSize + countSize + triangleSize*count
	if int64(len(data)) < want {
		return nil, fmt.Errorf("%w: binary header declares %d triangles (%d bytes) but input has %d bytes", ErrTruncated, count, want, len(data))
	}
	return nil, fmt.Errorf("%w: binary header declares %d triangles (%d bytes) but input has %d bytes", ErrMalformed, count, want, len(data))
}

func hasSolidPrefix(data []byte) bool {
	trimmed := bytes.TrimLeft(data, " \t\r\n")
	return len(trimmed) >= 5 && bytes.EqualFold(trimmed[:5], []byte("solid"))
}

func decodeBinary(data []byte, count int, lim Limits) (*File, error) {
	if count > lim.MaxTriangles {
		return nil, fmt.Errorf("%w: %d triangles, limit is %d", ErrLimit, count, lim.MaxTriangles)
	}
	f := &File{
		Format:    FormatBinary,
		Name:      string(bytes.TrimRight(data[:headerSize], "\x00 ")),
		Triangles: make([][3]r3.Vec, count),
	}
	for i := range count {
		rec := data[headerSize+countSize+i*triangleSize:]
		var vals [12]float64
		for k := range 12 {
			vals[k] = float64(math.Float32frombits(binary.LittleEndian.Uint32(rec[4*k:])))
		}
		for k := range 3 {
			v := r3.NewVec(vals[3+3*k], vals[4+3*k], vals[5+3*k])
			if !finite(v) {
				return nil, fmt.Errorf("%w: triangle %d vertex %d is %v", ErrNonFinite, i, k, v)
			}
			f.Triangles[i][k] = v
		}
		if disagrees(r3.NewVec(vals[0], vals[1], vals[2]), f.Triangles[i]) {
			f.NormalsDisagreeing++
		}
	}
	return f, nil
}

// asciiScanner splits ASCII STL into whitespace-separated tokens and tracks
// the line number for error messages.
type asciiScanner struct {
	data []byte
	pos  int
	line int
}

func (s *asciiScanner) skipSpace() {
	for s.pos < len(s.data) {
		switch s.data[s.pos] {
		case '\n':
			s.line++
		case ' ', '\t', '\r', '\f', '\v':
		default:
			return
		}
		s.pos++
	}
}

func (s *asciiScanner) next() (string, bool) {
	s.skipSpace()
	if s.pos >= len(s.data) {
		return "", false
	}
	start := s.pos
	for s.pos < len(s.data) && !isSpace(s.data[s.pos]) {
		s.pos++
	}
	return string(s.data[start:s.pos]), true
}

// restOfLine returns the text up to the next newline, without surrounding
// spaces, and leaves the scanner at the newline.
func (s *asciiScanner) restOfLine() string {
	start := s.pos
	for s.pos < len(s.data) && s.data[s.pos] != '\n' {
		s.pos++
	}
	return string(bytes.TrimSpace(s.data[start:s.pos]))
}

func isSpace(b byte) bool {
	switch b {
	case ' ', '\t', '\r', '\n', '\f', '\v':
		return true
	}
	return false
}

func (s *asciiScanner) errorf(base error, format string, args ...any) error {
	return fmt.Errorf("%w: line %d: %s", base, s.line, fmt.Sprintf(format, args...))
}

func (s *asciiScanner) expect(kw string) error {
	tok, ok := s.next()
	if !ok {
		return s.errorf(ErrTruncated, "input ends where %q was expected", kw)
	}
	if !equalFold(tok, kw) {
		return s.errorf(ErrMalformed, "expected %q, found %q", kw, tok)
	}
	return nil
}

func (s *asciiScanner) vec() (r3.Vec, error) {
	var c [3]float64
	for k := range 3 {
		tok, ok := s.next()
		if !ok {
			return r3.Vec{}, s.errorf(ErrTruncated, "input ends inside a coordinate triple")
		}
		v, err := strconv.ParseFloat(tok, 64)
		if err != nil {
			var numErr *strconv.NumError
			if errors.As(err, &numErr) && errors.Is(numErr.Err, strconv.ErrRange) {
				return r3.Vec{}, s.errorf(ErrNonFinite, "coordinate %q is outside the float64 range", tok)
			}
			return r3.Vec{}, s.errorf(ErrMalformed, "coordinate %q is not a number", tok)
		}
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return r3.Vec{}, s.errorf(ErrNonFinite, "coordinate %q is not finite", tok)
		}
		c[k] = v
	}
	return r3.NewVec(c[0], c[1], c[2]), nil
}

func decodeASCII(data []byte, lim Limits) (*File, error) {
	s := &asciiScanner{data: data, line: 1}
	if err := s.expect("solid"); err != nil {
		return nil, err
	}
	f := &File{Format: FormatASCII, Name: s.restOfLine()}
	for {
		tok, ok := s.next()
		if !ok {
			return nil, s.errorf(ErrTruncated, "input ends before \"endsolid\"")
		}
		if equalFold(tok, "endsolid") {
			break
		}
		if !equalFold(tok, "facet") {
			return nil, s.errorf(ErrMalformed, "expected \"facet\" or \"endsolid\", found %q", tok)
		}
		if len(f.Triangles) == lim.MaxTriangles {
			return nil, fmt.Errorf("%w: more than %d triangles", ErrLimit, lim.MaxTriangles)
		}
		tri, normal, err := s.facet()
		if err != nil {
			return nil, err
		}
		if disagrees(normal, tri) {
			f.NormalsDisagreeing++
		}
		f.Triangles = append(f.Triangles, tri)
	}
	s.restOfLine() // the optional solid name after endsolid
	s.skipSpace()
	if s.pos < len(s.data) {
		return nil, s.errorf(ErrMalformed, "unexpected text after \"endsolid\"; only one solid per file is read")
	}
	return f, nil
}

func (s *asciiScanner) facet() ([3]r3.Vec, r3.Vec, error) {
	var tri [3]r3.Vec
	if err := s.expect("normal"); err != nil {
		return tri, r3.Vec{}, err
	}
	normal, err := s.vec()
	if err != nil {
		return tri, r3.Vec{}, err
	}
	if err := s.expect("outer"); err != nil {
		return tri, r3.Vec{}, err
	}
	if err := s.expect("loop"); err != nil {
		return tri, r3.Vec{}, err
	}
	for k := range 3 {
		if err := s.expect("vertex"); err != nil {
			return tri, r3.Vec{}, err
		}
		if tri[k], err = s.vec(); err != nil {
			return tri, r3.Vec{}, err
		}
	}
	if err := s.expect("endloop"); err != nil {
		return tri, r3.Vec{}, err
	}
	if err := s.expect("endfacet"); err != nil {
		return tri, r3.Vec{}, err
	}
	return tri, normal, nil
}

// disagrees reports whether a nonzero stored normal points against the normal
// the vertex winding gives.
func disagrees(stored r3.Vec, tri [3]r3.Vec) bool {
	if stored == (r3.Vec{}) {
		return false
	}
	computed := tri[1].Sub(tri[0]).Cross(tri[2].Sub(tri[0]))
	return stored.Dot(computed) < 0
}

func finite(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}

func equalFold(a, b string) bool {
	return strings.EqualFold(a, b)
}
