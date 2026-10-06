package stl_test

import (
	"bytes"
	"encoding/binary"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/lestrrat-3d/mtilt/stl"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func readFixture(t *testing.T, name string) []byte {
	t.Helper()
	data, err := os.ReadFile(filepath.Join("..", "testdata", name))
	require.NoError(t, err)
	return data
}

func TestRead(t *testing.T) {
	t.Run("ascii keeps float64 coordinates", func(t *testing.T) {
		text := fixture.ASCII("precise", [][3]r3.Vec{{
			r3.NewVec(0.1, 0.2, 0.30000000000000004), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		}})
		f, err := stl.Read(strings.NewReader(text), stl.Limits{})
		require.NoError(t, err)
		require.Equal(t, stl.FormatASCII, f.Format)
		require.Equal(t, "precise", f.Name)
		require.Equal(t, 0.30000000000000004, f.Triangles[0][0].Z)
	})

	t.Run("binary round trip rounds to float32 only", func(t *testing.T) {
		tris := fixture.ObliqueCuboid()
		var buf bytes.Buffer
		require.NoError(t, stl.WriteBinary(&buf, tris, "round trip"))
		f, err := stl.Read(&buf, stl.Limits{})
		require.NoError(t, err)
		require.Equal(t, stl.FormatBinary, f.Format)
		require.Equal(t, "round trip", f.Name)
		require.Len(t, f.Triangles, len(tris))
		for i := range tris {
			for k := range 3 {
				require.True(t, tris[i][k].Equal(f.Triangles[i][k], 1e-5))
			}
		}
	})

	t.Run("binary file whose header starts with solid is read as binary", func(t *testing.T) {
		data := readFixture(t, "solid-header-binary.stl")
		require.True(t, bytes.HasPrefix(data, []byte("solid")))
		f, err := stl.Decode(data, stl.Limits{})
		require.NoError(t, err)
		require.Equal(t, stl.FormatBinary, f.Format)
		require.Len(t, f.Triangles, 12)
	})

	errorCases := []struct {
		file string
		want error
	}{
		{"truncated-binary.stl", stl.ErrTruncated},
		{"malformed-ascii.stl", stl.ErrMalformed},
		{"nonfinite-ascii.stl", stl.ErrNonFinite},
	}
	for _, tc := range errorCases {
		t.Run(tc.file, func(t *testing.T) {
			_, err := stl.Decode(readFixture(t, tc.file), stl.Limits{})
			require.ErrorIs(t, err, tc.want)
		})
	}

	t.Run("non-finite binary coordinate", func(t *testing.T) {
		var buf bytes.Buffer
		require.NoError(t, stl.WriteBinary(&buf, fixture.Cube(), "nan"))
		data := buf.Bytes()
		binary.LittleEndian.PutUint32(data[84+12:], 0x7fc00000) // NaN in triangle 0, vertex 0, X
		_, err := stl.Decode(data, stl.Limits{})
		require.ErrorIs(t, err, stl.ErrNonFinite)
	})

	t.Run("ascii coordinate out of float64 range", func(t *testing.T) {
		text := strings.Replace(fixture.ASCII("big", fixture.Cube()), "vertex 0 0 0", "vertex 1e999 0 0", 1)
		_, err := stl.Decode([]byte(text), stl.Limits{})
		require.ErrorIs(t, err, stl.ErrNonFinite)
	})

	t.Run("binary longer than its count", func(t *testing.T) {
		data := append(readFixture(t, "oblique-cuboid.stl"), 0, 0, 0)
		_, err := stl.Decode(data, stl.Limits{})
		require.ErrorIs(t, err, stl.ErrMalformed)
	})

	t.Run("declared count is never trusted for allocation", func(t *testing.T) {
		data := make([]byte, 84)
		binary.LittleEndian.PutUint32(data[80:], 0xffffffff)
		_, err := stl.Decode(data, stl.Limits{})
		require.ErrorIs(t, err, stl.ErrTruncated)
	})

	t.Run("too short", func(t *testing.T) {
		_, err := stl.Decode([]byte("abc"), stl.Limits{})
		require.ErrorIs(t, err, stl.ErrMalformed)
	})

	t.Run("text after endsolid", func(t *testing.T) {
		text := fixture.ASCII("a", fixture.Cube()) + fixture.ASCII("b", fixture.Cube())
		_, err := stl.Decode([]byte(text), stl.Limits{})
		require.ErrorIs(t, err, stl.ErrMalformed)
	})

	t.Run("ascii ends inside a facet", func(t *testing.T) {
		text := fixture.ASCII("cut", fixture.Cube())
		_, err := stl.Decode([]byte(text[:strings.Index(text, "endloop")]), stl.Limits{})
		require.ErrorIs(t, err, stl.ErrTruncated)
	})

	t.Run("byte limit", func(t *testing.T) {
		data := readFixture(t, "cube.stl")
		_, err := stl.Read(bytes.NewReader(data), stl.Limits{MaxBytes: int64(len(data) - 1)})
		require.ErrorIs(t, err, stl.ErrLimit)
		_, err = stl.Read(bytes.NewReader(data), stl.Limits{MaxBytes: int64(len(data))})
		require.NoError(t, err)
	})

	t.Run("triangle limit", func(t *testing.T) {
		for _, name := range []string{"cube.stl", "solid-header-binary.stl"} {
			_, err := stl.Decode(readFixture(t, name), stl.Limits{MaxTriangles: 11})
			require.ErrorIs(t, err, stl.ErrLimit, name)
			_, err = stl.Decode(readFixture(t, name), stl.Limits{MaxTriangles: 12})
			require.NoError(t, err, name)
		}
	})

	t.Run("stored normals that disagree with winding are counted", func(t *testing.T) {
		text := fixture.ASCII("flipped-normal", fixture.Cube())
		i := strings.Index(text, "facet normal ") + len("facet normal ")
		j := i + strings.Index(text[i:], "\n")
		stored := strings.Fields(text[i:j])
		negated := make([]string, 3)
		for k, v := range stored {
			if strings.HasPrefix(v, "-") {
				negated[k] = v[1:]
			} else {
				negated[k] = "-" + v
			}
		}
		text = text[:i] + strings.Join(negated, " ") + text[j:]
		f, err := stl.Decode([]byte(text), stl.Limits{})
		require.NoError(t, err)
		require.Equal(t, 1, f.NormalsDisagreeing)
	})
}

func TestWriteBinary(t *testing.T) {
	t.Run("refuses a header starting with solid", func(t *testing.T) {
		require.Error(t, stl.WriteBinary(&bytes.Buffer{}, fixture.Cube(), "solid cube"))
	})
	t.Run("refuses a header over 80 bytes", func(t *testing.T) {
		require.Error(t, stl.WriteBinary(&bytes.Buffer{}, fixture.Cube(), strings.Repeat("x", 81)))
	})
	t.Run("refuses coordinates outside float32", func(t *testing.T) {
		tris := fixture.Cube()
		tris[0][0] = r3.NewVec(1e300, 0, 0)
		require.ErrorIs(t, stl.WriteBinary(&bytes.Buffer{}, tris, "big"), stl.ErrNonFinite)
	})
	t.Run("writes normals from winding", func(t *testing.T) {
		var buf bytes.Buffer
		tri := [3]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0)}
		require.NoError(t, stl.WriteBinary(&buf, [][3]r3.Vec{tri}, "one"))
		data := buf.Bytes()
		require.Len(t, data, 84+50)
		normal := [3]float32{}
		for k := range 3 {
			normal[k] = math.Float32frombits(binary.LittleEndian.Uint32(data[84+4*k:]))
		}
		require.Equal(t, [3]float32{0, 0, -1}, normal)
	})
}

func FuzzDecode(f *testing.F) {
	entries, err := os.ReadDir(filepath.Join("..", "testdata"))
	if err != nil {
		f.Fatal(err)
	}
	for _, e := range entries {
		data, err := os.ReadFile(filepath.Join("..", "testdata", e.Name()))
		if err != nil {
			f.Fatal(err)
		}
		f.Add(data)
	}
	f.Fuzz(func(t *testing.T, data []byte) {
		file, err := stl.Decode(data, stl.Limits{MaxTriangles: 10_000})
		if err != nil {
			return
		}
		require.LessOrEqual(t, len(file.Triangles), 10_000)
		for _, tri := range file.Triangles {
			for _, v := range tri {
				require.False(t, v.X != v.X || v.Y != v.Y || v.Z != v.Z, "NaN accepted")
			}
		}
	})
}
