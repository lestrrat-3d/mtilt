package mtilt_test

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func loadFixture(t *testing.T, name string) *mesh.Mesh {
	t.Helper()
	f, err := os.Open(filepath.Join("testdata", name))
	require.NoError(t, err)
	defer f.Close()
	file, err := stl.Read(f, stl.Limits{})
	require.NoError(t, err)
	m, err := mesh.FromSoup(file.Triangles)
	require.NoError(t, err)
	return m
}

func soupMesh(t *testing.T, soup [][3]r3.Vec) *mesh.Mesh {
	t.Helper()
	m, err := mesh.FromSoup(soup)
	require.NoError(t, err)
	return m
}

func prepare(t *testing.T, m *mesh.Mesh, opts mtilt.Options) (*mtilt.Result, error) {
	t.Helper()
	if opts.Profile.Name == "" {
		opts.Profile = mtilt.ExampleProfile()
	}
	return mtilt.Prepare(t.Context(), mtilt.Input{Mesh: m, Unit: mtilt.UnitMillimeter}, opts)
}
