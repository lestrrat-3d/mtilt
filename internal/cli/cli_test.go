package cli_test

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"slices"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/cli"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
	"github.com/stretchr/testify/require"
)

// binary is the mtilt executable built once for the whole test run.
var binary string

func TestMain(m *testing.M) {
	dir, err := os.MkdirTemp("", ".tmp-mtilt-cli-*")
	if err != nil {
		panic(err)
	}
	binary = filepath.Join(dir, "mtilt")
	if runtime.GOOS == "windows" {
		binary += ".exe"
	}
	build := exec.CommandContext(context.Background(), "go", "build", "-o", binary, "github.com/lestrrat-3d/mtilt/cmd/mtilt")
	build.Stderr = os.Stderr
	if err := build.Run(); err != nil {
		os.RemoveAll(dir)
		panic(err)
	}
	code := m.Run()
	os.RemoveAll(dir)
	os.Exit(code)
}

const (
	prepareCmd = "prepare"
	unitsFlag  = "--units"
)

var (
	testdata = filepath.Join("..", "..", "testdata")
	profile  = filepath.Join("..", "..", "profiles", "example-fdm.json")
)

type run struct {
	code           int
	stdout, stderr string
}

func mtilt(t *testing.T, args ...string) run {
	t.Helper()
	cmd := exec.CommandContext(t.Context(), binary, args...)
	var stdout, stderr bytes.Buffer
	cmd.Stdout, cmd.Stderr = &stdout, &stderr
	err := cmd.Run()
	code := 0
	var exitErr *exec.ExitError
	if errors.As(err, &exitErr) {
		code = exitErr.ExitCode()
	} else {
		require.NoError(t, err)
	}
	return run{code: code, stdout: stdout.String(), stderr: stderr.String()}
}

func fixture(name string) string { return filepath.Join(testdata, name) }

func digest(t *testing.T, path string) [32]byte {
	t.Helper()
	data, err := os.ReadFile(path)
	require.NoError(t, err)
	return sha256.Sum256(data)
}

// entries lists the names in dir.
func entries(t *testing.T, dir string) []string {
	t.Helper()
	des, err := os.ReadDir(dir)
	require.NoError(t, err)
	var out []string
	for _, d := range des {
		out = append(out, d.Name())
	}
	return out
}

type plan struct {
	SchemaVersion int `json:"schema_version"`
	Input         struct {
		SHA256 string `json:"sha256"`
		Unit   string `json:"unit"`
	} `json:"input"`
	Selected *int `json:"selected_candidate"`
	Model    struct {
		File string `json:"file"`
	} `json:"model"`
	Supports []struct {
		ID   string `json:"id"`
		File string `json:"file"`
	} `json:"supports"`
	Validation []mesh.Check `json:"validation"`
}

func readPlan(t *testing.T, dir string) plan {
	t.Helper()
	data, err := os.ReadFile(filepath.Join(dir, "plan.json"))
	require.NoError(t, err)
	var p plan
	require.NoError(t, json.Unmarshal(data, &p))
	return p
}

func readMesh(t *testing.T, path string) *mesh.Mesh {
	t.Helper()
	f, err := os.Open(path)
	require.NoError(t, err)
	defer f.Close()
	file, err := stl.Read(f, stl.Limits{})
	require.NoError(t, err)
	m, err := mesh.FromSoup(file.Triangles)
	require.NoError(t, err)
	return m
}

func TestAnalyze(t *testing.T) {
	t.Run("json", func(t *testing.T) {
		r := mtilt(t, "analyze", unitsFlag, "mm", "--json", fixture("oblique-cuboid.stl"))
		require.Equal(t, cli.ExitOK, r.code, r.stderr)
		var out struct {
			Analysis struct {
				Input struct {
					Triangles int    `json:"triangles"`
					Format    string `json:"format"`
				} `json:"input"`
				Candidates []struct {
					Source string `json:"source"`
				} `json:"candidates"`
			} `json:"analysis"`
			Ranking []int `json:"ranking"`
		}
		require.NoError(t, json.Unmarshal([]byte(r.stdout), &out))
		require.Equal(t, 12, out.Analysis.Input.Triangles)
		require.Equal(t, "binary", out.Analysis.Input.Format)
		require.Equal(t, "planar_face", out.Analysis.Candidates[out.Ranking[0]].Source)
		require.Contains(t, r.stderr, "built-in example profile")
	})

	t.Run("text", func(t *testing.T) {
		r := mtilt(t, "analyze", unitsFlag, "mm", "--profile", profile, fixture("bracket.stl"))
		require.Equal(t, cli.ExitOK, r.code, r.stderr)
		require.Contains(t, r.stdout, "passed    input_closed")
		require.Contains(t, r.stdout, "unchecked input_self_intersection")
		require.Contains(t, r.stdout, "candidates")
	})

	t.Run("invalid mesh reports checks and exits 3", func(t *testing.T) {
		r := mtilt(t, "analyze", unitsFlag, "mm", fixture("open-mesh.stl"))
		require.Equal(t, cli.ExitInput, r.code)
		require.Contains(t, r.stdout, "failed    input_closed")
		require.Contains(t, r.stderr, "invalid mesh")
	})

	for _, name := range []string{"truncated-binary.stl", "malformed-ascii.stl", "nonfinite-ascii.stl", "two-components.stl"} {
		t.Run(name+" exits 3", func(t *testing.T) {
			r := mtilt(t, "analyze", unitsFlag, "mm", fixture(name))
			require.Equal(t, cli.ExitInput, r.code, r.stderr)
			require.NotEmpty(t, r.stderr)
		})
	}

	t.Run("binary with a solid header", func(t *testing.T) {
		r := mtilt(t, "analyze", unitsFlag, "mm", "--json", fixture("solid-header-binary.stl"))
		require.Equal(t, cli.ExitOK, r.code, r.stderr)
		require.Contains(t, r.stdout, `"format": "binary"`)
	})

	t.Run("usage errors exit 2", func(t *testing.T) {
		for _, args := range [][]string{
			{"analyze", fixture("cube.stl")},
			{"analyze", unitsFlag, "furlong", fixture("cube.stl")},
			{"analyze", unitsFlag, "mm"},
			{"analyze", "--bogus", fixture("cube.stl")},
			{"frobnicate"},
			{},
		} {
			r := mtilt(t, args...)
			require.Equal(t, cli.ExitUsage, r.code, "%q", args)
			require.NotEmpty(t, r.stderr, "%q", args)
		}
	})

	t.Run("missing file exits 3", func(t *testing.T) {
		r := mtilt(t, "analyze", unitsFlag, "mm", fixture("no-such-file.stl"))
		require.Equal(t, cli.ExitInput, r.code)
	})
}

func TestPrepare(t *testing.T) {
	t.Run("bracket with kept orientation writes a validated assembly", func(t *testing.T) {
		parent := t.TempDir()
		out := filepath.Join(parent, "result")
		input := fixture("bracket.stl")
		before := digest(t, input)

		r := mtilt(t, prepareCmd, unitsFlag, "mm", "--profile", profile, "--keep-orientation", "--out", out, input)
		require.Equal(t, cli.ExitOK, r.code, r.stderr)
		require.Empty(t, r.stdout)
		require.Contains(t, r.stderr, "support bodies")
		require.Equal(t, before, digest(t, input), "input file changed")
		require.Equal(t, []string{"result"}, entries(t, parent), "staging directory left behind")
		require.Equal(t, []string{"model.stl", "plan.json", "supports"}, entries(t, out))

		p := readPlan(t, out)
		require.Equal(t, 1, p.SchemaVersion)
		require.Equal(t, "mm", p.Input.Unit)
		require.Len(t, p.Input.SHA256, 64)
		require.Equal(t, "model.stl", p.Model.File)
		require.NotEmpty(t, p.Supports)
		var listed []string
		for _, s := range p.Supports {
			listed = append(listed, filepath.Base(s.File))
			require.Equal(t, "supports/"+s.ID+".stl", s.File)
		}
		require.Equal(t, listed, entries(t, filepath.Join(out, "supports")))
		for _, c := range p.Validation {
			require.NotEqual(t, mesh.StatusFailed, c.Status, "%s: %s", c.Name, c.Detail)
		}
		require.True(t, slices.ContainsFunc(p.Validation, func(c mesh.Check) bool { return c.Name == "serialized_model_matches" }))

		// The written model and supports share one frame: every pillar
		// stands on the plate under the model's arm.
		model := readMesh(t, filepath.Join(out, "model.stl"))
		mb := model.Bounds()
		require.Zero(t, mb.Min.Z)
		for _, s := range p.Supports {
			b := readMesh(t, filepath.Join(out, s.File)).Bounds()
			require.InDelta(t, 0, b.Min.Z, 1e-6)
			require.Less(t, b.Max.Z, mb.Max.Z)
			require.GreaterOrEqual(t, b.Min.X, mb.Min.X)
		}

		t.Run("refuses to overwrite", func(t *testing.T) {
			planBefore := digest(t, filepath.Join(out, "plan.json"))
			r := mtilt(t, prepareCmd, unitsFlag, "mm", "--profile", profile, "--keep-orientation", "--out", out, input)
			require.Equal(t, cli.ExitUsage, r.code)
			require.Contains(t, r.stderr, "refusing to overwrite")
			require.Equal(t, planBefore, digest(t, filepath.Join(out, "plan.json")))
		})
	})

	t.Run("oblique cuboid is reoriented and needs no supports", func(t *testing.T) {
		out := filepath.Join(t.TempDir(), "result")
		r := mtilt(t, prepareCmd, unitsFlag, "mm", "--profile", profile, "--out", out, fixture("oblique-cuboid.stl"))
		require.Equal(t, cli.ExitOK, r.code, r.stderr)
		p := readPlan(t, out)
		require.NotNil(t, p.Selected)
		require.NotZero(t, *p.Selected)
		require.Empty(t, p.Supports)
		require.Empty(t, entries(t, filepath.Join(out, "supports")))
		require.InDelta(t, 10, readMesh(t, filepath.Join(out, "model.stl")).Bounds().Size().Z, 1e-4)
	})

	t.Run("occluded overhang fails and writes nothing", func(t *testing.T) {
		parent := t.TempDir()
		out := filepath.Join(parent, "result")
		r := mtilt(t, prepareCmd, unitsFlag, "mm", "--profile", profile, "--keep-orientation", "--out", out, fixture("occluded.stl"))
		require.Equal(t, cli.ExitFailure, r.code)
		require.Contains(t, r.stderr, "no feasible orientation")
		require.Contains(t, r.stderr, "occluded")
		require.Empty(t, entries(t, parent))
	})

	t.Run("invalid input writes nothing", func(t *testing.T) {
		parent := t.TempDir()
		r := mtilt(t, prepareCmd, unitsFlag, "mm", "--profile", profile, "--out", filepath.Join(parent, "result"), fixture("inconsistent-winding.stl"))
		require.Equal(t, cli.ExitInput, r.code)
		require.Contains(t, r.stderr, "consistent_winding")
		require.Empty(t, entries(t, parent))
	})

	t.Run("usage errors exit 2", func(t *testing.T) {
		dir := t.TempDir()
		bad := filepath.Join(dir, "bad.json")
		require.NoError(t, os.WriteFile(bad, []byte(`{"name": "x"}`), 0o644))
		out := filepath.Join(dir, "result")
		for _, args := range [][]string{
			{prepareCmd, unitsFlag, "mm", "--out", out, fixture("cube.stl")},
			{prepareCmd, unitsFlag, "mm", "--profile", profile, fixture("cube.stl")},
			{prepareCmd, "--profile", profile, "--out", out, fixture("cube.stl")},
			{prepareCmd, unitsFlag, "mm", "--profile", bad, "--out", out, fixture("cube.stl")},
		} {
			r := mtilt(t, args...)
			require.Equal(t, cli.ExitUsage, r.code, "%q: %s", args, r.stderr)
		}
		require.Equal(t, []string{"bad.json"}, entries(t, dir))
	})
}

func TestRunCancelled(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	var stdout, stderr bytes.Buffer
	out := filepath.Join(t.TempDir(), "result")
	code := cli.Run(ctx, []string{prepareCmd, unitsFlag, "mm", "--profile", profile, "--out", out, fixture("bracket.stl")}, &stdout, &stderr)
	require.Equal(t, cli.ExitCancelled, code, stderr.String())
	_, err := os.Stat(out)
	require.ErrorIs(t, err, os.ErrNotExist)
}

func TestVersion(t *testing.T) {
	r := mtilt(t, "version")
	require.Equal(t, cli.ExitOK, r.code)
	require.Contains(t, r.stdout, "mtilt ")
}
