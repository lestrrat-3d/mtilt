package cli

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
)

// Paths inside an assembly directory.
const (
	modelFile   = "model.stl"
	supportsDir = "supports"
	planFile    = "plan.json"
)

// errOutputExists is returned when --out names an existing path.
var errOutputExists = errors.New("output path already exists")

func runPrepare(ctx context.Context, args []string, stderr io.Writer) int {
	started := time.Now()
	fs := newFlagSet("prepare", stderr)
	var c common
	c.register(fs)
	out := fs.String("out", "", "assembly directory to create; must not exist (required)")
	path, err := parse(fs, args)
	if err != nil {
		if errors.Is(err, flag.ErrHelp) {
			return ExitOK
		}
		fmt.Fprintf(stderr, "mtilt prepare: %v\n", err)
		return ExitUsage
	}
	if *out == "" {
		fmt.Fprintln(stderr, "mtilt prepare: --out is required")
		return ExitUsage
	}
	opts, err := c.options(true)
	if err != nil {
		fmt.Fprintf(stderr, "mtilt prepare: %v\n", err)
		return ExitUsage
	}
	if _, err := os.Lstat(*out); err == nil {
		fmt.Fprintf(stderr, "mtilt prepare: %s: %v; refusing to overwrite\n", *out, errOutputExists)
		return ExitUsage
	}
	in, err := readInput(path, mtilt.Unit(c.units))
	if err != nil {
		return fail(stderr, err)
	}
	res, err := mtilt.Prepare(ctx, in, opts)
	if err != nil {
		return fail(stderr, err)
	}
	if err := writeAssembly(ctx, *out, res); err != nil {
		return fail(stderr, err)
	}

	sel := res.Report.Candidates[*res.Report.Selected]
	fmt.Fprintf(stderr, "mtilt: prepared %s: candidate %d (%s), %d support bodies, height %.3f mm\n",
		*out, sel.ID, sel.Source, len(res.Supports), sel.Metrics.HeightMM)
	for _, w := range res.Report.Warnings {
		fmt.Fprintf(stderr, "mtilt: warning: %s\n", w)
	}
	// Timing is not reproducible, so it goes to stderr and never into
	// plan.json.
	fmt.Fprintf(stderr, "mtilt: elapsed %s\n", time.Since(started).Round(time.Millisecond))
	return ExitOK
}

// writeAssembly writes the assembly into a staging directory next to out,
// reads every mesh back and re-validates it, writes plan.json, and only then
// renames the staging directory to out. On any failure the staging
// directory is removed and out is not created.
func writeAssembly(ctx context.Context, out string, res *mtilt.Result) error {
	parent, base := filepath.Split(filepath.Clean(out))
	if parent == "" {
		parent = "."
	}
	staging, err := os.MkdirTemp(parent, "."+base+".partial-*")
	if err != nil {
		return fmt.Errorf("creating staging directory: %w", err)
	}
	if err := stageAssembly(ctx, staging, res); err != nil {
		os.RemoveAll(staging)
		return err
	}
	// The existence check is repeated here because rename(2) replaces an
	// empty directory that appeared since the first check.
	if _, err := os.Lstat(out); err == nil {
		os.RemoveAll(staging)
		return fmt.Errorf("%s: %w", out, errOutputExists)
	}
	if err := os.Rename(staging, out); err != nil {
		os.RemoveAll(staging)
		return err
	}
	return nil
}

func stageAssembly(ctx context.Context, staging string, res *mtilt.Result) error {
	rep := res.Report
	model := *rep.Model
	model.File = modelFile
	rep.Model = &model
	if err := writeSTL(filepath.Join(staging, modelFile), res.Model, "mtilt model, millimeters"); err != nil {
		return err
	}
	if err := os.Mkdir(filepath.Join(staging, supportsDir), 0o755); err != nil {
		return err
	}
	rep.Supports = append([]mtilt.SupportReport(nil), rep.Supports...)
	for i := range rep.Supports {
		rel := filepath.ToSlash(filepath.Join(supportsDir, rep.Supports[i].ID+".stl"))
		rep.Supports[i].File = rel
		if err := writeSTL(filepath.Join(staging, rel), res.Supports[i], "mtilt "+rep.Supports[i].ID+", millimeters"); err != nil {
			return err
		}
	}

	reread, err := readBack(filepath.Join(staging, modelFile))
	if err != nil {
		return err
	}
	supports := make([]*mesh.Mesh, len(rep.Supports))
	for i, s := range rep.Supports {
		if supports[i], err = readBack(filepath.Join(staging, s.File)); err != nil {
			return err
		}
	}
	checks, err := mtilt.ValidateSerialized(ctx, res, reread, supports)
	if err != nil {
		return err
	}
	rep.Validation = append(append([]mesh.Check(nil), rep.Validation...), checks...)
	for _, c := range checks {
		if c.Status == mesh.StatusFailed {
			return fmt.Errorf("written files failed check %s: %s", c.Name, c.Detail)
		}
	}

	var buf bytes.Buffer
	enc := json.NewEncoder(&buf)
	enc.SetIndent("", "  ")
	if err := enc.Encode(rep); err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(staging, planFile), buf.Bytes(), 0o644); err != nil {
		return err
	}
	return ctx.Err()
}

func writeSTL(path string, m *mesh.Mesh, header string) error {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0o644)
	if err != nil {
		return err
	}
	if err := stl.WriteBinary(f, m.Soup(), header); err != nil {
		f.Close()
		return fmt.Errorf("%s: %w", path, err)
	}
	return f.Close()
}

// readBack reads a written STL file with exact-position vertex merging, the
// same way an input file is read.
func readBack(path string) (*mesh.Mesh, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	file, err := stl.Read(f, stl.Limits{})
	if err != nil {
		return nil, fmt.Errorf("re-reading %s: %w", path, err)
	}
	return mesh.FromSoup(file.Triangles)
}
