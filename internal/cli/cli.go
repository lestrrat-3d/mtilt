// Package cli implements the mtilt command line: argument parsing, file
// reading and the staged writing of a prepared assembly. All geometry work
// is done by package mtilt.
package cli

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"strings"

	"github.com/lestrrat-3d/mtilt"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/mtilt/stl"
)

// Exit codes.
const (
	ExitOK = 0
	// ExitFailure: preparation ran and found no feasible result, or the
	// output could not be written.
	ExitFailure = 1
	// ExitUsage: invalid command, flags or profile.
	ExitUsage = 2
	// ExitInput: the input file is unreadable, malformed, invalid or
	// outside the supported topology.
	ExitInput = 3
	// ExitCancelled: interrupted before completion.
	ExitCancelled = 130
)

const usage = `usage:
  mtilt analyze --units UNIT [--profile FILE] [--json] [--keep-orientation] MODEL.stl
  mtilt prepare --units UNIT --profile FILE --out DIR [--keep-orientation] MODEL.stl
  mtilt version

UNIT is the length unit of the STL coordinates: mm, cm, m or in.
`

// Run executes one mtilt command and returns its exit code. Diagnostics go
// to stderr; machine-readable output goes to stdout.
func Run(ctx context.Context, args []string, stdout, stderr io.Writer) int {
	if len(args) == 0 {
		fmt.Fprint(stderr, usage)
		return ExitUsage
	}
	switch args[0] {
	case "analyze":
		return runAnalyze(ctx, args[1:], stdout, stderr)
	case "prepare":
		return runPrepare(ctx, args[1:], stderr)
	case "version":
		fmt.Fprintf(stdout, "mtilt %s (%s)\n", mtilt.Version, mtilt.AlgorithmVersion)
		return ExitOK
	case "help", "-h", "--help":
		fmt.Fprint(stdout, usage)
		return ExitOK
	}
	fmt.Fprintf(stderr, "mtilt: unknown command %q\n%s", args[0], usage)
	return ExitUsage
}

// common holds the flags analyze and prepare share.
type common struct {
	units   string
	profile string
	keep    bool
}

func (c *common) register(fs *flag.FlagSet) {
	fs.StringVar(&c.units, "units", "", "length unit of the STL coordinates: mm, cm, m or in (required)")
	fs.StringVar(&c.profile, "profile", "", "printer/process profile JSON file")
	fs.BoolVar(&c.keep, "keep-orientation", false, "keep the input rotation; only move the model onto the plate")
}

func newFlagSet(name string, stderr io.Writer) *flag.FlagSet {
	fs := flag.NewFlagSet(name, flag.ContinueOnError)
	fs.SetOutput(stderr)
	return fs
}

// parse parses flags and requires exactly one positional argument after
// them.
func parse(fs *flag.FlagSet, args []string) (string, error) {
	if err := fs.Parse(args); err != nil {
		return "", err
	}
	rest := fs.Args()
	if len(rest) != 1 {
		return "", fmt.Errorf("expected exactly one input file after the flags, got %q", rest)
	}
	return rest[0], nil
}

func (c *common) options(requireProfile bool) (mtilt.Options, error) {
	if c.units == "" {
		return mtilt.Options{}, errors.New("--units is required: STL files carry no unit")
	}
	if _, ok := mtilt.Unit(c.units).ToMillimeters(); !ok {
		return mtilt.Options{}, fmt.Errorf("--units %q: use mm, cm, m or in", c.units)
	}
	opts := mtilt.Options{KeepOrientation: c.keep}
	if c.profile == "" {
		if requireProfile {
			return mtilt.Options{}, errors.New("--profile is required")
		}
		opts.Profile = mtilt.ExampleProfile()
		return opts, nil
	}
	f, err := os.Open(c.profile)
	if err != nil {
		return mtilt.Options{}, err
	}
	defer f.Close()
	if opts.Profile, err = mtilt.DecodeProfile(f); err != nil {
		return mtilt.Options{}, fmt.Errorf("%s: %w", c.profile, err)
	}
	return opts, nil
}

// readInput reads and decodes an STL file. The SHA-256 digest covers the
// exact bytes read.
func readInput(path string, unit mtilt.Unit) (mtilt.Input, error) {
	f, err := os.Open(path)
	if err != nil {
		return mtilt.Input{}, err
	}
	defer f.Close()
	h := sha256.New()
	file, err := stl.Read(io.TeeReader(f, h), stl.Limits{})
	if err != nil {
		return mtilt.Input{}, fmt.Errorf("%s: %w", path, err)
	}
	m, err := mesh.FromSoup(file.Triangles)
	if err != nil {
		return mtilt.Input{}, fmt.Errorf("%s: %w", path, err)
	}
	return mtilt.Input{
		Mesh: m,
		Unit: unit,
		Source: mtilt.Source{
			SHA256:             hex.EncodeToString(h.Sum(nil)),
			Format:             string(file.Format),
			Name:               file.Name,
			NormalsDisagreeing: file.NormalsDisagreeing,
		},
	}, nil
}

// exitCode maps an error from the library or file handling to an exit
// code.
func exitCode(err error) int {
	switch {
	case errors.Is(err, context.Canceled), errors.Is(err, context.DeadlineExceeded):
		return ExitCancelled
	case errors.Is(err, mtilt.ErrInvalidOptions):
		return ExitUsage
	case errors.Is(err, mtilt.ErrInvalidMesh), errors.Is(err, mtilt.ErrUnsupportedInput),
		errors.Is(err, mtilt.ErrLimit), errors.Is(err, mesh.ErrNonFinite),
		errors.Is(err, stl.ErrMalformed), errors.Is(err, stl.ErrTruncated),
		errors.Is(err, stl.ErrNonFinite), errors.Is(err, stl.ErrLimit),
		errors.Is(err, os.ErrNotExist), errors.Is(err, os.ErrPermission):
		return ExitInput
	}
	return ExitFailure
}

func fail(stderr io.Writer, err error) int {
	msg := err.Error()
	if !strings.HasPrefix(msg, "mtilt: ") {
		msg = "mtilt: " + msg
	}
	fmt.Fprintln(stderr, msg)
	var fe *mtilt.FailureError
	if errors.As(err, &fe) {
		printFailure(stderr, fe.Report)
	}
	return exitCode(err)
}

func printFailure(w io.Writer, r *mtilt.Report) {
	for _, c := range r.Validation {
		if c.Status == mesh.StatusFailed {
			fmt.Fprintf(w, "  check %s failed: %s\n", c.Name, c.Detail)
		}
	}
	for _, c := range r.Candidates {
		if c.Attempt.Status == mtilt.AttemptNotAttempted && c.Attempt.Reason == "" {
			continue
		}
		fmt.Fprintf(w, "  candidate %d (%s, rank %d): %s: %s\n", c.ID, c.Source, c.Rank, c.Attempt.Status, c.Attempt.Reason)
	}
	if r.Search.AttemptLimitReached {
		fmt.Fprintf(w, "  the support attempt limit (%d) stopped the search\n", r.Limits.MaxSupportAttempts)
	}
	if r.Search.CandidateLimitReached {
		fmt.Fprintf(w, "  the candidate limit (%d) dropped %d orientations\n", r.Limits.MaxCandidates, r.Search.CandidatesDistinct-r.Search.CandidatesEvaluated)
	}
}
