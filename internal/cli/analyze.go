package cli

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"

	"github.com/lestrrat-3d/mtilt"
)

// maxListedCandidates caps the candidates the text output of analyze lists.
const maxListedCandidates = 10

func runAnalyze(ctx context.Context, args []string, stdout, stderr io.Writer) int {
	fs := newFlagSet("analyze", stderr)
	var c common
	c.register(fs)
	asJSON := fs.Bool("json", false, "write the analysis as JSON to stdout")
	path, err := parse(fs, args)
	if err != nil {
		if errors.Is(err, flag.ErrHelp) {
			return ExitOK
		}
		fmt.Fprintf(stderr, "mtilt analyze: %v\n", err)
		return ExitUsage
	}
	opts, err := c.options(false)
	if err != nil {
		fmt.Fprintf(stderr, "mtilt analyze: %v\n", err)
		return ExitUsage
	}
	if c.profile == "" {
		fmt.Fprintf(stderr, "mtilt analyze: no --profile given; using the built-in example profile (uncalibrated) for the overhang threshold\n")
	}
	in, err := readInput(path, mtilt.Unit(c.units))
	if err != nil {
		return fail(stderr, err)
	}
	a, err := mtilt.Analyze(ctx, in, opts)
	if a == nil {
		return fail(stderr, err)
	}
	if *asJSON {
		enc := json.NewEncoder(stdout)
		enc.SetIndent("", "  ")
		if encErr := enc.Encode(analysisJSON{Report: a.Report, Ranking: a.Ranking}); encErr != nil {
			return fail(stderr, encErr)
		}
	} else {
		printAnalysis(stdout, path, a)
	}
	if err != nil {
		return fail(stderr, err)
	}
	return ExitOK
}

type analysisJSON struct {
	Report  mtilt.Report `json:"analysis"`
	Ranking []int        `json:"ranking"`
}

func printAnalysis(w io.Writer, path string, a *mtilt.Analysis) {
	r := a.Report
	in := r.Input
	fmt.Fprintf(w, "input      %s (%s, %d triangles, %d vertices, sha256 %s)\n", path, in.Format, in.Triangles, in.Vertices, in.SHA256)
	fmt.Fprintf(w, "unit       %s (x%g to mm)\n", in.Unit, in.ScaleToMM)
	b := in.BoundsMM
	fmt.Fprintf(w, "size       %.4g x %.4g x %.4g mm\n", b[1][0]-b[0][0], b[1][1]-b[0][1], b[1][2]-b[0][2])
	if in.VolumeMM3 != 0 {
		fmt.Fprintf(w, "volume     %.6g mm3, surface area %.6g mm2\n", in.VolumeMM3, in.SurfaceAreaMM2)
	}
	fmt.Fprintf(w, "components %d\n", in.Components)
	fmt.Fprintln(w, "validation")
	for _, c := range r.Validation {
		if c.Detail != "" {
			fmt.Fprintf(w, "  %-9s %s: %s\n", c.Status, c.Name, c.Detail)
			continue
		}
		fmt.Fprintf(w, "  %-9s %s\n", c.Status, c.Name)
	}
	if len(a.Ranking) > 0 {
		fmt.Fprintf(w, "profile    %s (overhang threshold %g deg)\n", r.Profile.Name, r.Profile.OverhangThreshold)
		fmt.Fprintf(w, "candidates %d evaluated, best first (supports not built by analyze)\n", r.Search.CandidatesEvaluated)
		fmt.Fprintf(w, "  %4s %4s %-12s %10s %14s %12s %10s %s\n", "rank", "id", "source", "score", "demand_mm2", "contact_mm2", "height_mm", "fit")
		for i, id := range a.Ranking {
			if i == maxListedCandidates {
				fmt.Fprintf(w, "  ... %d more (use --json)\n", len(a.Ranking)-i)
				break
			}
			c := r.Candidates[id]
			m := c.Metrics
			fmt.Fprintf(w, "  %4d %4d %-12s %10.5f %14.3f %12.3f %10.3f %s\n",
				c.Rank, c.ID, c.Source, c.Score, m.SupportDemandProjectedAreaMM2, m.BedContactAreaMM2, m.HeightMM, c.Fit)
		}
	}
	for _, u := range r.Unchecked {
		fmt.Fprintf(w, "unchecked  %s: %s\n", u.Property, u.Note)
	}
	for _, msg := range r.Warnings {
		fmt.Fprintf(w, "warning    %s\n", msg)
	}
}
