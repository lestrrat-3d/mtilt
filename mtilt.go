package mtilt

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
)

// Input is a mesh to prepare and the unit its coordinates are in.
type Input struct {
	// Mesh must not be nil. Prepare and Analyze do not modify it.
	Mesh *mesh.Mesh
	// Unit is required: STL coordinates carry no unit.
	Unit Unit
	// Source is copied into the report.
	Source Source
}

// maxUncoveredListed caps the uncovered samples a candidate report lists.
const maxUncoveredListed = 10

// session holds the state of one Prepare or Analyze call.
type session struct {
	opts   Options
	in     Input
	scale  float64
	model  *mesh.Mesh // unit-converted input
	tol    mesh.Tolerance
	report *Report

	cands  []orient.Candidate
	placed []*mesh.Mesh
	full   []r3.Transform
}

func newSession(ctx context.Context, in Input, opts Options) (*session, error) {
	opts, err := opts.normalized()
	if err != nil {
		return nil, err
	}
	if in.Mesh == nil {
		return nil, fmt.Errorf("%w: input mesh is nil", ErrInvalidOptions)
	}
	scale, ok := in.Unit.ToMillimeters()
	if !ok {
		return nil, fmt.Errorf("%w: unknown input unit %q (use mm, cm, m or in)", ErrInvalidOptions, in.Unit)
	}
	if len(in.Mesh.Triangles) > opts.Limits.MaxTriangles {
		return nil, fmt.Errorf("%w: %d triangles, limit is %d", ErrLimit, len(in.Mesh.Triangles), opts.Limits.MaxTriangles)
	}
	s := &session{opts: opts, in: in, scale: scale, model: in.Mesh.Scaled(scale)}

	// Placement moves the model to within one diagonal of the origin, so
	// tolerances cover both the input coordinates and any placed ones.
	b := s.model.Bounds()
	d := b.Diagonal()
	s.tol = mesh.ToleranceFor(b.Union(mesh.Box{Min: r3.NewVec(-d, -d, -d), Max: r3.NewVec(d, d, d)}))

	mode := "optimize"
	if opts.KeepOrientation {
		mode = "keep_orientation"
	}
	s.report = &Report{
		SchemaVersion: SchemaVersion,
		Tool:          ToolReport{Name: "mtilt", Version: Version, Algorithm: AlgorithmVersion},
		Input: InputReport{
			Source:    in.Source,
			Triangles: len(in.Mesh.Triangles),
			Vertices:  len(in.Mesh.Vertices),
			Unit:      in.Unit,
			ScaleToMM: scale,
			BoundsMM:  boxArray(b),
		},
		Mode:       mode,
		Tolerances: ToleranceReport{LengthMM: s.tol.Length, SerializationMM: s.tol.Serialization, PlateMM: s.tol.Plate},
		Profile:    opts.Profile,
		Limits:     opts.Limits,
		Supports:   []SupportReport{},
		Candidates: []CandidateReport{},
	}
	if in.Source.NormalsDisagreeing > 0 {
		s.warnf("%d stored facet normals point against the vertex winding; mtilt used the normals computed from the winding", in.Source.NormalsDisagreeing)
	}
	if !opts.Profile.Calibrated {
		s.warnf("profile %q is not calibrated; its dimensions are illustrative and must be checked by printing", opts.Profile.Name)
	}

	rep, err := mesh.Validate(ctx, s.model, s.tol)
	if err != nil {
		return nil, err
	}
	s.report.Validation = append(s.report.Validation, prefixChecks("input_", rep.Checks)...)
	s.report.Input.Components = rep.Components
	if failed := rep.Failed(); len(failed) > 0 {
		base := ErrInvalidMesh
		if len(failed) == 1 && failed[0].Name == mesh.CheckSingleComponent {
			base = ErrUnsupportedInput
		}
		s.addUnchecked()
		return s, &FailureError{Err: fmt.Errorf("%w: %s: %s", base, failed[0].Name, failed[0].Detail), Report: s.report}
	}
	s.report.Input.SurfaceAreaMM2 = s.model.SurfaceArea()
	s.report.Input.VolumeMM3 = s.model.Volume()
	return s, nil
}

func (s *session) warnf(format string, args ...any) {
	s.report.Warnings = append(s.report.Warnings, fmt.Sprintf(format, args...))
}

func prefixChecks(prefix string, checks []mesh.Check) []mesh.Check {
	out := make([]mesh.Check, len(checks))
	for i, c := range checks {
		c.Name = prefix + c.Name
		out[i] = c
	}
	return out
}

// evaluate generates, places, measures and ranks the candidates. It returns
// candidate IDs in rank order.
func (s *session) evaluate(ctx context.Context) ([]int, error) {
	if s.opts.KeepOrientation {
		s.cands = []orient.Candidate{{ID: 0, Source: orient.SourceOriginal, Rotation: r3.Identity()}}
		s.report.Search.CandidatesDistinct = 1
	} else {
		cands, gen, err := orient.Generate(ctx, s.model, orient.Limits{
			MaxCandidates:  s.opts.Limits.MaxCandidates,
			MaxPlanarFaces: s.opts.Limits.MaxPlanarFaces,
		})
		if err != nil {
			return nil, err
		}
		s.cands = cands
		s.report.Search.CandidatesDistinct = gen.Distinct
		s.report.Search.CandidateLimitReached = gen.Truncated
	}
	s.report.Search.CandidatesEvaluated = len(s.cands)

	centroid := s.model.Centroid()
	var reach float64
	for _, v := range s.model.Vertices {
		reach = math.Max(reach, v.Sub(centroid).Len())
	}
	scale := orient.Scale{SurfaceAreaMM2: s.model.SurfaceArea(), SizeMM: 2 * reach}
	s.report.Objective = ObjectiveReport{
		Weights:  s.opts.Weights,
		Scale:    scale,
		Formula:  "score = support_demand*projected_demand_area/surface_area + height*height/size - bed_contact*bed_contact_area/surface_area; lower is better",
		TieBreak: "feasible before infeasible, then score rounded to score_quantum, then lower candidate id",
		Quantum:  orient.ScoreQuantum,
	}

	entries := make([]orient.Entry, len(s.cands))
	s.placed = make([]*mesh.Mesh, len(s.cands))
	s.full = make([]r3.Transform, len(s.cands))
	for i, c := range s.cands {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		placed, full, err := orient.Place(s.model, c.Rotation)
		if err != nil {
			return nil, fmt.Errorf("mtilt: placing candidate %d: %w", c.ID, err)
		}
		s.placed[i], s.full[i] = placed, full
		met := orient.Measure(placed, s.opts.Profile.OverhangThreshold, s.tol)
		terms, score := orient.Score(met, s.opts.Weights, scale)
		fit := s.fit(placed.Bounds())
		cr := CandidateReport{
			ID:       c.ID,
			Source:   string(c.Source),
			Rotation: rotationRows(c.Rotation),
			Metrics:  met,
			Fit:      fit,
			Terms:    terms,
			Score:    score,
			Attempt:  AttemptReport{Status: AttemptNotAttempted},
		}
		if c.Source == orient.SourcePlanarFace {
			n, a := vec3(c.FaceNormal), c.FaceAreaMM2
			cr.FaceNormal, cr.FaceAreaMM2 = &n, &a
		}
		s.report.Candidates = append(s.report.Candidates, cr)
		entries[i] = orient.Entry{ID: c.ID, Feasible: fit != fitExceeds, Score: score}
	}
	order := orient.Rank(entries)
	for rank, id := range order {
		s.report.Candidates[id].Rank = rank + 1
	}
	return order, nil
}

const (
	fitFits      = "fits"
	fitExceeds   = "exceeds"
	fitUnchecked = "unchecked"
)

// fit tests a placed box against the build volume. The volume's X and Y
// extents are centered on the origin.
func (s *session) fit(b mesh.Box) string {
	v := s.opts.Profile.BuildVolume
	if v == nil {
		return fitUnchecked
	}
	hx, hy := v.XMM/2-v.MarginMM, v.YMM/2-v.MarginMM
	e := s.tol.Length
	if b.Min.X < -hx-e || b.Max.X > hx+e || b.Min.Y < -hy-e || b.Max.Y > hy+e || b.Min.Z < -e || b.Max.Z > v.ZMM-v.MarginMM+e {
		return fitExceeds
	}
	return fitFits
}

func (s *session) addUnchecked() {
	s.report.Unchecked = []Unchecked{
		{Property: "input_self_intersection", Note: "mtilt does not test whether input triangles cross each other"},
		{Property: "removal_accessibility", Note: "whether each support can be reached and broken away was not analyzed"},
		{Property: "physical_printability", Note: "no print, slicer run or material test was performed"},
		{Property: "slicer_interpretation", Note: "slicers can merge, offset or re-interface support bodies; inspect the sliced preview"},
		{Property: "bridges", Note: "bridges are not detected; every downward surface below the threshold, including spans between walls, counts as support demand"},
		{Property: "stability", Note: "centroid_over_contact_hull is a geometric heuristic, not a physical simulation"},
	}
	if s.opts.Profile.BuildVolume == nil {
		s.report.Unchecked = append(s.report.Unchecked, Unchecked{
			Property: "build_volume_fit", Note: "the profile has no build_volume",
		})
	}
}

// Prepare orients the input for printing and builds breakaway supports for
// it.
//
// Candidates are tried in rank order (see Report.Objective). A candidate
// that exceeds the build volume is skipped. For each of the first
// Options.Limits.MaxSupportAttempts remaining candidates, Prepare builds
// supports (see package internal/support) and validates the assembly; the
// first candidate whose supports cover all demand and pass every assembly
// check is selected. A candidate with no support demand needs no supports.
//
// On failure Prepare returns a *FailureError wrapping ErrInvalidMesh,
// ErrUnsupportedInput or ErrNoFeasibleCandidate, with the report so far. It
// returns ErrInvalidOptions or ErrLimit unwrapped, and ctx.Err() when ctx is
// cancelled.
func Prepare(ctx context.Context, in Input, opts Options) (*Result, error) {
	s, err := newSession(ctx, in, opts)
	if err != nil {
		return nil, err
	}
	s.addUnchecked()
	order, err := s.evaluate(ctx)
	if err != nil {
		return nil, err
	}

	params := s.opts.Profile.supportParams()
	lim := support.Limits{MaxSupports: s.opts.Limits.MaxSupports, MaxSamples: s.opts.Limits.MaxSamples}
	for i, id := range order {
		cr := &s.report.Candidates[id]
		if cr.Fit == fitExceeds {
			cr.Attempt.Reason = "the model alone exceeds the build volume"
			continue
		}
		if s.report.Search.SupportAttempts == s.opts.Limits.MaxSupportAttempts {
			for _, rest := range order[i:] {
				if s.report.Candidates[rest].Fit != fitExceeds {
					s.report.Search.AttemptLimitReached = true
					break
				}
			}
			break
		}
		s.report.Search.SupportAttempts++
		res, err := s.attempt(ctx, id, params, lim)
		switch {
		case errors.Is(err, errAttemptFailed):
			continue
		case err != nil:
			return nil, err
		}
		return res, nil
	}
	return nil, &FailureError{
		Err:    fmt.Errorf("%w: %d of %d candidates attempted, none succeeded", ErrNoFeasibleCandidate, s.report.Search.SupportAttempts, len(s.cands)),
		Report: s.report,
	}
}

// errAttemptFailed is returned by attempt when a candidate's supports fail;
// the reason is recorded in the candidate's report.
var errAttemptFailed = errors.New("mtilt: support attempt failed")

// attempt builds and validates supports for one candidate.
func (s *session) attempt(ctx context.Context, id int, params support.Params, lim support.Limits) (*Result, error) {
	cr := &s.report.Candidates[id]
	placed := s.placed[id]
	plan, err := support.Build(ctx, placed, params, lim, s.tol)
	switch {
	case errors.Is(err, support.ErrLimit):
		cr.Attempt.Status, cr.Attempt.Reason = AttemptFailed, err.Error()
		return nil, errAttemptFailed
	case err != nil:
		return nil, err
	}
	cr.Attempt.Samples = plan.Samples
	cr.Attempt.Supports = len(plan.Pillars)
	if n := len(plan.Uncovered); n > 0 {
		cr.Attempt.Status = AttemptFailed
		cr.Attempt.UncoveredCount = n
		cr.Attempt.Uncovered = plan.Uncovered[:min(n, maxUncoveredListed)]
		cr.Attempt.Reason = fmt.Sprintf("coverage gap: %d of %d demand samples uncovered; first: %s", n, plan.Samples, plan.Uncovered[0].Reason)
		return nil, errAttemptFailed
	}

	supports := make([]*mesh.Mesh, len(plan.Pillars))
	for i, pl := range plan.Pillars {
		supports[i] = params.Mesh(pl)
	}
	checks := support.RecheckPlan(placed, plan, params, s.tol)
	more, err := support.ValidateAssembly(ctx, placed, supports, params.TopGapMM, 0, s.tol)
	if err != nil {
		return nil, err
	}
	checks = append(checks, more...)
	all := placed.Bounds()
	for _, m := range supports {
		all = all.Union(m.Bounds())
	}
	switch s.fit(all) {
	case fitFits:
		checks = append(checks, mesh.Check{Name: "assembly_build_volume_fit", Status: mesh.StatusPassed})
	case fitExceeds:
		checks = append(checks, mesh.Check{Name: "assembly_build_volume_fit", Status: mesh.StatusFailed, Detail: "model plus supports exceed the build volume"})
	default:
		checks = append(checks, mesh.Check{Name: "assembly_build_volume_fit", Status: mesh.StatusUnchecked, Detail: "the profile has no build_volume"})
	}
	if s.full[id].IsReflection() {
		checks = append(checks, mesh.Check{Name: "proper_rigid_transform", Status: mesh.StatusFailed, Detail: "transform mirrors the model"})
	} else {
		checks = append(checks, mesh.Check{Name: "proper_rigid_transform", Status: mesh.StatusPassed})
	}
	for _, c := range checks {
		if c.Status == mesh.StatusFailed {
			cr.Attempt.Status = AttemptFailed
			cr.Attempt.Reason = fmt.Sprintf("assembly check %s failed: %s", c.Name, c.Detail)
			return nil, errAttemptFailed
		}
	}

	cr.Attempt.Status = AttemptSucceeded
	if len(plan.Pillars) == 0 {
		cr.Attempt.Status = AttemptNotNeeded
	}
	rep := s.report
	rep.Validation = append(rep.Validation, checks...)
	rep.Selected = &cr.ID
	tr, err := transformReport(s.full[id], s.scale)
	if err != nil {
		return nil, err
	}
	rep.Transform = tr
	rep.Model = &ModelReport{BoundsMM: boxArray(placed.Bounds()), VolumeMM3: placed.Volume()}
	for i, pl := range plan.Pillars {
		rep.Supports = append(rep.Supports, SupportReport{
			ID:         fmt.Sprintf("support-%04d", i+1),
			CenterMM:   [2]float64{pl.X, pl.Y},
			SurfaceZMM: pl.SurfaceZ,
			TopZMM:     pl.TopZ,
			Origin:     string(pl.Origin),
			VolumeMM3:  supports[i].Volume(),
			BoundsMM:   boxArray(supports[i].Bounds()),
		})
	}
	if plan.DemandTriangles > 0 {
		s.warnf("support geometry passed mtilt's geometric checks only; breakaway behavior depends on the printer, material and slicer and has not been tested")
	}
	return &Result{Model: placed, Supports: supports, Transform: s.full[id], Report: *rep}, nil
}

// Analysis is the outcome of Analyze: input validation, geometry, and the
// ranked candidate orientations. Analyze builds no supports, so every
// candidate's support attempt is not_attempted.
type Analysis struct {
	Report Report
	// Ranking lists candidate IDs best first.
	Ranking []int
}

// Analyze validates the input and measures and ranks candidate orientations
// without building supports. When the input fails validation it returns the
// analysis so far together with a *FailureError.
func Analyze(ctx context.Context, in Input, opts Options) (*Analysis, error) {
	s, err := newSession(ctx, in, opts)
	if err != nil {
		var fe *FailureError
		if errors.As(err, &fe) {
			return &Analysis{Report: *fe.Report}, err
		}
		return nil, err
	}
	s.addUnchecked()
	order, err := s.evaluate(ctx)
	if err != nil {
		return nil, err
	}
	return &Analysis{Report: *s.report, Ranking: order}, nil
}

// ValidateSerialized checks meshes read back from the files written for res
// against res itself. model and supports must hold the triangles in the
// order they were written. Coordinates may differ by the serialization
// tolerance recorded in the report; topology checks run again, and the
// support assembly checks run again on the re-read meshes.
func ValidateSerialized(ctx context.Context, res *Result, model *mesh.Mesh, supports []*mesh.Mesh) ([]mesh.Check, error) {
	tol := mesh.Tolerance{
		Length:        res.Report.Tolerances.LengthMM,
		Serialization: res.Report.Tolerances.SerializationMM,
		Plate:         res.Report.Tolerances.PlateMM,
	}
	var checks []mesh.Check

	modelCheck := mesh.Check{Name: "serialized_model_matches", Status: mesh.StatusPassed}
	if d, ok := maxDeviation(res.Model, model); !ok {
		modelCheck.Status, modelCheck.Detail = mesh.StatusFailed, "triangle count differs"
	} else if d > tol.Serialization {
		modelCheck.Status, modelCheck.Detail = mesh.StatusFailed, fmt.Sprintf("a coordinate moved %g mm, allowance %g", d, tol.Serialization)
	}
	checks = append(checks, modelCheck)

	rep, err := mesh.Validate(ctx, model, tol)
	if err != nil {
		return nil, err
	}
	checks = append(checks, prefixChecks("serialized_model_", rep.Checks)...)

	supportCheck := mesh.Check{Name: "serialized_supports_match", Status: mesh.StatusPassed}
	if len(supports) != len(res.Supports) {
		supportCheck.Status, supportCheck.Detail = mesh.StatusFailed, "support count differs"
	} else {
		for i := range supports {
			d, ok := maxDeviation(res.Supports[i], supports[i])
			if !ok || d > tol.Serialization {
				supportCheck.Status = mesh.StatusFailed
				supportCheck.Detail = fmt.Sprintf("support %d differs from what was built", i+1)
				break
			}
		}
	}
	checks = append(checks, supportCheck)

	more, err := support.ValidateAssembly(ctx, model, supports, res.Report.Profile.TopContactGapMM, 2*tol.Serialization, tol)
	if err != nil {
		return nil, err
	}
	return append(checks, prefixChecks("serialized_", more)...), nil
}

func maxDeviation(want, got *mesh.Mesh) (float64, bool) {
	if len(want.Triangles) != len(got.Triangles) {
		return 0, false
	}
	var d float64
	for i := range want.Triangles {
		a, b := want.Triangle(i), got.Triangle(i)
		for k := range 3 {
			diff := a[k].Sub(b[k])
			d = math.Max(d, math.Max(math.Abs(diff.X), math.Max(math.Abs(diff.Y), math.Abs(diff.Z))))
		}
	}
	return d, true
}
