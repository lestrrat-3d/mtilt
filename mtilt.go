package mtilt

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// maxUncoveredListed caps the uncovered samples a candidate report lists.
const maxUncoveredListed = 10

// session holds the state of one Prepare or Analyze call. Nothing in it is
// shared between calls.
type session struct {
	opts   Options
	body   *decad.Body
	model  *mesh.Mesh // the input body's tessellation
	bound  float64    // its proven distance from the true surface, mm
	tol    mesh.Tolerance
	pr     orient.Principal
	report *Report

	scale  orient.Scale
	cands  []orient.Candidate
	placed []*mesh.Mesh
	full   []r3.Transform
}

func newSession(ctx context.Context, body *decad.Body, opts Options) (*session, error) {
	opts, err := opts.normalized()
	if err != nil {
		return nil, err
	}
	if body == nil {
		return nil, fmt.Errorf("%w: body is nil", ErrInvalidOptions)
	}
	s := &session{opts: opts, body: body}
	s.report = &Report{
		SchemaVersion: SchemaVersion,
		Tool:          ToolReport{Name: "mtilt", Version: Version, Algorithm: AlgorithmVersion},
		Mode:          "optimize",
		Profile:       opts.Profile,
		Limits:        opts.Limits,
		Supports:      []SupportReport{},
		Candidates:    []CandidateReport{},
	}
	if opts.KeepOrientation {
		s.report.Mode = "keep_orientation"
	}
	if !opts.Profile.Calibrated {
		s.warnf("profile %q is not calibrated; its dimensions are illustrative and must be checked by printing", opts.Profile.Name)
	}
	s.addUnchecked()

	in := &s.report.Input
	in.Faces = len(body.Faces())
	in.Lumps = len(body.Lumps())
	in.ChordToleranceMM = opts.ChordToleranceMM
	if !body.IsSolid() {
		return s, s.fail(fmt.Errorf("%w: %w", ErrInvalidBody, decad.ErrNotSolid))
	}
	if in.Lumps != 1 {
		return s, s.fail(fmt.Errorf("%w: the body has %d lumps; mtilt prepares one connected solid", ErrUnsupportedInput, in.Lumps))
	}
	if err := s.readBody(); err != nil {
		return s, s.fail(fmt.Errorf("%w: %w", ErrInvalidBody, err))
	}

	m, bound, err := mesh.FromBody(ctx, body, opts.ChordToleranceMM)
	if err != nil {
		return s, s.fail(fmt.Errorf("%w: %w", ErrInvalidBody, err))
	}
	if len(m.Triangles) > opts.Limits.MaxTriangles {
		return nil, fmt.Errorf("%w: tessellation has %d triangles, limit is %d", ErrLimit, len(m.Triangles), opts.Limits.MaxTriangles)
	}
	s.model, s.bound = m, bound
	in.TessellationTriangles = len(m.Triangles)
	in.TessellationBoundMM = bound

	// Placement moves the model to within one diagonal of the origin, so
	// tolerances cover both the input coordinates and any placed ones.
	b := m.Bounds()
	d := b.Diagonal()
	s.tol = mesh.ToleranceFor(b.Union(mesh.Box{Min: r3.NewVec(-d, -d, -d), Max: r3.NewVec(d, d, d)}))
	s.report.Tolerances = ToleranceReport{LengthMM: s.tol.Length, SerializationMM: s.tol.Serialization, PlateMM: s.tol.Plate}

	rep, err := mesh.Validate(ctx, m, s.tol)
	if err != nil {
		return nil, err
	}
	s.report.Validation = append(s.report.Validation, prefixChecks("tessellation_", rep.Checks)...)
	if failed := rep.Failed(); len(failed) > 0 {
		c := failed[0]
		if c.Name == mesh.CheckSingleComponent {
			return s, s.fail(fmt.Errorf("%w: tessellation has %d components", ErrUnsupportedInput, rep.Components))
		}
		return s, s.fail(fmt.Errorf("%w: tessellation check %s failed: %s", ErrInvalidBody, c.Name, c.Detail))
	}

	s.pr = orient.NewPrincipal(m.Inertia())
	in.LongAxis = vec3(s.pr.Axes[0])
	in.Elongation = s.pr.Elongation
	return s, nil
}

// readBody records decad's own measurements of the input body.
func (s *session) readBody() error {
	in := &s.report.Input
	vol, err := s.body.Volume()
	if err != nil {
		return err
	}
	if in.VolumeMM3, err = vol.Value.In(units.CubicMillimeter); err != nil {
		return err
	}
	area, err := s.body.Area()
	if err != nil {
		return err
	}
	if in.SurfaceAreaMM2, err = area.Value.In(units.SquareMillimeter); err != nil {
		return err
	}
	box, err := s.body.Bounds()
	if err != nil {
		return err
	}
	in.BoundsMM = boxArray(mesh.Box{Min: box.Min, Max: box.Max})
	return nil
}

func (s *session) fail(err error) error {
	return &FailureError{Err: err, Report: s.report}
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

// evaluate searches for orientations, then places and measures each
// candidate. It returns candidate IDs in rank order: allowed and fitting
// first, then by estimated support cost.
func (s *session) evaluate(ctx context.Context) ([]int, error) {
	centroid := s.model.Centroid()
	var reach float64
	for _, v := range s.model.Vertices {
		reach = math.Max(reach, v.Sub(centroid).Len())
	}
	scale := orient.Scale{SurfaceAreaMM2: s.model.SurfaceArea(), SizeMM: 2 * reach, Elongation: s.pr.Elongation}
	s.scale = scale
	est := orient.NewEstimator(s.model, s.opts.Profile.OverhangThreshold, s.opts.Profile.PlateAnchorMM,
		s.opts.Profile.LayerHeightMM, s.planningParams().MinReachMM(), s.tol, s.opts.CostWeights, scale)

	if s.opts.KeepOrientation {
		d := r3.NewVec(0, 0, -1)
		s.cands = []orient.Candidate{{
			ID: 0, Source: orient.SourceOriginal, Down: d, Rotation: r3.Identity(), Allowed: true,
			ElevationDeg: orient.LongAxisElevation(s.pr, r3.Identity()), Estimate: est.Estimate(d),
		}}
	} else {
		cands, stats, err := orient.Search(ctx, s.model, s.pr, est, orient.SearchOptions{
			Directions:       s.opts.Limits.SweepDirections,
			MaxPlanarFaces:   s.opts.Limits.MaxPlanarFaces,
			Finalists:        s.opts.Limits.MaxSupportAttempts,
			MaxTiltDeg:       s.opts.MaxLongAxisTiltDeg,
			MinFirstLayerMM2: s.opts.Profile.MinFirstLayerAreaMM2,
		})
		if err != nil {
			return nil, err
		}
		s.cands = cands
		s.report.Search.SearchStats = stats
	}
	s.report.Search.Finalists = len(s.cands)
	s.report.Objective = ObjectiveReport{
		MaxLongAxisTiltDeg: s.opts.MaxLongAxisTiltDeg,
		TiltLimited:        s.report.Search.Constrained,
		CostWeights:        s.opts.CostWeights,
		Scale:              scale,
		Formula: "cost = volume*support_volume/(surface_area*size) + contact*support_contact/surface_area;" +
			" the estimate uses the column under each overhang, the planned cost the built pillars",
		Selection: "only orientations within the long-axis tilt limit and the build volume, with at least the minimum first-layer area; " +
			"among planned ones the lowest planned cost, then the larger first-layer area, the lower height, the lower long-axis elevation, the lower id",
		Quantum: orient.ScoreQuantum,
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
		met := orient.Measure(placed, s.opts.Profile.OverhangThreshold, s.opts.Profile.PlateAnchorMM, s.opts.Profile.LayerHeightMM, s.tol)
		met.LongAxisElevationDeg = c.ElevationDeg
		fit := s.fit(placed.Bounds())
		cr := CandidateReport{
			ID:          c.ID,
			Source:      string(c.Source),
			Down:        vec3(c.Down),
			Rotation:    rotationRows(c.Rotation),
			Metrics:     met,
			TiltAllowed: c.Allowed,
			Fit:         fit,
			Estimate:    c.Estimate,
			Attempt:     AttemptReport{Status: AttemptNotAttempted},
		}
		if c.Source == orient.SourcePlanarFace {
			n, a := vec3(c.FaceNormal), c.FaceAreaMM2
			cr.FaceNormal, cr.FaceAreaMM2 = &n, &a
		}
		s.report.Candidates = append(s.report.Candidates, cr)
		entries[i] = s.entry(&cr, c.Estimate.Score)
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
	e := s.tol.Length + s.bound
	if b.Min.X < -hx-e || b.Max.X > hx+e || b.Min.Y < -hy-e || b.Max.Y > hy+e || b.Min.Z < -e || b.Max.Z > v.ZMM-v.MarginMM+e {
		return fitExceeds
	}
	return fitFits
}

func (s *session) addUnchecked() {
	s.report.Unchecked = []Unchecked{
		{Property: "removal_accessibility", Note: "whether each support can be reached and broken away was not analyzed"},
		{Property: "physical_printability", Note: "no print, slicer run or material test was performed"},
		{Property: "slicer_interpretation", Note: "slicers can merge, offset or re-interface support bodies; inspect the sliced preview"},
		{Property: "bridges", Note: "spans up to max_bridge_mm between two walls are left unsupported; whether the printer bridges them cleanly was not tested"},
		{Property: "stability", Note: "centroid_over_contact_hull is a geometric heuristic, not a physical simulation"},
		{Property: "layer_strength", Note: "the strength term scores the long axis's angle to the plate; no load or layer-adhesion analysis was run"},
	}
	if s.opts.Profile.BuildVolume == nil {
		s.report.Unchecked = append(s.report.Unchecked, Unchecked{
			Property: "build_volume_fit", Note: "the profile has no build_volume",
		})
	}
}

// planningParams returns the support dimensions planning uses on the
// tessellation: the profile's, with the top gap and side clearance widened
// by the tessellation bound, because the true surface can lie that far from
// the mesh.
func (s *session) planningParams() support.Params {
	p := s.opts.Profile.supportParams()
	p.TopGapMM += s.bound
	p.SideClearanceMM += s.bound
	return p
}

// Prepare orients body for printing and builds breakaway supports for it as
// decad bodies.
//
// Candidates are tried in rank order (see Report.Objective). A candidate
// that exceeds the build volume is skipped. For each of the first
// Options.Limits.MaxSupportAttempts remaining candidates, Prepare plans
// supports on the tessellation (see package internal/support) and checks
// the plan; the first candidate whose plan covers all demand and passes
// every check is selected. A candidate with no support demand needs no
// supports.
//
// For the selected candidate only, Prepare adds bodies to body's document:
// the moved model (body.PlacedCopy; body itself stays live and unchanged)
// and one revolved pillar per support. It then runs decad's Verify and
// requires that no two of these bodies interfere and that each is a valid
// solid, and re-checks the pillars' gaps and spacing on their own
// tessellations.
//
// On failure Prepare returns a *FailureError wrapping ErrInvalidBody,
// ErrUnsupportedInput, ErrNoFeasibleCandidate or ErrAssembly, with the
// report so far. It returns ErrInvalidOptions or ErrLimit unwrapped, and
// ctx.Err() when ctx is cancelled.
func Prepare(ctx context.Context, body *decad.Body, opts Options) (*Result, error) {
	s, err := newSession(ctx, body, opts)
	if err != nil {
		return nil, err
	}
	order, err := s.evaluate(ctx)
	if err != nil {
		return nil, err
	}

	params := s.planningParams()
	lim := support.Limits{MaxSupports: s.opts.Limits.MaxSupports, MaxSamples: s.opts.Limits.MaxSamples}
	best := -1
	var bestPlan *support.Plan
	for _, id := range order {
		cr := &s.report.Candidates[id]
		switch {
		case !cr.TiltAllowed:
			cr.Attempt.Reason = fmt.Sprintf("the long axis rises %.1f degrees, over the %.1f degree limit", cr.Metrics.LongAxisElevationDeg, s.opts.MaxLongAxisTiltDeg)
			continue
		case cr.Fit == fitExceeds:
			cr.Attempt.Reason = "the model alone exceeds the build volume"
			continue
		case !s.firstLayerOK(cr):
			cr.Attempt.Reason = fmt.Sprintf("first layer area %.1f mm2 is under the profile's %.1f mm2 minimum", cr.Metrics.FirstLayerAreaMM2, s.opts.Profile.MinFirstLayerAreaMM2)
			continue
		case s.report.Search.SupportAttempts == s.opts.Limits.MaxSupportAttempts:
			s.report.Search.AttemptLimitReached = true
			continue
		}
		s.report.Search.SupportAttempts++
		plan, err := s.attempt(ctx, id, params, lim)
		switch {
		case errors.Is(err, errAttemptFailed):
			continue
		case err != nil:
			return nil, err
		}
		cost := s.plannedCost(plan)
		cr.Planned = &cost
		cr.Attempt.Status = AttemptSucceeded
		if len(plan.Pillars) == 0 {
			cr.Attempt.Status = AttemptNotNeeded
		}
		if best < 0 || orient.Less(s.entry(cr, cr.Planned.Score), s.entry(&s.report.Candidates[best], s.report.Candidates[best].Planned.Score)) {
			best, bestPlan = id, plan
		}
	}
	if best < 0 {
		return nil, s.fail(fmt.Errorf("%w: %d of %d candidates planned, none succeeded",
			ErrNoFeasibleCandidate, s.report.Search.SupportAttempts, len(s.cands)))
	}
	return s.build(ctx, best, bestPlan)
}

// entry describes a candidate for ranking with the given cost score. A
// candidate is feasible when it is within the tilt limit, fits the build
// volume, and has at least the profile's minimum first-layer area.
func (s *session) entry(cr *CandidateReport, score float64) orient.Entry {
	return orient.Entry{
		ID:                cr.ID,
		Feasible:          cr.TiltAllowed && cr.Fit != fitExceeds && s.firstLayerOK(cr),
		Score:             score,
		FirstLayerAreaMM2: cr.Metrics.FirstLayerAreaMM2,
		HeightMM:          cr.Metrics.HeightMM,
		ElevationDeg:      cr.Metrics.LongAxisElevationDeg,
	}
}

func (s *session) firstLayerOK(cr *CandidateReport) bool {
	return cr.Metrics.FirstLayerAreaMM2 >= s.opts.Profile.MinFirstLayerAreaMM2
}

// plannedCost returns the cost of a plan's pillars: their total volume and
// total contact area, priced like the estimate.
func (s *session) plannedCost(plan *support.Plan) orient.Cost {
	p := s.opts.Profile.supportParams()
	return orient.NewCost(p.PlanVolume(plan), float64(len(plan.Pillars))*p.ContactArea(), s.opts.CostWeights, s.scale)
}

// errAttemptFailed is returned by attempt when a candidate's supports fail;
// the reason is recorded in the candidate's report.
var errAttemptFailed = errors.New("mtilt: support attempt failed")

// attempt plans and checks supports for one candidate on its tessellation.
func (s *session) attempt(ctx context.Context, id int, params support.Params, lim support.Limits) (*support.Plan, error) {
	cr := &s.report.Candidates[id]
	placed := s.placed[id]
	failed := func(reason string) (*support.Plan, error) {
		cr.Attempt.Status, cr.Attempt.Reason = AttemptFailed, reason
		return nil, errAttemptFailed
	}
	plan, err := support.Build(ctx, placed, params, lim, s.tol)
	switch {
	case errors.Is(err, support.ErrLimit):
		return failed(err.Error())
	case err != nil:
		return nil, err
	}
	cr.Attempt.Samples = plan.Samples
	cr.Attempt.BridgedSamples, cr.Attempt.HeldSamples = plan.Bridged, plan.Held
	cr.Attempt.Supports = len(plan.Pillars)
	if n := len(plan.Uncovered); n > 0 {
		cr.Attempt.UncoveredCount = n
		cr.Attempt.Uncovered = plan.Uncovered[:min(n, maxUncoveredListed)]
		return failed(fmt.Sprintf("coverage gap: %d of %d demand samples uncovered; first: %s", n, plan.Samples, plan.Uncovered[0].Reason))
	}
	for _, c := range support.RecheckPlan(placed, plan, params, s.tol) {
		if c.Status == mesh.StatusFailed {
			return failed(fmt.Sprintf("plan check %s failed: %s", c.Name, c.Detail))
		}
	}
	all := placed.Bounds()
	half := s.opts.Profile.BaseWidthMM / 2
	for _, pl := range plan.Pillars {
		all = all.Union(mesh.Box{Min: r3.NewVec(pl.X-half, pl.Y-half, 0), Max: r3.NewVec(pl.X+half, pl.Y+half, pl.TopZ)})
	}
	if s.fit(all) == fitExceeds {
		return failed("model plus supports exceed the build volume")
	}
	return plan, nil
}

// build turns the selected candidate's plan into decad bodies and verifies
// them.
func (s *session) build(ctx context.Context, id int, plan *support.Plan) (*Result, error) {
	cr := &s.report.Candidates[id]
	rep := s.report
	full := s.full[id]

	model, err := s.body.PlacedCopy(ctx, full)
	if err != nil {
		return nil, fmt.Errorf("mtilt: placing the model: %w", err)
	}
	res := &Result{Model: model, Transform: full, Report: *rep}
	params := s.opts.Profile.supportParams()
	w := sketch.NewWorld()
	// One body per straight pillar or single branch, then one per tree.
	var bodies []supportBody
	for _, pl := range plan.Pillars {
		if pl.Tree == 0 {
			bodies = append(bodies, supportBody{tips: []support.Pillar{pl}})
		}
	}
	for i := range plan.Trees {
		bodies = append(bodies, supportBody{tree: &plan.Trees[i], tips: plan.Members(i + 1)})
	}
	upright := make([]bool, len(bodies))
	for i, sb := range bodies {
		var pb *decad.Body
		var err error
		if sb.tree != nil {
			pb, err = params.TreeBody(ctx, w, s.body.Document(), *sb.tree, sb.tips)
		} else {
			pb, err = params.Body(ctx, w, s.body.Document(), sb.tips[0])
		}
		if err != nil {
			s.remove(res)
			return nil, fmt.Errorf("mtilt: building support body: %w", err)
		}
		res.Supports = append(res.Supports, pb)
		upright[i] = sb.tree == nil && !sb.tips[0].IsBranch()
	}

	checks, err := s.verifyAssembly(ctx, res, upright)
	if err != nil {
		s.remove(res)
		return nil, err
	}
	rep.Validation = append(rep.Validation, checks...)
	if full.IsReflection() {
		rep.Validation = append(rep.Validation, mesh.Check{Name: "proper_rigid_transform", Status: mesh.StatusFailed, Detail: "transform mirrors the model"})
	} else {
		rep.Validation = append(rep.Validation, mesh.Check{Name: "proper_rigid_transform", Status: mesh.StatusPassed})
	}

	cr.Attempt.Status = AttemptSucceeded
	if len(plan.Pillars) == 0 {
		cr.Attempt.Status = AttemptNotNeeded
	}
	rep.Selected = &cr.ID
	if rep.Transform, err = transformReport(full); err != nil {
		return nil, err
	}
	mb, err := model.Bounds()
	if err != nil {
		return nil, err
	}
	rep.Model = &ModelReport{BoundsMM: boxArray(mesh.Box{Min: mb.Min, Max: mb.Max}), VolumeMM3: rep.Input.VolumeMM3}
	for i, sb := range bodies {
		pl := sb.tips[0]
		sr := SupportReport{
			ID:         fmt.Sprintf("support-%04d", i+1),
			CenterMM:   [2]float64{pl.X, pl.Y},
			Kind:       "pillar",
			SurfaceZMM: pl.SurfaceZ,
			TopZMM:     pl.TopZ,
			Origin:     string(pl.Origin),
		}
		switch {
		case sb.tree != nil:
			sr.Kind = "tree"
			foot := [2]float64{sb.tree.FootX, sb.tree.FootY}
			top := sb.tree.TopZ
			sr.FootMM, sr.KneeZMM = &foot, &top
			for _, m := range sb.tips {
				sr.TipsMM = append(sr.TipsMM, [3]float64{m.X, m.Y, m.TopZ})
			}
		case pl.IsBranch():
			sr.Kind = "branch"
			foot := [2]float64{pl.FootX, pl.FootY}
			knee := pl.KneeZ
			sr.FootMM, sr.KneeZMM = &foot, &knee
		}
		if vol, err := res.Supports[i].Volume(); err == nil {
			sr.VolumeMM3, _ = vol.Value.In(units.CubicMillimeter)
		}
		if b, err := res.Supports[i].Bounds(); err == nil {
			sr.BoundsMM = boxArray(mesh.Box{Min: b.Min, Max: b.Max})
		}
		rep.Supports = append(rep.Supports, sr)
	}
	if len(plan.Pillars) > 0 {
		s.warnf("support geometry passed mtilt's and decad's geometric checks only; breakaway behavior depends on the printer, material and slicer and has not been tested")
	}
	res.Report = *rep

	for _, c := range rep.Validation {
		if c.Status == mesh.StatusFailed {
			cr.Attempt.Status = AttemptFailed
			cr.Attempt.Reason = fmt.Sprintf("assembly check %s failed: %s", c.Name, c.Detail)
			rep.Selected = nil
			res.Report = *rep
			s.remove(res)
			return nil, &FailureError{Err: fmt.Errorf("%w: %s: %s", ErrAssembly, c.Name, c.Detail), Report: rep, Result: res}
		}
	}
	return res, nil
}

// supportBody is one support body to build: a straight pillar or single
// branch (tree nil, one tip), or a tree and its members.
type supportBody struct {
	tree *support.Tree
	tips []support.Pillar
}

// remove takes every body Prepare added for res out of the document again
// (decad.Document.Remove), so a failed preparation leaves the caller's
// document as it was. The removed bodies stay readable.
func (s *session) remove(res *Result) {
	doc := s.body.Document()
	for _, b := range append([]*decad.Body{res.Model}, res.Supports...) {
		if b != nil {
			_ = doc.Remove(b)
		}
	}
}

// verifyAssembly runs decad's Verify over the document and keeps the rows
// that concern the assembly's own bodies, then re-checks the supports on
// their tessellations. upright[i] is true for straight pillars.
func (s *session) verifyAssembly(ctx context.Context, res *Result, upright []bool) ([]mesh.Check, error) {
	ours := map[*decad.Body]string{res.Model: "model"}
	for i, b := range res.Supports {
		ours[b] = fmt.Sprintf("support-%04d", i+1)
	}
	vr, err := s.body.Document().Verify(ctx)
	if err != nil {
		return nil, fmt.Errorf("mtilt: decad verify: %w", err)
	}
	validity := mesh.Check{Name: "decad_body_validity", Status: mesh.StatusPassed}
	seen := 0
	for _, br := range vr.Bodies {
		name, ok := ours[br.Body]
		if !ok {
			continue
		}
		seen++
		if br.Validity.Outcome != decad.ValidityValid && validity.Status == mesh.StatusPassed {
			validity.Status = mesh.StatusFailed
			validity.Detail = fmt.Sprintf("%s: decad validity outcome %v", name, br.Validity.Outcome)
		}
	}
	if seen != len(ours) && validity.Status == mesh.StatusPassed {
		validity.Status = mesh.StatusFailed
		validity.Detail = fmt.Sprintf("decad reported %d of %d assembly bodies", seen, len(ours))
	}
	lumps := mesh.Check{Name: "decad_support_lumps", Status: mesh.StatusPassed}
	for i, b := range res.Supports {
		if n := len(b.Lumps()); n != 1 {
			lumps.Status = mesh.StatusFailed
			lumps.Detail = fmt.Sprintf("support-%04d is %d lumps", i+1, n)
			break
		}
	}
	interference := mesh.Check{Name: "decad_interference", Status: mesh.StatusPassed}
	for _, in := range vr.Interferences {
		a, okA := ours[in.A]
		b, okB := ours[in.B]
		if okA && okB {
			interference.Status = mesh.StatusFailed
			interference.Detail = fmt.Sprintf("%s and %s overlap by %s", a, b, in.Volume.Value)
			break
		}
	}
	checks := []mesh.Check{validity, lumps, interference}

	model, mbound, err := mesh.FromBody(ctx, res.Model, s.opts.ChordToleranceMM)
	if err != nil {
		return nil, err
	}
	supports := make([]*mesh.Mesh, len(res.Supports))
	var sbound float64
	for i, b := range res.Supports {
		m, bound, err := mesh.FromBody(ctx, b, s.opts.ChordToleranceMM)
		if err != nil {
			return nil, err
		}
		supports[i] = m
		sbound = math.Max(sbound, bound)
	}
	more, err := support.ValidateAssembly(ctx, model, supports, upright, s.opts.Profile.TopContactGapMM, mbound+sbound, s.tol)
	if err != nil {
		return nil, err
	}
	return append(checks, prefixChecks("tessellated_", more)...), nil
}

// Analysis is the outcome of Analyze: the input readings and the ranked
// candidate orientations. Analyze builds no supports and adds no bodies to
// the document, so every candidate's support attempt is not_attempted.
type Analysis struct {
	Report Report
	// Ranking lists candidate IDs best first.
	Ranking []int
}

// Analyze measures and ranks candidate orientations of body without building
// supports. When the body is not a supported input it returns the analysis
// so far together with a *FailureError.
func Analyze(ctx context.Context, body *decad.Body, opts Options) (*Analysis, error) {
	s, err := newSession(ctx, body, opts)
	if err != nil {
		var fe *FailureError
		if errors.As(err, &fe) {
			return &Analysis{Report: *fe.Report}, err
		}
		return nil, err
	}
	order, err := s.evaluate(ctx)
	if err != nil {
		return nil, err
	}
	return &Analysis{Report: *s.report, Ranking: order}, nil
}
