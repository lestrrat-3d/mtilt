package mtilt

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/r3"
)

// Version identifiers recorded in every report.
const (
	// SchemaVersion is the version of the Report JSON layout.
	SchemaVersion = 2
	// Version is the mtilt release.
	Version = "0.1.0-dev"
	// AlgorithmVersion names the orientation search and the support
	// strategy. It changes whenever either can produce different output
	// for the same input and options.
	AlgorithmVersion = "orient-2/pillar-2"
)

// Errors Prepare and Analyze return. Failures that carry a report are
// wrapped in *FailureError.
var (
	// ErrInvalidBody reports an input body that is not a solid, or whose
	// tessellation fails a check.
	ErrInvalidBody = errors.New("mtilt: invalid body")
	// ErrUnsupportedInput reports a solid outside the supported topology:
	// more than one lump.
	ErrUnsupportedInput = errors.New("mtilt: unsupported input")
	// ErrAssembly reports that the support bodies built for the selected
	// candidate failed decad's verification. Prepare removes them from the
	// document again (decad.Document.Remove); the FailureError's Result still
	// holds them, readable but retired.
	ErrAssembly = errors.New("mtilt: support assembly failed verification")
	// ErrNoFeasibleCandidate reports that no evaluated orientation got a
	// complete, validated set of supports within the work limits.
	ErrNoFeasibleCandidate = errors.New("mtilt: no feasible orientation")
	// ErrLimit reports input larger than a configured limit.
	ErrLimit = errors.New("mtilt: work limit reached")
)

// FailureError is a failed preparation together with the report built up to
// the failure.
type FailureError struct {
	Err    error
	Report *Report
	// Result is set only with ErrAssembly: the bodies that were built.
	Result *Result
}

func (e *FailureError) Error() string { return e.Err.Error() }
func (e *FailureError) Unwrap() error { return e.Err }

// Result is a successful preparation. Model and Supports are live bodies in
// the input body's document; the input body itself is left live and
// unchanged.
type Result struct {
	// Model is a copy of the input body moved by Transform
	// (decad.Body.PlacedCopy).
	Model *decad.Body
	// Supports are the support bodies, in the same frame as Model, in
	// Report.Supports order.
	Supports []*decad.Body
	// Transform maps the input body to Model.
	Transform r3.Transform
	Report    Report
}

// Report is the reproducible record of one preparation. Encoded as JSON it
// is the plan.json file the CLI writes. It holds no timing data.
type Report struct {
	SchemaVersion int               `json:"schema_version"`
	Tool          ToolReport        `json:"tool"`
	Input         InputReport       `json:"input"`
	Mode          string            `json:"mode"`
	Tolerances    ToleranceReport   `json:"tolerances"`
	Profile       Profile           `json:"profile"`
	Objective     ObjectiveReport   `json:"objective"`
	Limits        Limits            `json:"limits"`
	Search        SearchReport      `json:"search"`
	Candidates    []CandidateReport `json:"candidates"`
	// Selected is the ID of the candidate whose supports succeeded, or
	// nil when none did.
	Selected   *int             `json:"selected_candidate"`
	Transform  *TransformReport `json:"transform"`
	Model      *ModelReport     `json:"model"`
	Supports   []SupportReport  `json:"supports"`
	Validation []mesh.Check     `json:"validation"`
	Unchecked  []Unchecked      `json:"unchecked"`
	Warnings   []string         `json:"warnings"`
}

// ToolReport identifies the program that wrote a report.
type ToolReport struct {
	Name      string `json:"name"`
	Version   string `json:"version"`
	Algorithm string `json:"algorithm"`
}

// InputReport describes the input body and the tessellation mtilt planned on.
type InputReport struct {
	Faces          int           `json:"faces"`
	Lumps          int           `json:"lumps"`
	VolumeMM3      float64       `json:"volume_mm3"`
	SurfaceAreaMM2 float64       `json:"surface_area_mm2"`
	BoundsMM       [2][3]float64 `json:"bounds_mm"`
	// TessellationTriangles and TessellationBoundMM describe the mesh
	// decad produced at ChordToleranceMM; the bound is decad's proven
	// distance from the true surface to the mesh.
	ChordToleranceMM      float64 `json:"chord_tolerance_mm"`
	TessellationTriangles int     `json:"tessellation_triangles"`
	TessellationBoundMM   float64 `json:"tessellation_bound_mm"`
	// LongAxis is the principal axis with the smallest moment of inertia,
	// in the input frame, and Elongation is how much longer the body is
	// along it than across it (see orient.Principal).
	LongAxis   [3]float64 `json:"long_axis"`
	Elongation float64    `json:"elongation"`
}

// ToleranceReport records the numeric tolerances used (see mesh.Tolerance).
type ToleranceReport struct {
	LengthMM        float64 `json:"length_mm"`
	SerializationMM float64 `json:"serialization_mm"`
	PlateMM         float64 `json:"plate_mm"`
}

// ObjectiveReport records how candidates were scored and ordered.
type ObjectiveReport struct {
	// MaxLongAxisTiltDeg is the tilt limit; TiltLimited says whether it
	// applied (the body has a long axis and the limit is under 90).
	MaxLongAxisTiltDeg float64      `json:"max_long_axis_tilt_deg"`
	TiltLimited        bool         `json:"tilt_limited"`
	CostWeights        CostWeights  `json:"cost_weights"`
	Scale              orient.Scale `json:"normalization"`
	Formula            string       `json:"formula"`
	Selection          string       `json:"selection"`
	Quantum            float64      `json:"score_quantum"`
}

// SearchReport records how much of the search ran and whether a limit cut
// it short.
type SearchReport struct {
	orient.SearchStats
	// Finalists counts the orientations reported as candidates.
	Finalists       int `json:"finalists"`
	SupportAttempts int `json:"support_attempts"`
	// AttemptLimitReached is true when the attempt limit left a feasible
	// finalist unplanned.
	AttemptLimitReached bool `json:"support_attempt_limit_reached"`
}

// CandidateReport is one orientation: the original one or a search
// finalist.
type CandidateReport struct {
	ID     int    `json:"id"`
	Source string `json:"source"`
	// Rank is the 1-based position by estimated support cost.
	Rank int `json:"rank"`
	// Down is the input-frame direction that faces the plate.
	Down [3]float64 `json:"down"`
	// Rotation is the 3x3 rotation, row-major, applied before placement.
	Rotation    [3][3]float64  `json:"rotation"`
	FaceNormal  *[3]float64    `json:"face_normal,omitempty"`
	FaceAreaMM2 *float64       `json:"face_area_mm2,omitempty"`
	Metrics     orient.Metrics `json:"metrics"`
	// TiltAllowed is false when the long axis rises above the tilt limit.
	TiltAllowed bool `json:"tilt_allowed"`
	// Fit is "fits", "exceeds" or "unchecked" (no build volume).
	Fit string `json:"build_volume_fit"`
	// Estimate is the one-pass support cost estimate (see
	// docs/design.md); Planned is the cost of the planned pillars, set
	// only when supports were planned.
	Estimate orient.Cost   `json:"estimated_support_cost"`
	Planned  *orient.Cost  `json:"planned_support_cost,omitempty"`
	Attempt  AttemptReport `json:"support_attempt"`
}

// Attempt statuses.
const (
	AttemptSucceeded    = "succeeded"
	AttemptFailed       = "failed"
	AttemptNotAttempted = "not_attempted"
	AttemptNotNeeded    = "not_needed"
)

// AttemptReport is the outcome of building supports for one candidate.
type AttemptReport struct {
	Status         string           `json:"status"`
	Reason         string           `json:"reason,omitempty"`
	Supports       int              `json:"supports"`
	Samples        int              `json:"demand_samples"`
	BridgedSamples int              `json:"bridged_samples"`
	HeldSamples    int              `json:"wall_held_samples"`
	UncoveredCount int              `json:"uncovered_samples"`
	Uncovered      []support.Sample `json:"uncovered_first,omitempty"`
}

// TransformReport records the rigid transform from the input body to the
// output model.
//
// A point p of the input body maps to the output as q = R * p + t. Matrix is
// the 4x4 homogeneous form [R t; 0 0 0 1], stored row-major, that acts on
// column vectors (x, y, z, 1) of millimeter coordinates. Inverse is
// [R^T, -R^T t; 0 0 0 1].
type TransformReport struct {
	Matrix      [4][4]float64 `json:"matrix"`
	Inverse     [4][4]float64 `json:"inverse"`
	Translation [3]float64    `json:"translation_mm"`
	Convention  string        `json:"convention"`
}

// ModelReport describes the output model body, as decad measures it.
type ModelReport struct {
	BoundsMM  [2][3]float64 `json:"bounds_mm"`
	VolumeMM3 float64       `json:"volume_mm3"`
}

// SupportReport describes one support body.
type SupportReport struct {
	ID string `json:"id"`
	// Kind is "pillar" or "branch". A branch\'s foot stands at FootMM on
	// the plate; its vertical shaft ends at KneeZMM, where it leans toward
	// CenterMM.
	Kind       string        `json:"kind"`
	FootMM     *[2]float64   `json:"foot_mm,omitempty"`
	KneeZMM    *float64      `json:"knee_z_mm,omitempty"`
	CenterMM   [2]float64    `json:"center_mm"`
	SurfaceZMM float64       `json:"held_surface_z_mm"`
	TopZMM     float64       `json:"top_z_mm"`
	Origin     string        `json:"origin"`
	VolumeMM3  float64       `json:"volume_mm3"`
	BoundsMM   [2][3]float64 `json:"bounds_mm"`
}

// Unchecked names a property mtilt did not verify.
type Unchecked struct {
	Property string `json:"property"`
	Note     string `json:"note"`
}

func vec3(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }

func boxArray(b mesh.Box) [2][3]float64 { return [2][3]float64{vec3(b.Min), vec3(b.Max)} }

func rotationRows(t r3.Transform) [3][3]float64 {
	b := t.Basis()
	return [3][3]float64{
		{b.EX.X, b.EY.X, b.EZ.X},
		{b.EX.Y, b.EY.Y, b.EZ.Y},
		{b.EX.Z, b.EY.Z, b.EZ.Z},
	}
}

func homogeneous(t r3.Transform) [4][4]float64 {
	r := rotationRows(t)
	tr := t.Translation()
	// Adding 0 turns -0 into +0, so the JSON never prints "-0".
	for i := range r {
		for j := range r[i] {
			r[i][j] += 0
		}
	}
	tr = tr.Add(r3.Vec{})
	return [4][4]float64{
		{r[0][0], r[0][1], r[0][2], tr.X},
		{r[1][0], r[1][1], r[1][2], tr.Y},
		{r[2][0], r[2][1], r[2][2], tr.Z},
		{0, 0, 0, 1},
	}
}

func transformReport(t r3.Transform) (*TransformReport, error) {
	inv, err := t.Inverse()
	if err != nil {
		return nil, fmt.Errorf("mtilt: inverting transform: %w", err)
	}
	return &TransformReport{
		Matrix:      homogeneous(t),
		Inverse:     homogeneous(inv),
		Translation: vec3(t.Translation()),
		Convention:  "q = R * p + t; matrix is row-major [R t; 0 0 0 1] acting on column vectors (x, y, z, 1) in mm",
	}, nil
}
