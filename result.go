package mtilt

import (
	"errors"
	"fmt"

	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
	"github.com/lestrrat-3d/mtilt/mesh"
	"github.com/lestrrat-3d/r3"
)

// Version identifiers recorded in every report.
const (
	// SchemaVersion is the version of the Report JSON layout.
	SchemaVersion = 1
	// Version is the mtilt release.
	Version = "0.1.0-dev"
	// AlgorithmVersion names the orientation search and the support
	// strategy. It changes whenever either can produce different output
	// for the same input and options.
	AlgorithmVersion = "orient-1/pillar-1"
)

// Errors Prepare and Analyze return. Failures that carry a report are
// wrapped in *FailureError.
var (
	// ErrInvalidMesh reports an input mesh that fails a validation check.
	ErrInvalidMesh = errors.New("mtilt: invalid mesh")
	// ErrUnsupportedInput reports a valid-looking input outside the
	// supported topology: more than one connected component (including
	// nested shells).
	ErrUnsupportedInput = errors.New("mtilt: unsupported input")
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
}

func (e *FailureError) Error() string { return e.Err.Error() }
func (e *FailureError) Unwrap() error { return e.Err }

// Result is a successful preparation.
type Result struct {
	// Model is the input model converted to millimeters and moved by
	// Transform.
	Model *mesh.Mesh
	// Supports are the support bodies, in the same frame as Model, in
	// Report.Supports order.
	Supports []*mesh.Mesh
	// Transform maps the input, after unit conversion, to the output frame.
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

// Source describes where an input mesh came from. The library copies it into
// reports and does not interpret it.
type Source struct {
	SHA256             string `json:"sha256,omitempty"`
	Format             string `json:"format,omitempty"`
	Name               string `json:"name,omitempty"`
	NormalsDisagreeing int    `json:"stored_normals_disagreeing"`
}

// InputReport records the input and its unit conversion.
type InputReport struct {
	Source
	Triangles      int           `json:"triangles"`
	Vertices       int           `json:"vertices"`
	Unit           Unit          `json:"unit"`
	ScaleToMM      float64       `json:"scale_to_mm"`
	SurfaceAreaMM2 float64       `json:"surface_area_mm2"`
	VolumeMM3      float64       `json:"volume_mm3"`
	BoundsMM       [2][3]float64 `json:"bounds_mm"`
	Components     int           `json:"components"`
}

// ToleranceReport records the numeric tolerances used (see mesh.Tolerance).
type ToleranceReport struct {
	LengthMM        float64 `json:"length_mm"`
	SerializationMM float64 `json:"serialization_mm"`
	PlateMM         float64 `json:"plate_mm"`
}

// ObjectiveReport records how candidates were scored and ordered.
type ObjectiveReport struct {
	Weights  Weights      `json:"weights"`
	Scale    orient.Scale `json:"normalization"`
	Formula  string       `json:"formula"`
	TieBreak string       `json:"tie_break"`
	Quantum  float64      `json:"score_quantum"`
}

// SearchReport records how much of the search ran and whether a limit cut
// it short.
type SearchReport struct {
	CandidatesDistinct    int  `json:"candidates_distinct"`
	CandidatesEvaluated   int  `json:"candidates_evaluated"`
	CandidateLimitReached bool `json:"candidate_limit_reached"`
	SupportAttempts       int  `json:"support_attempts"`
	// AttemptLimitReached is true when the attempt limit stopped the
	// search before every feasible candidate was tried and none had
	// succeeded.
	AttemptLimitReached bool `json:"support_attempt_limit_reached"`
}

// CandidateReport is one evaluated orientation.
type CandidateReport struct {
	ID     int    `json:"id"`
	Source string `json:"source"`
	// Rank is the 1-based position in the ranking.
	Rank int `json:"rank"`
	// Rotation is the 3x3 rotation, row-major, applied before placement.
	Rotation    [3][3]float64  `json:"rotation"`
	FaceNormal  *[3]float64    `json:"face_normal,omitempty"`
	FaceAreaMM2 *float64       `json:"face_area_mm2,omitempty"`
	Metrics     orient.Metrics `json:"metrics"`
	// Fit is "fits", "exceeds" or "unchecked" (no build volume).
	Fit     string        `json:"build_volume_fit"`
	Terms   orient.Terms  `json:"score_terms"`
	Score   float64       `json:"score"`
	Attempt AttemptReport `json:"support_attempt"`
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
	UncoveredCount int              `json:"uncovered_samples"`
	Uncovered      []support.Sample `json:"uncovered_first,omitempty"`
}

// TransformReport records the rigid transform from the unit-converted input
// to the output frame.
//
// A point p of the input file maps to the output as
//
//	q = R * (ScaleToMM * p) + t
//
// Matrix is the 4x4 homogeneous form [R t; 0 0 0 1], stored row-major, that
// acts on column vectors (x, y, z, 1) of millimeter coordinates. Inverse is
// [R^T, -R^T t; 0 0 0 1]; applying it to an output point and dividing by
// ScaleToMM returns the input point.
type TransformReport struct {
	ScaleToMM   float64       `json:"scale_to_mm"`
	Matrix      [4][4]float64 `json:"matrix"`
	Inverse     [4][4]float64 `json:"inverse"`
	Translation [3]float64    `json:"translation_mm"`
	Convention  string        `json:"convention"`
}

// ModelReport describes the output model.
type ModelReport struct {
	File      string        `json:"file,omitempty"`
	BoundsMM  [2][3]float64 `json:"bounds_mm"`
	VolumeMM3 float64       `json:"volume_mm3"`
}

// SupportReport describes one support body.
type SupportReport struct {
	ID         string        `json:"id"`
	File       string        `json:"file,omitempty"`
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

func transformReport(t r3.Transform, scale float64) (*TransformReport, error) {
	inv, err := t.Inverse()
	if err != nil {
		return nil, fmt.Errorf("mtilt: inverting transform: %w", err)
	}
	return &TransformReport{
		ScaleToMM:   scale,
		Matrix:      homogeneous(t),
		Inverse:     homogeneous(inv),
		Translation: vec3(t.Translation()),
		Convention:  "q = R * (scale_to_mm * p) + t; matrix is row-major [R t; 0 0 0 1] acting on column vectors (x, y, z, 1) in mm",
	}, nil
}
