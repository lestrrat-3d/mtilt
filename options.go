package mtilt

import (
	"bytes"
	_ "embed"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/mtilt/internal/orient"
	"github.com/lestrrat-3d/mtilt/internal/support"
)

// ErrInvalidOptions is returned when options or a profile fail validation.
var ErrInvalidOptions = errors.New("mtilt: invalid options")

// BuildVolume is the printable box. X and Y extents are centered on the
// origin; Z runs from the plate (0) up. Margin is kept clear on every side
// except the plate.
type BuildVolume struct {
	XMM      float64 `json:"x_mm"`
	YMM      float64 `json:"y_mm"`
	ZMM      float64 `json:"z_mm"`
	MarginMM float64 `json:"margin_mm"`
}

// Profile is the printer and process description supports are built for.
// Lengths are millimeters and angles degrees. Every field is required.
type Profile struct {
	// Name labels the profile in reports.
	Name string `json:"name"`
	// Calibrated must be false unless the values were checked by printing
	// with the intended printer, material and slicer. mtilt copies it into
	// every report and does nothing else with it.
	Calibrated bool `json:"calibrated"`

	NozzleDiameterMM  float64 `json:"nozzle_diameter_mm"`
	ExtrusionWidthMM  float64 `json:"extrusion_width_mm"`
	LayerHeightMM     float64 `json:"layer_height_mm"`
	MinFeatureMM      float64 `json:"min_feature_mm"`
	OverhangThreshold float64 `json:"overhang_threshold_deg"`

	SupportSpacingMM float64 `json:"support_spacing_mm"`
	TopContactGapMM  float64 `json:"top_contact_gap_mm"`
	SideClearanceMM  float64 `json:"side_clearance_mm"`
	ContactWidthMM   float64 `json:"contact_width_mm"`
	PillarWidthMM    float64 `json:"pillar_width_mm"`
	TipHeightMM      float64 `json:"tip_height_mm"`
	BaseWidthMM      float64 `json:"base_width_mm"`
	BaseThicknessMM  float64 `json:"base_thickness_mm"`

	// PlateAnchorMM is the height above the plate at or below which a
	// downward surface counts as printed from the plate's first layers and
	// needs no support. 0 supports everything above the plate.
	PlateAnchorMM float64 `json:"plate_anchor_height_mm"`

	// MinFirstLayerAreaMM2 is the smallest first-layer area (the model's
	// cross-section at the layer height) an orientation may have: less
	// than this holds too little of the part to the plate.
	MinFirstLayerAreaMM2 float64 `json:"min_first_layer_area_mm2"`

	// MaxBridgeMM is the longest span the printer draws in the air between
	// two walls; overhang on a shorter span needs no support. 0 turns
	// bridges off. MaxBridgeTiltDeg is the steepest tilt from horizontal a
	// bridged surface may have.
	MaxBridgeMM      float64 `json:"max_bridge_mm"`
	MaxBridgeTiltDeg float64 `json:"bridge_max_tilt_deg"`

	// MaxBranchLeanDeg is the steepest a branch support may lean from
	// vertical; it must leave the branch's own walls printable, so it is
	// at most 90 - overhang_threshold_deg. 0 turns branches off, leaving
	// straight pillars only.
	MaxBranchLeanDeg float64 `json:"max_branch_lean_deg"`

	// BuildVolume is optional. Without it, fit is reported as unchecked.
	BuildVolume *BuildVolume `json:"build_volume,omitempty"`
}

//go:embed profiles/example-fdm.json
var exampleProfile []byte

// ExampleProfile returns the illustrative profile shipped in
// profiles/example-fdm.json. Its values are not calibrated for any printer
// or material.
func ExampleProfile() Profile {
	p, err := DecodeProfile(bytes.NewReader(exampleProfile))
	if err != nil {
		panic(err) // the embedded file is checked by tests
	}
	return p
}

// DecodeProfile reads a JSON profile from r and validates it. Unknown fields
// and missing fields are errors.
func DecodeProfile(r io.Reader) (Profile, error) {
	var raw map[string]json.RawMessage
	data, err := io.ReadAll(io.LimitReader(r, 1<<20))
	if err != nil {
		return Profile{}, err
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return Profile{}, fmt.Errorf("%w: profile: %w", ErrInvalidOptions, err)
	}
	for _, k := range profileKeys {
		if _, ok := raw[k]; !ok {
			return Profile{}, fmt.Errorf("%w: profile: missing field %q", ErrInvalidOptions, k)
		}
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	var p Profile
	if err := dec.Decode(&p); err != nil {
		return Profile{}, fmt.Errorf("%w: profile: %w", ErrInvalidOptions, err)
	}
	if err := p.Validate(); err != nil {
		return Profile{}, err
	}
	return p, nil
}

// profileKeys are the JSON fields every profile must set.
var profileKeys = []string{
	"name", "calibrated", "nozzle_diameter_mm", "extrusion_width_mm", "layer_height_mm",
	"min_feature_mm", "overhang_threshold_deg", "support_spacing_mm", "top_contact_gap_mm",
	"side_clearance_mm", "contact_width_mm", "pillar_width_mm", "tip_height_mm",
	"base_width_mm", "base_thickness_mm", "plate_anchor_height_mm", "min_first_layer_area_mm2",
	"max_bridge_mm", "bridge_max_tilt_deg", "max_branch_lean_deg",
}

// Validate checks the profile's values and their combinations. It returns
// ErrInvalidOptions naming the first rule that fails:
//
//   - every length is finite and positive, except side_clearance_mm,
//     plate_anchor_height_mm, min_first_layer_area_mm2 and max_bridge_mm,
//     which may be 0;
//   - bridge_max_tilt_deg is in [0, overhang_threshold_deg);
//   - overhang_threshold_deg is in (0, 90);
//   - layer_height_mm < nozzle_diameter_mm <= extrusion_width_mm <=
//     min_feature_mm <= contact_width_mm <= pillar_width_mm <= base_width_mm;
//   - base_thickness_mm and tip_height_mm are at least layer_height_mm;
//   - base_width_mm < support_spacing_mm, so grid pillars never share a base;
//   - a build volume, when given, has positive extents and a non-negative
//     margin smaller than half of each horizontal extent.
func (p Profile) Validate() error {
	fail := func(format string, args ...any) error {
		return fmt.Errorf("%w: profile: %s", ErrInvalidOptions, fmt.Sprintf(format, args...))
	}
	positive := []struct {
		name string
		v    float64
	}{
		{"nozzle_diameter_mm", p.NozzleDiameterMM}, {"extrusion_width_mm", p.ExtrusionWidthMM},
		{"layer_height_mm", p.LayerHeightMM}, {"min_feature_mm", p.MinFeatureMM},
		{"support_spacing_mm", p.SupportSpacingMM}, {"top_contact_gap_mm", p.TopContactGapMM},
		{"contact_width_mm", p.ContactWidthMM}, {"pillar_width_mm", p.PillarWidthMM},
		{"tip_height_mm", p.TipHeightMM}, {"base_width_mm", p.BaseWidthMM},
		{"base_thickness_mm", p.BaseThicknessMM},
	}
	for _, f := range positive {
		if !(f.v > 0) || math.IsInf(f.v, 0) {
			return fail("%s must be a positive finite number, got %v", f.name, f.v)
		}
	}
	if !(p.SideClearanceMM >= 0) || math.IsInf(p.SideClearanceMM, 0) {
		return fail("side_clearance_mm must be finite and not negative, got %v", p.SideClearanceMM)
	}
	if !(p.PlateAnchorMM >= 0) || math.IsInf(p.PlateAnchorMM, 0) {
		return fail("plate_anchor_height_mm must be finite and not negative, got %v", p.PlateAnchorMM)
	}
	if !(p.MinFirstLayerAreaMM2 >= 0) || math.IsInf(p.MinFirstLayerAreaMM2, 0) {
		return fail("min_first_layer_area_mm2 must be finite and not negative, got %v", p.MinFirstLayerAreaMM2)
	}
	if !(p.MaxBridgeMM >= 0) || math.IsInf(p.MaxBridgeMM, 0) {
		return fail("max_bridge_mm must be finite and not negative, got %v", p.MaxBridgeMM)
	}
	if !(p.MaxBranchLeanDeg >= 0 && p.MaxBranchLeanDeg <= 90-p.OverhangThreshold) {
		return fail("max_branch_lean_deg must be between 0 and 90 - overhang_threshold_deg, got %v", p.MaxBranchLeanDeg)
	}
	if !(p.MaxBridgeTiltDeg >= 0 && p.MaxBridgeTiltDeg < p.OverhangThreshold) {
		return fail("bridge_max_tilt_deg must be at least 0 and below overhang_threshold_deg, got %v", p.MaxBridgeTiltDeg)
	}
	if !(p.OverhangThreshold > 0 && p.OverhangThreshold < 90) {
		return fail("overhang_threshold_deg must be between 0 and 90, got %v", p.OverhangThreshold)
	}
	chain := []struct {
		name string
		v    float64
	}{
		{"nozzle_diameter_mm", p.NozzleDiameterMM}, {"extrusion_width_mm", p.ExtrusionWidthMM},
		{"min_feature_mm", p.MinFeatureMM}, {"contact_width_mm", p.ContactWidthMM},
		{"pillar_width_mm", p.PillarWidthMM}, {"base_width_mm", p.BaseWidthMM},
	}
	if p.LayerHeightMM >= p.NozzleDiameterMM {
		return fail("layer_height_mm (%v) must be below nozzle_diameter_mm (%v)", p.LayerHeightMM, p.NozzleDiameterMM)
	}
	for i := 1; i < len(chain); i++ {
		if chain[i].v < chain[i-1].v {
			return fail("%s (%v) must be at least %s (%v)", chain[i].name, chain[i].v, chain[i-1].name, chain[i-1].v)
		}
	}
	if p.BaseThicknessMM < p.LayerHeightMM {
		return fail("base_thickness_mm (%v) must be at least layer_height_mm (%v)", p.BaseThicknessMM, p.LayerHeightMM)
	}
	if p.TipHeightMM < p.LayerHeightMM {
		return fail("tip_height_mm (%v) must be at least layer_height_mm (%v)", p.TipHeightMM, p.LayerHeightMM)
	}
	if p.BaseWidthMM >= p.SupportSpacingMM {
		return fail("base_width_mm (%v) must be below support_spacing_mm (%v)", p.BaseWidthMM, p.SupportSpacingMM)
	}
	if v := p.BuildVolume; v != nil {
		for _, f := range []float64{v.XMM, v.YMM, v.ZMM} {
			if !(f > 0) || math.IsInf(f, 0) {
				return fail("build_volume extents must be positive finite numbers")
			}
		}
		if !(v.MarginMM >= 0) || 2*v.MarginMM >= math.Min(v.XMM, v.YMM) {
			return fail("build_volume margin_mm must be non-negative and below half of x_mm and y_mm")
		}
	}
	return nil
}

func (p Profile) supportParams() support.Params {
	return support.Params{
		ThresholdDeg:     p.OverhangThreshold,
		SpacingMM:        p.SupportSpacingMM,
		ContactWidthMM:   p.ContactWidthMM,
		PillarWidthMM:    p.PillarWidthMM,
		TipHeightMM:      p.TipHeightMM,
		BaseWidthMM:      p.BaseWidthMM,
		BaseThicknessMM:  p.BaseThicknessMM,
		TopGapMM:         p.TopContactGapMM,
		SideClearanceMM:  p.SideClearanceMM,
		PlateAnchorMM:    p.PlateAnchorMM,
		LayerHeightMM:    p.LayerHeightMM,
		MaxBridgeMM:      p.MaxBridgeMM,
		MaxBridgeTiltDeg: p.MaxBridgeTiltDeg,
		MaxLeanDeg:       p.MaxBranchLeanDeg,
	}
}

// CostWeights weight support volume against support contact area in the
// support cost (see Report.Objective).
type CostWeights = orient.CostWeights

// DefaultCostWeights returns the weights Options uses when both are zero:
// volume 1, contact 1.
func DefaultCostWeights() CostWeights {
	return CostWeights{Volume: 1, Contact: 1}
}

// DefaultMaxLongAxisTiltDeg is the long-axis tilt limit Options uses when
// MaxLongAxisTiltDeg is 0.
const DefaultMaxLongAxisTiltDeg = 15

// Limits bound the work Prepare and Analyze do. A zero field means the
// default listed beside it.
type Limits struct {
	// MaxTriangles caps the tessellation of the input body. Default
	// 2,000,000.
	MaxTriangles int `json:"max_triangles"`
	// SweepDirections is the number of evenly spread down directions the
	// search estimates. Default 2,000.
	SweepDirections int `json:"sweep_directions"`
	// MaxPlanarFaces caps the face-down seed directions. Default 12.
	MaxPlanarFaces int `json:"max_planar_faces"`
	// MaxSupportAttempts is the number of finalist orientations that get a
	// support plan. Default 8.
	MaxSupportAttempts int `json:"max_support_attempts"`
	// MaxSupports caps the pillars in one attempt. Default 5,000.
	MaxSupports int `json:"max_supports"`
	// MaxSamples caps the support-demand samples in one attempt. Default
	// 500,000.
	MaxSamples int `json:"max_samples"`
}

func (l Limits) withDefaults() Limits {
	def := func(v *int, d int) {
		if *v <= 0 {
			*v = d
		}
	}
	def(&l.MaxTriangles, 2_000_000)
	def(&l.SweepDirections, 2_000)
	def(&l.MaxPlanarFaces, 12)
	def(&l.MaxSupportAttempts, 8)
	def(&l.MaxSupports, 5_000)
	def(&l.MaxSamples, 500_000)
	return l
}

// DefaultChordToleranceMM is the tessellation chord tolerance Options uses
// when ChordToleranceMM is 0.
const DefaultChordToleranceMM = 0.01

// Options configure Prepare and Analyze.
type Options struct {
	// Profile is required.
	Profile Profile
	// MaxLongAxisTiltDeg is the largest angle, in degrees, between a long
	// part's long axis and the build plate. Strength comes first: no
	// orientation over the limit is chosen, however little support it
	// needs. It applies only to parts with a long axis (elongation at
	// least 0.1). 0 means DefaultMaxLongAxisTiltDeg; 90 turns the limit
	// off.
	MaxLongAxisTiltDeg float64
	// CostWeights default to DefaultCostWeights when both are zero.
	CostWeights CostWeights
	// ChordToleranceMM is the chord tolerance the body is tessellated with
	// for planning. Default DefaultChordToleranceMM. The bound decad proves
	// for the tessellation is added to every clearance mtilt checks on it.
	ChordToleranceMM float64
	// KeepOrientation evaluates only the input orientation. The model is
	// still translated onto the plate, and the translation is reported.
	// The tilt limit does not apply.
	KeepOrientation bool
	Limits          Limits
}

func (o Options) normalized() (Options, error) {
	if err := o.Profile.Validate(); err != nil {
		return Options{}, err
	}
	if o.CostWeights == (CostWeights{}) {
		o.CostWeights = DefaultCostWeights()
	}
	for _, w := range []float64{o.CostWeights.Volume, o.CostWeights.Contact} {
		if !(w >= 0) || math.IsInf(w, 0) {
			return Options{}, fmt.Errorf("%w: cost weights must be finite and not negative", ErrInvalidOptions)
		}
	}
	if o.MaxLongAxisTiltDeg == 0 {
		o.MaxLongAxisTiltDeg = DefaultMaxLongAxisTiltDeg
	}
	if !(o.MaxLongAxisTiltDeg > 0 && o.MaxLongAxisTiltDeg <= 90) {
		return Options{}, fmt.Errorf("%w: long-axis tilt limit must be in (0, 90] degrees", ErrInvalidOptions)
	}
	if o.ChordToleranceMM == 0 {
		o.ChordToleranceMM = DefaultChordToleranceMM
	}
	if !(o.ChordToleranceMM > 0) || math.IsInf(o.ChordToleranceMM, 0) {
		return Options{}, fmt.Errorf("%w: chord tolerance must be positive and finite", ErrInvalidOptions)
	}
	o.Limits = o.Limits.withDefaults()
	return o, nil
}
