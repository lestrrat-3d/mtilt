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

// Unit is the length unit of an input mesh's coordinates. STL files carry
// no unit, so the caller always names one.
type Unit string

// Supported input units.
const (
	UnitMillimeter Unit = "mm"
	UnitCentimeter Unit = "cm"
	UnitMeter      Unit = "m"
	UnitInch       Unit = "in"
)

// ToMillimeters returns the factor that converts a coordinate in u to
// millimeters, and false for an unknown unit.
func (u Unit) ToMillimeters() (float64, bool) {
	switch u {
	case UnitMillimeter:
		return 1, true
	case UnitCentimeter:
		return 10, true
	case UnitMeter:
		return 1000, true
	case UnitInch:
		return 25.4, true
	}
	return 0, false
}

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
	"base_width_mm", "base_thickness_mm",
}

// Validate checks the profile's values and their combinations. It returns
// ErrInvalidOptions naming the first rule that fails:
//
//   - every length is finite and positive, except side_clearance_mm, which
//     may be 0;
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
		ThresholdDeg:    p.OverhangThreshold,
		SpacingMM:       p.SupportSpacingMM,
		ContactWidthMM:  p.ContactWidthMM,
		PillarWidthMM:   p.PillarWidthMM,
		TipHeightMM:     p.TipHeightMM,
		BaseWidthMM:     p.BaseWidthMM,
		BaseThicknessMM: p.BaseThicknessMM,
		TopGapMM:        p.TopContactGapMM,
		SideClearanceMM: p.SideClearanceMM,
	}
}

// Weights are the orientation objective's term weights. See Objective.
type Weights = orient.Weights

// DefaultWeights returns the weights Options uses when none are set:
// support demand 1, height 0.1, bed contact 0.5.
func DefaultWeights() Weights {
	return Weights{SupportDemand: 1, Height: 0.1, BedContact: 0.5}
}

// Limits bound the work Prepare and Analyze do. A zero field means the
// default listed beside it.
type Limits struct {
	// MaxTriangles caps the input mesh size. Default 2,000,000.
	MaxTriangles int `json:"max_triangles"`
	// MaxCandidates caps the orientations evaluated. Default 64.
	MaxCandidates int `json:"max_candidates"`
	// MaxPlanarFaces caps the planar-face candidates. Default 12.
	MaxPlanarFaces int `json:"max_planar_faces"`
	// MaxSupportAttempts caps how many ranked candidates get a support
	// attempt. Default 8.
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
	def(&l.MaxCandidates, 64)
	def(&l.MaxPlanarFaces, 12)
	def(&l.MaxSupportAttempts, 8)
	def(&l.MaxSupports, 5_000)
	def(&l.MaxSamples, 500_000)
	return l
}

// Options configure Prepare and Analyze.
type Options struct {
	// Profile is required.
	Profile Profile
	// Weights default to DefaultWeights when all three are zero.
	Weights Weights
	// KeepOrientation evaluates only the input orientation. The model is
	// still translated onto the plate, and the translation is reported.
	KeepOrientation bool
	Limits          Limits
}

func (o Options) normalized() (Options, error) {
	if err := o.Profile.Validate(); err != nil {
		return Options{}, err
	}
	if o.Weights == (Weights{}) {
		o.Weights = DefaultWeights()
	}
	for _, w := range []float64{o.Weights.SupportDemand, o.Weights.Height, o.Weights.BedContact} {
		if !(w >= 0) || math.IsInf(w, 0) {
			return Options{}, fmt.Errorf("%w: weights must be finite and not negative", ErrInvalidOptions)
		}
	}
	o.Limits = o.Limits.withDefaults()
	return o, nil
}
