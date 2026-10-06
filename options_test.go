package mtilt_test

import (
	"encoding/json"
	"strings"
	"testing"

	"github.com/lestrrat-3d/mtilt"
	"github.com/stretchr/testify/require"
)

func TestProfile(t *testing.T) {
	ex := mtilt.ExampleProfile()
	require.False(t, ex.Calibrated)
	require.NoError(t, ex.Validate())
	require.InDelta(t, 1.2, ex.PlateAnchorMM, 0)

	encoded, err := json.Marshal(ex)
	require.NoError(t, err)

	t.Run("round trip", func(t *testing.T) {
		p, err := mtilt.DecodeProfile(strings.NewReader(string(encoded)))
		require.NoError(t, err)
		require.Equal(t, ex, p)
	})

	t.Run("missing field", func(t *testing.T) {
		var raw map[string]any
		require.NoError(t, json.Unmarshal(encoded, &raw))
		delete(raw, "plate_anchor_height_mm")
		b, err := json.Marshal(raw)
		require.NoError(t, err)
		_, err = mtilt.DecodeProfile(strings.NewReader(string(b)))
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
		require.ErrorContains(t, err, "plate_anchor_height_mm")
	})

	t.Run("unknown field", func(t *testing.T) {
		b := strings.Replace(string(encoded), "{", `{"support_magic": 1,`, 1)
		_, err := mtilt.DecodeProfile(strings.NewReader(b))
		require.ErrorIs(t, err, mtilt.ErrInvalidOptions)
	})

	rules := map[string]func(*mtilt.Profile){
		"contact narrower than min feature": func(p *mtilt.Profile) { p.ContactWidthMM = p.MinFeatureMM / 2 },
		"base as wide as spacing":           func(p *mtilt.Profile) { p.BaseWidthMM = p.SupportSpacingMM },
		"layer as tall as nozzle":           func(p *mtilt.Profile) { p.LayerHeightMM = p.NozzleDiameterMM },
		"threshold of 90":                   func(p *mtilt.Profile) { p.OverhangThreshold = 90 },
		"negative anchor":                   func(p *mtilt.Profile) { p.PlateAnchorMM = -1 },
		"negative bridge":                   func(p *mtilt.Profile) { p.MaxBridgeMM = -1 },
		"bridge tilt at the threshold":      func(p *mtilt.Profile) { p.MaxBridgeTiltDeg = p.OverhangThreshold },
		"zero gap":                          func(p *mtilt.Profile) { p.TopContactGapMM = 0 },
		"margin too large":                  func(p *mtilt.Profile) { p.BuildVolume = &mtilt.BuildVolume{XMM: 10, YMM: 10, ZMM: 10, MarginMM: 5} },
	}
	for name, breakIt := range rules {
		t.Run(name, func(t *testing.T) {
			p := mtilt.ExampleProfile()
			breakIt(&p)
			require.ErrorIs(t, p.Validate(), mtilt.ErrInvalidOptions)
		})
	}
}
