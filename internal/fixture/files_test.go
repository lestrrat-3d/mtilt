package fixture_test

import (
	"flag"
	"os"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/mtilt/internal/fixture"
	"github.com/stretchr/testify/require"
)

var update = flag.Bool("update", false, "rewrite testdata/ from the fixture builders")

// TestTestdataUpToDate checks that every file in testdata/ holds exactly the
// bytes the fixture builders produce. Run `go test ./internal/fixture -update`
// to regenerate them.
func TestTestdataUpToDate(t *testing.T) {
	dir := filepath.Join("..", "..", "testdata")
	for _, f := range fixture.Files() {
		path := filepath.Join(dir, f.Name)
		if *update {
			require.NoError(t, os.WriteFile(path, f.Data, 0o644))
			continue
		}
		got, err := os.ReadFile(path)
		require.NoError(t, err, "run `go test ./internal/fixture -update` to create %s", f.Name)
		require.Equal(t, f.Data, got, "%s is stale; run `go test ./internal/fixture -update`", f.Name)
	}
}
