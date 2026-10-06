// Command mtilt orients a triangle mesh for FDM printing and writes it with
// separate breakaway support bodies. Run `mtilt help` for usage.
package main

import (
	"context"
	"os"
	"os/signal"
	"syscall"

	"github.com/lestrrat-3d/mtilt/internal/cli"
)

func main() {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	code := cli.Run(ctx, os.Args[1:], os.Stdout, os.Stderr)
	stop()
	os.Exit(code)
}
