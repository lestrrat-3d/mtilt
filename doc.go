// Package mtilt orients a triangle mesh for FDM printing and builds
// separate, sacrificial support bodies for it.
//
// The pipeline is: validate the mesh, evaluate candidate orientations, build
// supports for the best-ranked candidates until one succeeds, validate the
// resulting assembly, and report. Prepare runs all of it; Analyze stops after
// ranking. Both take a mesh.Mesh, not a file: package stl reads and writes
// files, and cmd/mtilt is the command-line front end.
//
// Coordinates are float64 millimeters in a right-handed frame with +Z as the
// build direction and the build plate at Z = 0. Input coordinates are
// converted from the caller's Unit exactly once. The model is then moved by
// one proper rigid transform (a rotation followed by a translation), and the
// supports are built in that same frame. The model is never scaled again,
// mirrored, remeshed or merged with a support.
//
// "Best" means best among the evaluated candidates under the documented
// objective and profile. Nothing in this package claims a global optimum,
// guaranteed printability, or tested breakaway behavior.
package mtilt
