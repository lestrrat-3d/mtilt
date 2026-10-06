// Package mtilt orients a decad body for FDM printing and builds separate,
// sacrificial support bodies for it.
//
// Prepare takes a solid *decad.Body. It tessellates the body, evaluates
// candidate orientations, plans supports on the mesh for the best-ranked
// candidates until one succeeds, and then adds the moved model and one
// revolved pillar body per support to the body's document and verifies them
// with decad. Analyze stops after ranking. Both return a JSON-encodable
// report.
//
// Coordinates are millimeters with +Z as the build direction and the build
// plate at Z = 0. The model moves by one proper rigid transform (a rotation
// followed by a translation) and is never scaled, mirrored, remeshed or
// merged with a support.
//
// "Best" means best among the evaluated candidates under the documented
// objective and profile. Nothing in this package claims a global optimum,
// guaranteed printability, or tested breakaway behavior.
package mtilt
