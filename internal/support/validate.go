package support

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
)

// Names of the checks ValidateAssembly and RecheckPlan report.
const (
	CheckSupportTopology     = "support_topology"
	CheckSupportOnPlate      = "support_on_plate"
	CheckSupportModelContact = "support_model_contact"
	CheckSupportTopGap       = "support_top_gap"
	CheckSupportSeparation   = "support_separation"
	CheckClearanceZones      = "support_clearance_zones"
	CheckCoverage            = "support_coverage"
)

// ValidateAssembly checks support meshes against a placed model using only
// the meshes, so it can run again on meshes read back from files. allowance
// is the extra distance the length checks tolerate: 0 for in-memory
// meshes, the serialization tolerance for re-read ones.
//
//   - support_topology: every support passes mesh.Validate with no failed
//     check (one closed, manifold, outward-wound component).
//   - support_on_plate: every support's lowest Z is 0 within allowance, and
//     so its single connected body reaches the plate.
//   - support_model_contact: no support triangle touches a model triangle.
//   - support_top_gap: the model's lowest point over each support's top
//     face is at least gap above that face, less allowance.
//   - support_separation: the XY bounding boxes of the supports are
//     pairwise disjoint. A pillar lies inside the vertical prism over its
//     XY bounding box, so disjoint boxes mean disjoint bodies.
func ValidateAssembly(ctx context.Context, model *mesh.Mesh, supports []*mesh.Mesh, gap, allowance float64, tol mesh.Tolerance) ([]mesh.Check, error) {
	ix := newIndex(model)
	var topology, plate, contact, topGap []string
	boxes := make([]mesh.Box, len(supports))
	for i, s := range supports {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		name := fmt.Sprintf("support %d", i+1)
		rep, err := mesh.Validate(ctx, s, tol)
		if err != nil {
			return nil, err
		}
		if failed := rep.Failed(); len(failed) > 0 {
			topology = append(topology, fmt.Sprintf("%s: %s (%s)", name, failed[0].Name, failed[0].Detail))
		}
		b := s.Bounds()
		boxes[i] = b
		if math.Abs(b.Min.Z) > allowance+tol.Length {
			plate = append(plate, fmt.Sprintf("%s: lowest Z is %g", name, b.Min.Z))
		}
		for _, t := range s.Soup() {
			if ti, hit := ix.anyTouching(triangleConvex(t), tol.Length); hit {
				contact = append(contact, fmt.Sprintf("%s touches model triangle %d", name, ti))
				break
			}
		}
		top := b.Max.Z
		x0, y0, x1, y1 := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
		for _, v := range s.Vertices {
			if v.Z < top-allowance-tol.Length {
				continue
			}
			x0, y0 = math.Min(x0, v.X), math.Min(y0, v.Y)
			x1, y1 = math.Max(x1, v.X), math.Max(y1, v.Y)
		}
		half := math.Max(x1-x0, y1-y0) / 2
		above := ix.lowestOver((x0+x1)/2, (y0+y1)/2, half)
		if above-top < gap-allowance-tol.Length {
			topGap = append(topGap, fmt.Sprintf("%s: %g mm below the model, needs %g", name, above-top, gap))
		}
	}

	order := make([]int, len(boxes))
	for i := range order {
		order[i] = i
	}
	slices.SortFunc(order, func(a, b int) int { return cmp.Compare(boxes[a].Min.X, boxes[b].Min.X) })
	var overlap []string
	for oi, i := range order {
		for _, j := range order[oi+1:] {
			if boxes[j].Min.X > boxes[i].Max.X {
				break
			}
			if boxes[j].Min.Y <= boxes[i].Max.Y && boxes[i].Min.Y <= boxes[j].Max.Y {
				overlap = append(overlap, fmt.Sprintf("supports %d and %d", min(i, j)+1, max(i, j)+1))
			}
		}
	}

	return []mesh.Check{
		check(CheckSupportTopology, topology),
		check(CheckSupportOnPlate, plate),
		check(CheckSupportModelContact, contact),
		check(CheckSupportTopGap, topGap),
		check(CheckSupportSeparation, overlap),
	}, nil
}

// RecheckPlan re-runs the construction-time checks on a finished plan
// against the placed model: every pillar's clearance zones are clear of the
// model, and every demand sample is covered under the coverage rule (see
// Build).
func RecheckPlan(placed *mesh.Mesh, plan *Plan, p Params, tol mesh.Tolerance) []mesh.Check {
	b := &builder{
		ix:    newIndex(placed),
		p:     p,
		eps:   tol.Length,
		dzMax: p.SpacingMM * math.Max(1, math.Tan(p.ThresholdDeg*math.Pi/180)),
		cells: make(map[[2]int64][]int),
	}
	b.lim.MaxSupports = len(plan.Pillars)
	var zones []string
	for i, pl := range plan.Pillars {
		for _, z := range p.clearanceZones(pl) {
			if ti, hit := b.ix.anyTouching(z, b.eps); hit {
				zones = append(zones, fmt.Sprintf("support %d zone touches model triangle %d", i+1, ti))
				break
			}
		}
		_ = b.add(pl)
	}
	var gaps []string
	for _, s := range plan.samples {
		if !b.covered(s) {
			gaps = append(gaps, fmt.Sprintf("sample %v", s))
		}
	}
	return []mesh.Check{check(CheckClearanceZones, zones), check(CheckCoverage, gaps)}
}

func check(name string, problems []string) mesh.Check {
	if len(problems) == 0 {
		return mesh.Check{Name: name, Status: mesh.StatusPassed}
	}
	detail := fmt.Sprintf("%d problem(s), first: %s", len(problems), problems[0])
	return mesh.Check{Name: name, Status: mesh.StatusFailed, Detail: detail}
}
