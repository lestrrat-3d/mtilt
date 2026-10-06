package orient

import (
	"cmp"
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/mtilt/internal/mesh"
	"github.com/lestrrat-3d/r3"
)

// Estimator estimates the support cost of a down direction in one pass over
// the triangles, without placing the mesh or planning supports.
//
// For down direction d, a point p sits at height h(p) = -p.d - min over
// vertices of (-v.d). A triangle with unit normal n needs support when
// n.d > cos(threshold) (it faces down at an overhang angle below the
// threshold), unless all its vertices are within the plate tolerance of the
// plate (bed contact) or at or below the plate-anchor height (anchored). Each
// such triangle adds its projected area A (n.d) to the contact estimate and
// A (n.d) times its mean vertex height to the volume estimate: the volume of
// the vertical column under it down to the plate.
//
// A counted triangle with any vertex below the shortest pillar's reach adds
// its projected area to TooLowMM2: the planner cannot support it.
//
// The volume estimate is an upper bound for plate-rooted supports only when
// nothing of the model lies under the triangle. Where something does, the
// column overlaps the model, and the planner will report that demand as
// occluded; the estimate does not detect it.
type Estimator struct {
	normals []r3.Vec
	areas   []float64
	tris    [][3]r3.Vec
	verts   []r3.Vec

	mesh         *mesh.Mesh
	cosThreshold float64
	anchor       float64
	layer        float64
	minPillar    float64
	plateTol     float64
	weights      CostWeights
	scale        Scale
}

// NewEstimator prepares m for repeated estimates. layerMM is the layer
// height, at which FirstLayer cuts the model; minPillarMM is the lowest
// surface height a pillar fits under (shortest pillar plus top gap).
func NewEstimator(m *mesh.Mesh, thresholdDeg, anchorMM, layerMM, minPillarMM float64, tol mesh.Tolerance, w CostWeights, s Scale) *Estimator {
	e := &Estimator{
		minPillar:    minPillarMM,
		mesh:         m,
		layer:        layerMM,
		normals:      make([]r3.Vec, len(m.Triangles)),
		areas:        make([]float64, len(m.Triangles)),
		tris:         m.Soup(),
		verts:        m.Vertices,
		cosThreshold: math.Cos(thresholdDeg * math.Pi / 180),
		anchor:       anchorMM,
		plateTol:     tol.Plate,
		weights:      w,
		scale:        s,
	}
	for i, t := range e.tris {
		e.normals[i], _ = mesh.TriangleNormal(t)
		e.areas[i] = mesh.TriangleArea(t)
	}
	return e
}

// Estimate returns the estimated support cost of down direction d (a unit
// vector).
func (e *Estimator) Estimate(d r3.Vec) Cost {
	base := math.Inf(1)
	for _, v := range e.verts {
		base = math.Min(base, -v.Dot(d))
	}
	var vol, contact, tooLow float64
	for i, n := range e.normals {
		c := n.Dot(d)
		if c <= e.cosThreshold {
			continue
		}
		t := e.tris[i]
		h0, h1, h2 := -t[0].Dot(d)-base, -t[1].Dot(d)-base, -t[2].Dot(d)-base
		hmax := math.Max(h0, math.Max(h1, h2))
		if hmax <= e.plateTol || hmax <= e.anchor {
			continue
		}
		proj := e.areas[i] * c
		contact += proj
		vol += proj * (h0 + h1 + h2) / 3
		if math.Min(h0, math.Min(h1, h2)) < e.minPillar {
			tooLow += proj
		}
	}
	cost := NewCost(vol, contact, e.weights, e.scale)
	cost.TooLowMM2 = tooLow
	return cost
}

// FirstLayer returns the first-layer area, in mm^2, of down direction d:
// the model's cross-section one layer height above its lowest point along
// -d.
func (e *Estimator) FirstLayer(d r3.Vec) float64 {
	up := d.Scale(-1)
	base := math.Inf(1)
	for _, v := range e.verts {
		base = math.Min(base, v.Dot(up))
	}
	return e.mesh.SectionArea(up, base+e.layer)
}

// SearchOptions configure Search.
type SearchOptions struct {
	// Directions is the number of directions in the even sweep.
	Directions int
	// MaxPlanarFaces caps the face-down seed directions.
	MaxPlanarFaces int
	// Finalists is the number of distinct best directions refined and
	// returned.
	Finalists int
	// MaxTiltDeg is the largest allowed long-axis elevation. It applies
	// only when the principal elongation is at least MinElongation.
	MaxTiltDeg float64
	// MinFirstLayerMM2 is the smallest allowed first-layer area.
	MinFirstLayerMM2 float64
}

// MinElongation is the Principal.Elongation below which a body has no long
// axis, and the tilt limit does not apply.
const MinElongation = 0.1

// Refinement constants. See Search.
const (
	// finalistSeparationDeg is the smallest angle between two finalists.
	finalistSeparationDeg = 1.0
	// refineMinStepDeg ends refinement.
	refineMinStepDeg = 0.25
	// refineNeighbors is the number of directions tried around a point
	// at each step.
	refineNeighbors = 8
)

// SearchStats reports what Search did.
type SearchStats struct {
	// Constrained is true when the tilt limit applied.
	Constrained bool `json:"tilt_limited"`
	// Seeds counts the original, axis and face-down directions.
	Seeds int `json:"seed_directions"`
	// Swept counts the sweep directions.
	Swept int `json:"sweep_directions"`
	// Allowed counts seeds and sweep directions within the tilt limit,
	// with the minimum first-layer area and no too-low overhang.
	Allowed int `json:"allowed_directions"`
	// Refined counts finalists that refinement moved.
	Refined int `json:"refined_finalists"`
}

// Search finds the down directions with the lowest estimated support cost.
//
//  1. Seeds: the original orientation (down = -Z), the six coordinate
//     directions, and the normals of the opts.MaxPlanarFaces largest planar
//     face clusters (a face put exactly on the plate).
//  2. Sweep: opts.Directions directions spread evenly over the sphere (a
//     Fibonacci lattice).
//  3. Every seed and sweep direction within the tilt limit whose first-layer
//     area (Estimator.FirstLayer) is at least opts.MinFirstLayerMM2 is
//     estimated, and kept when its estimate has no TooLowMM2. Directions are ranked by estimate (ties: lower elevation,
//     then generation order), and the best opts.Finalists that are at least
//     1 degree apart are kept.
//  4. Each finalist is refined: its refineNeighbors neighbors at a step
//     equal to the sweep spacing are estimated, and the best one that meets
//     the same limits and is strictly cheaper replaces it; the step halves
//     whenever no neighbor is cheaper, down to 0.25 degrees.
//
// Search returns the original orientation first (its Allowed reflects the
// tilt limit only) and then the finalists by ascending estimate. IDs follow
// that order. A refined direction is never worse than the one it
// started from, but the result is the best found at this resolution, not a
// proven optimum.
func Search(ctx context.Context, m *mesh.Mesh, pr Principal, est *Estimator, opts SearchOptions) ([]Candidate, SearchStats, error) {
	var stats SearchStats
	stats.Constrained = pr.Elongation >= MinElongation && opts.MaxTiltDeg < 90
	maxSin := math.Sin(opts.MaxTiltDeg * math.Pi / 180)
	elevation := func(d r3.Vec) float64 {
		return math.Asin(math.Min(1, math.Abs(pr.Axes[0].Dot(d)))) * 180 / math.Pi
	}
	tiltOK := func(d r3.Vec) bool {
		return !stats.Constrained || math.Abs(pr.Axes[0].Dot(d)) <= maxSin
	}
	allowed := func(d r3.Vec) bool {
		return tiltOK(d) && est.FirstLayer(d) >= opts.MinFirstLayerMM2
	}
	supportable := func(c Cost) bool { return c.TooLowMM2 == 0 }

	type dir struct {
		d      r3.Vec
		source Source
		face   cluster
		cost   Cost
	}
	var seeds []dir
	seeds = append(seeds, dir{d: r3.NewVec(0, 0, -1), source: SourceOriginal})
	for _, a := range []r3.Vec{
		r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0),
		r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1),
	} {
		seeds = append(seeds, dir{d: a, source: SourceAxis})
	}
	clusters, err := planarClusters(ctx, m)
	if err != nil {
		return nil, SearchStats{}, err
	}
	for i, c := range clusters {
		if i == opts.MaxPlanarFaces {
			break
		}
		seeds = append(seeds, dir{d: c.normal, source: SourcePlanarFace, face: c})
	}
	stats.Seeds = len(seeds)
	sweep := fibonacciSphere(opts.Directions)
	stats.Swept = len(sweep)

	all := slices.Clone(seeds)
	for _, d := range sweep {
		all = append(all, dir{d: d, source: SourceSweep})
	}
	var pool []dir
	for i, s := range all {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, SearchStats{}, err
			}
		}
		if !allowed(s.d) {
			continue
		}
		s.cost = est.Estimate(s.d)
		if !supportable(s.cost) {
			continue
		}
		pool = append(pool, s)
	}
	stats.Allowed = len(pool)

	order := make([]int, len(pool))
	for i := range order {
		order[i] = i
	}
	slices.SortStableFunc(order, func(a, b int) int {
		qa, qb := math.Round(pool[a].cost.Score/ScoreQuantum), math.Round(pool[b].cost.Score/ScoreQuantum)
		return cmp.Or(cmp.Compare(qa, qb), cmp.Compare(math.Round(elevation(pool[a].d)/tieQuantum), math.Round(elevation(pool[b].d)/tieQuantum)))
	})
	cosSep := math.Cos(finalistSeparationDeg * math.Pi / 180)
	var finalists []dir
	for _, i := range order {
		if len(finalists) == opts.Finalists {
			break
		}
		p := pool[i]
		near := false
		for _, f := range finalists {
			if f.d.Dot(p.d) >= cosSep {
				near = true
				break
			}
		}
		if !near {
			finalists = append(finalists, p)
		}
	}

	step := math.Sqrt(4*math.Pi/float64(max(1, opts.Directions))) * 180 / math.Pi
	for i := range finalists {
		if err := ctx.Err(); err != nil {
			return nil, SearchStats{}, err
		}
		moved := false
		for s := step; s >= refineMinStepDeg; {
			best := finalists[i]
			for _, nb := range neighbors(best.d, s) {
				if !allowed(nb) {
					continue
				}
				if c := est.Estimate(nb); supportable(c) && c.Score < best.cost.Score-ScoreQuantum {
					best = dir{d: nb, source: SourceRefined, cost: c}
				}
			}
			if best.source == SourceRefined && best.d != finalists[i].d {
				finalists[i] = best
				moved = true
				continue
			}
			s /= 2
		}
		if moved {
			stats.Refined++
		}
	}
	slices.SortStableFunc(finalists, func(a, b dir) int {
		return cmp.Compare(math.Round(a.cost.Score/ScoreQuantum), math.Round(b.cost.Score/ScoreQuantum))
	})

	orig := seeds[0]
	orig.cost = est.Estimate(orig.d)
	list := []dir{orig}
	for _, f := range finalists {
		if f.d.Dot(orig.d) >= cosSep {
			continue // the original is already listed
		}
		list = append(list, f)
	}
	out := make([]Candidate, len(list))
	for i, l := range list {
		out[i] = Candidate{
			ID:           i,
			Source:       l.source,
			Down:         l.d,
			Rotation:     faceDown(l.d),
			FaceNormal:   l.face.normal,
			FaceAreaMM2:  l.face.area,
			ElevationDeg: elevation(l.d),
			Allowed:      tiltOK(l.d),
			Estimate:     l.cost,
		}
	}
	return out, stats, nil
}

// fibonacciSphere returns n directions spread evenly over the unit sphere.
func fibonacciSphere(n int) []r3.Vec {
	golden := math.Pi * (3 - math.Sqrt(5))
	out := make([]r3.Vec, n)
	for i := range n {
		z := 1 - 2*(float64(i)+0.5)/float64(n)
		r := math.Sqrt(math.Max(0, 1-z*z))
		phi := golden * float64(i)
		out[i] = r3.NewVec(r*math.Cos(phi), r*math.Sin(phi), z)
	}
	return out
}

// neighbors returns refineNeighbors unit directions at stepDeg degrees from
// d, evenly spaced around it.
func neighbors(d r3.Vec, stepDeg float64) []r3.Vec {
	ref := r3.NewVec(1, 0, 0)
	if math.Abs(d.X) > 0.9 {
		ref = r3.NewVec(0, 1, 0)
	}
	u, _ := d.Cross(ref).Normalize()
	v := d.Cross(u)
	a := stepDeg * math.Pi / 180
	out := make([]r3.Vec, refineNeighbors)
	for k := range refineNeighbors {
		th := 2 * math.Pi * float64(k) / refineNeighbors
		w := u.Scale(math.Cos(th)).Add(v.Scale(math.Sin(th)))
		n, _ := d.Scale(math.Cos(a)).Add(w.Scale(math.Sin(a))).Normalize()
		out[k] = n
	}
	return out
}
