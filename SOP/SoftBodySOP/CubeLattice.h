// Unit cube lattice for solid soft cubes.
//
// The cube [-1, 1]^3 (radius 1, like the unit sphere: half the edge length) is split into resolution^3 cells, (resolution + 1)^3 points. Every cell is
// split into 6 tetrahedra that share its main diagonal (Kuhn / Freudenthal subdivision); with the same
// split in every cell, neighbouring cells agree on their shared face diagonals, so the tets fit together.
// Edges and tets are ordered by colour (constraints of one colour share no point), so a solver can
// process them without waiting on the previous constraint's result.
//
// The surface is given separately for drawing: every point on a side of the cube gets one surface
// vertex per side it lies on (edges 2, corners 3), so the drawn cube can keep hard edges.

#pragma once

#include "Vec3.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace sb
{

struct CubeLattice
{
	int							resolution = 1;		// cells per side
	std::vector<Vec3>			points;				// in [-1, 1]^3
	std::vector<int32_t>		tets;				// 4 point indices per tet, positive volume
	std::vector<std::pair<int32_t, int32_t>>	edges;	// every tet edge once
	std::vector<int32_t>		surfacePoints;		// points on the boundary
	std::vector<int32_t>		surfaceVertexPoint;	// point of each surface vertex
	std::vector<int32_t>		surfaceTriangles;	// 3 surface vertex indices each, counter-clockwise seen from outside
	float						maxSurfaceEdge = 0.0f;	// longest surface edge (a face diagonal)
};

constexpr int		MaxLatticeResolution = 48;

// Returns a cached unit lattice of the given resolution (clamped to [1, MaxLatticeResolution]).
const CubeLattice&	getUnitCubeLattice(int resolution);

inline int			latticePointCount(int resolution) { return (resolution + 1) * (resolution + 1) * (resolution + 1); }

// Smallest resolution whose point count is >= points (and largest one whose count is <= points).
int					latticeResolutionAtLeast(int points);
int					latticeResolutionAtMost(int points);

// Smallest resolution whose cells are no longer than targetEdge for a cube of the given radius (half the edge
// length), clamped to [minResolution, maxResolution].
int					latticeResolutionFor(float radius, float targetEdge, int minResolution, int maxResolution);

}
