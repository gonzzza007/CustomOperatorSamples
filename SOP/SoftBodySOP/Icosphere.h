// Unit icosphere generator. Level 0 is an icosahedron (12 points, 20 triangles);
// every level splits each triangle into 4, so points grow roughly 4x per level.

#pragma once

#include "Vec3.h"
#include <cstdint>
#include <vector>

namespace sb
{

struct Icosphere
{
	std::vector<Vec3>		points;		// on the unit sphere
	std::vector<int32_t>	triangles;	// 3 indices per triangle, counter-clockwise seen from outside
};

// Returns a cached unit icosphere of the given level (clamped to [0, 7]).
const Icosphere&	getUnitIcosphere(int level);

// Edge length of a unit icosphere at level 0 (icosahedron inscribed in a unit sphere).
constexpr float		IcosahedronEdge = 1.0514622f;

// Picks the smallest level whose edges are no longer than targetEdge for a sphere of the given radius.
int					icosphereLevelFor(float radius, float targetEdge, int minLevel, int maxLevel);

}
