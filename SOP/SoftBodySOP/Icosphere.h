// Unit geodesic sphere generator (class I geodesic polyhedron).
// Each face of an icosahedron is split into frequency^2 triangles and projected onto the sphere,
// giving 10 * frequency^2 + 2 points. Any integer frequency works, so the point count can follow
// the blob size closely: 12, 42, 92, 162, 252, 362, 492, 642, 812, 1002, ... points.

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

constexpr int		MaxGeodesicFrequency = 64;

// Returns a cached unit geodesic sphere of the given frequency (clamped to [1, MaxGeodesicFrequency]).
const Icosphere&	getUnitGeodesicSphere(int frequency);

// Number of points of a geodesic sphere of the given frequency.
inline int			geodesicPointCount(int frequency) { return 10 * frequency * frequency + 2; }

// Smallest frequency whose point count is >= points (and largest one whose count is <= points).
int					geodesicFrequencyAtLeast(int points);
int					geodesicFrequencyAtMost(int points);

// Edge length of a unit icosahedron (frequency 1).
constexpr float		IcosahedronEdge = 1.0514622f;

// Picks the smallest frequency whose edges are no longer than targetEdge for a sphere of the given
// radius, clamped to [minFrequency, maxFrequency].
int					geodesicFrequencyFor(float radius, float targetEdge, int minFrequency, int maxFrequency);

}
