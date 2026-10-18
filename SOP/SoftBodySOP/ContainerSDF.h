// Signed distance field of a closed triangle mesh, sampled on a regular grid.
// Negative inside, positive outside. Used to keep particles inside an arbitrary container.
//
// Construction follows the usual recipe (Bridson / SDFGen): exact point-triangle distances
// in a narrow band around each triangle, fast sweeping to propagate the closest triangle to
// the rest of the grid, and ray parity along X to decide inside/outside.

#pragma once

#include "Vec3.h"
#include <cstdint>
#include <vector>

namespace sb
{

class ContainerSDF
{
public:
	// resolution = number of cells along the longest side of the mesh bounding box.
	// Returns false if the mesh is empty or degenerate.
	bool		build(const std::vector<Vec3>& points, const std::vector<int32_t>& triangles, int resolution);

	void		clear();
	bool		isValid() const { return !myPhi.empty(); }

	// Signed distance at p (trilinear). Outside the grid, the distance to the grid box is added.
	float		distance(const Vec3& p) const;

	// Signed distance and its gradient (points outward, roughly unit length).
	float		distanceAndGradient(const Vec3& p, Vec3& gradient) const;

	float		cellSize() const { return myDx; }
	float		insideVolume() const { return myInsideVolume; }
	Vec3		boundsMin() const { return myOrigin; }
	Vec3		boundsMax() const;

private:
	float		at(int i, int j, int k) const { return myPhi[(static_cast<size_t>(k) * myNy + j) * myNx + i]; }

	std::vector<float>	myPhi;
	Vec3				myOrigin;
	float				myDx = 0.0f;
	float				myInsideVolume = 0.0f;
	int					myNx = 0, myNy = 0, myNz = 0;
};

// Closest point on triangle (a, b, c) to p (Ericson, Real-Time Collision Detection 5.1.5).
Vec3	closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c);

}
