#pragma once

#include "CPlusPlus_Common.h"
#include "Parameters.h"

#include <string>
#include <vector>

// OpenCascade part of the SOP, kept free of the TouchDesigner node API so it
// can also be run outside of TouchDesigner

struct OcctSettings
{
	ModifierMenuItems	modifier;
	double				radius;				// fillet radius or chamfer distance, 0 = no modifier
	double				edgeAngle;			// radians
	double				deflection;
	double				angularDeflection;	// radians
	NormalsMenuItems	normals;
};

struct OcctResult
{
	std::vector<TD::Position>	points;
	std::vector<TD::Vector>		normals;
	std::vector<int32_t>		indices;	// counter-clockwise triangles
	std::string					warning;
	std::string					error;
};

// polygons: for each polygon its vertex count followed by its point indices,
// a negative count marks a primitive to skip
void	occtProcess(const TD::Position* points, int32_t numPoints, const std::vector<int32_t>& polygons,
					const OcctSettings& settings, OcctResult& result);
