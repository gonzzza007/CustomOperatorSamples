#include "ShapeGenerator.h"
#include <array>
#include <vector>
#include <algorithm>
#include "voronoi/voro++.hh"

using namespace TD;
using namespace voro;


// square distance
static float square_distance(const Position &a1, const Position &a2) {
	return (a1.x - a2.x) * (a1.x - a2.x) +
		(a1.y - a2.y) * (a1.y - a2.y) +
		(a1.z - a2.z) * (a1.z - a2.z);
}

static Position scaled(const Position &center, const Position &pos, const Position &spreaded, const float &scale) {
	return Position(
		center.x + (pos.x - center.x) * scale + spreaded.x,
		center.y + (pos.y - center.y) * scale + spreaded.y,
		center.z + (pos.z - center.z) * scale + spreaded.z
	);
}

// Cube descriptors
const std::array<Position, ShapeGenerator::theCubeNumPts8>
ShapeGenerator::theCubePos8 = {
	Position(-1.0f,  1.0f, -1.0f),
	Position(-1.0f,  1.0f,  1.0f),
	Position(-1.0f, -1.0f,  1.0f),
	Position(-1.0f, -1.0f, -1.0f),
	Position(1.0f,  1.0f, -1.0f),
	Position(1.0f,  1.0f,  1.0f),
	Position(1.0f, -1.0f,  1.0f),
	Position(1.0f, -1.0f, -1.0f)
};

// Cube descriptors
const std::array<TD::Position, ShapeGenerator::theCubeNumPts>
ShapeGenerator::theCubePos = {
	// front
	Position(-1.0f, -1.0f, 1.0f),
	Position(-1.0f, 1.0f, 1.0f),
	Position(1.0f, -1.0f, 1.0f),
	Position(1.0f, 1.0f, 1.0f),
	// back
	Position(-1.0f, -1.0f, -1.0f),
	Position(-1.0f, 1.0f, -1.0f),
	Position(1.0f, -1.0f, -1.0f),
	Position(1.0f, 1.0f, -1.0f),
	// top
	Position(-1.0f, 1.0f, -1.0f),
	Position(1.0f, 1.0f, -1.0f),
	Position(-1.0f, 1.0f, 1.0f),
	Position(1.0f, 1.0f, 1.0f),
	// bottom
	Position(-1.0f, -1.0f, -1.0f),
	Position(1.0f, -1.0f, -1.0f),
	Position(-1.0f, -1.0f, 1.0f),
	Position(1.0f, -1.0f, 1.0f),
	// right
	Position(1.0f, -1.0f, -1.0f),
	Position(1.0f, -1.0f, 1.0f),
	Position(1.0f, 1.0f, -1.0f),
	Position(1.0f, 1.0f, 1.0f),
	// left
	Position(-1.0f, -1.0f, -1.0f),
	Position(-1.0f, -1.0f, 1.0f),
	Position(-1.0f, 1.0f, -1.0f),
	Position(-1.0f, 1.0f, 1.0f)
};

const std::array<Vector, ShapeGenerator::theCubeNumPts>			
ShapeGenerator::theCubeNormals = {
	// front
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f),
	// back
	Vector(0.0f, 0.0f, -1.0f),
	Vector(0.0f, 0.0f, -1.0f),
	Vector(0.0f, 0.0f, -1.0f),
	Vector(0.0f, 0.0f, -1.0f),
	// top
	Vector(0.0f, 1.0f, 0.0f),
	Vector(0.0f, 1.0f, 0.0f),
	Vector(0.0f, 1.0f, 0.0f),
	Vector(0.0f, 1.0f, 0.0f),
	// bottom
	Vector(0.0f, -1.0f, 0.0f),
	Vector(0.0f, -1.0f, 0.0f),
	Vector(0.0f, -1.0f, 0.0f),
	Vector(0.0f, -1.0f, 0.0f),
	// right
	Vector(1.0f, 0.0f, 0.0f),
	Vector(1.0f, 0.0f, 0.0f),
	Vector(1.0f, 0.0f, 0.0f),
	Vector(1.0f, 0.0f, 0.0f),
	// left
	Vector(-1.0f, 0.0f, 0.0f),
	Vector(-1.0f, 0.0f, 0.0f),
	Vector(-1.0f, 0.0f, 0.0f),
	Vector(-1.0f, 0.0f, 0.0f)
};

// this cube has total 8 vertexes, so 3 faces use same vertex
const std::array<int32_t, ShapeGenerator::theCubeNumPrim * 3>
ShapeGenerator::theCubeVertices8 = {
	// left
	0, 1, 2,
	0, 3, 2,
	// right
	4, 5, 6,
	4, 7, 6,
	// top
	0, 1, 5,
	0, 4, 5,
	// bottom
	3, 2, 6,
	3, 7, 6,
	// front
	1, 2, 6,
	1, 5, 6,
	// back
	0, 3, 7,
	0, 4, 7
};

// this cube has 4 vertex per side and total 24 vertices
const std::array<int32_t, ShapeGenerator::theCubeNumPrim * 3>	
ShapeGenerator::theCubeVertices = {
	// front
	0, 1, 2,
	3, 2, 1,
	// back
	6, 5, 4,
	5, 6, 7,
	// top
	8, 9, 10,
	11, 10, 9,
	// bottom
	14, 13, 12,
	13, 14, 15,
	// right
	16, 17, 18,
	19, 18, 17,
	// left
	22, 21, 20,
	21, 22, 23
};

const std::array<TexCoord, ShapeGenerator::theCubeNumPts>
ShapeGenerator::theCubeTexture = {
	// front
	TexCoord(2 / 3.0f, 0.0f, 0.0f),
	TexCoord(2 / 3.0f, 0.5f, 0.0f),
	TexCoord(3 / 3.0f, 0.0f, 0.0f),
	TexCoord(3 / 3.0f, 0.5f, 0.0f),
	// back
	TexCoord(0 / 3.0f, 0.5f, 0.0f),
	TexCoord(0 / 3.0f, 0.0f, 0.0f),
	TexCoord(1 / 3.0f, 0.5f, 0.0f),
	TexCoord(1 / 3.0f, 0.0f, 0.0f),
	// top
	TexCoord(2 / 3.0f, 1.0f, 0.0f),
	TexCoord(3 / 3.0f, 1.0f, 0.0f),
	TexCoord(2 / 3.0f, 0.5f, 0.0f),
	TexCoord(3 / 3.0f, 0.5f, 0.0f),
	// bottom
	TexCoord(1 / 3.0f, 0.5f, 0.0f),
	TexCoord(2 / 3.0f, 0.5f, 0.0f),
	TexCoord(1 / 3.0f, 1.0f, 0.0f),
	TexCoord(2 / 3.0f, 1.0f, 0.0f),
	// right
	TexCoord(2 / 3.0f, 0.0f, 0.0f),
	TexCoord(1 / 3.0f, 0.0f, 0.0f),
	TexCoord(2 / 3.0f, 0.5f, 0.0f),
	TexCoord(1 / 3.0f, 0.5f, 0.0f),
	// left
	TexCoord(1 / 3.0f, 1.0f, 0.0f),
	TexCoord(0 / 3.0f, 1.0f, 0.0f),
	TexCoord(1 / 3.0f, 0.5f, 0.0f),
	TexCoord(0 / 3.0f, 0.5f, 0.0f),
};

// Square descriptors					
const std::array<Position, ShapeGenerator::theSquareNumPts>		
ShapeGenerator::theSquarePos = {
	Position(-1.0f, -1.0f, 0.0f),
	Position(-1.0f, 1.0f, 0.0f),
	Position(1.0f, -1.0f, 0.0f),
	Position(1.0f, 1.0f, 0.0f)
};

const std::array<Vector, ShapeGenerator::theSquareNumPts>		
ShapeGenerator::theSquareNormals = {
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f),
	Vector(0.0f, 0.0f, 1.0f)
};

const std::array<int32_t, ShapeGenerator::theSquareNumPrim * 3>	
ShapeGenerator::theSquareVertices = {
	0, 1, 2,
	3, 2, 1
};

const std::array<TexCoord, ShapeGenerator::theSquareNumPts>
ShapeGenerator::theSquareTexture = {
	TexCoord(0.0f, 0.0f, 0.0f),
	TexCoord(0.0f, 1.0f, 0.0f),
	TexCoord(1.0f, 0.0f, 0.0f),
	TexCoord(1.0f, 1.0f, 0.0f)
};

// Line descriptors						
const std::array<Position, ShapeGenerator::theLineNumPts>		
ShapeGenerator::theLinePos = {
	Position(-1.0f, -1.0f, -1.0f),
	Position(1.0f, 1.0f, 1.0f)
};

const std::array<Vector, ShapeGenerator::theLineNumPts>			
ShapeGenerator::theLineNormals = {
	Vector(-1.0f, 0.0f, 1.0f),
	Vector(-1.0f, 0.0f, 1.0f)
};

const std::array<int32_t, ShapeGenerator::theLineNumPts>			
ShapeGenerator::theLineVertices = {
	0, 1
};

const std::array<TexCoord, ShapeGenerator::theLineNumPts>
ShapeGenerator::theLineTexture = {
	TexCoord(0.0f, 0.0f, 0.0f),
	TexCoord(1.0f, 1.0f, 0.0f)
};

// Point descriptors
const Position													
ShapeGenerator::thePointPos = Position();

const Vector
ShapeGenerator::thePointNormal = Vector(0.0f, 0.0f, 1.0f);

const TexCoord
ShapeGenerator::thePointTexture = TexCoord();

void 
ShapeGenerator::outputDot(SOP_Output* output) const
{
	output->addPoint(thePointPos);
	output->setNormal(thePointNormal, 0);
	output->addParticleSystem(1, 0);
	output->setTexCoord(&thePointTexture, 1, 0);
}

void 
ShapeGenerator::outputLine(SOP_Output* output) const
{
	output->addPoints(theLinePos.data(), theLineNumPts);
	output->setNormals(theLineNormals.data(), theLineNumPts, 0);
	output->addLine(theLineVertices.data(), theLineNumPts);
	setPointTexCoords(output, theLineTexture.data(), theLineNumPts);
}

void 
ShapeGenerator::outputSquare(SOP_Output* output) const
{
	output->addPoints(theSquarePos.data(), theSquareNumPts);
	output->setNormals(theSquareNormals.data(), theSquareNumPts, 0);
	output->addTriangles(theSquareVertices.data(), theSquareNumPrim);
	setPointTexCoords(output, theSquareTexture.data(), theSquareNumPts);
}

void 
ShapeGenerator::outputCube(SOP_Output* output) const
{
	output->addPoints(theCubePos.data(), theCubeNumPts);
	output->setNormals(theCubeNormals.data(), theCubeNumPts, 0);
	output->addTriangles(theCubeVertices.data(), theCubeNumPrim);
	setPointTexCoords(output, theCubeTexture.data(), theCubeNumPts);
}

void
ShapeGenerator::outputDivider(const	OP_CHOPInput* input,
															float scale,
															float spread,
															SOP_Output* output) const
{
	// read points positions to per-axis vectors
	std::vector<float> xPos(input->channelData[0], input->channelData[0] + input->numSamples);
	std::vector<float> yPos(input->channelData[1], input->channelData[1] + input->numSamples);
	std::vector<float> zPos(input->channelData[2], input->channelData[2] + input->numSamples);

	std::sort(xPos.begin(), xPos.end());
	std::sort(yPos.begin(), yPos.end());
	std::sort(zPos.begin(), zPos.end());
	
	// add final point (1, 1, 1)
	xPos.push_back(1.0);
	yPos.push_back(1.0);
	zPos.push_back(1.0);

	int doneVertices(0);

	float minYPos(-1.0);
	float minZPos(-1.0);
	
	float prevXPos(-1.0);
	float prevYPos(-1.0);
	float prevZPos(-1.0);

	for (const float &xPosIt : xPos) {		
		if (prevXPos == xPosIt) continue;
		
		for (const float &yPosIt : yPos) {
			if (prevYPos == yPosIt) continue;
			
			for (const float &zPosIt : zPos) {	
				if (prevZPos == zPosIt) continue;

				Position center = {
					(prevXPos + xPosIt) / 2,
					(prevYPos + yPosIt) / 2,
					(prevZPos + zPosIt) / 2
				};

				Position spreaded = { 0, 0, 0 };
				if (spread > 0) 
					spreaded = center * spread;

				output->addPoint(scaled(center, { prevXPos,   yPosIt, prevZPos }, spreaded, scale));
				output->addPoint(scaled(center, { prevXPos,   yPosIt,   zPosIt }, spreaded, scale));
				output->addPoint(scaled(center, { prevXPos, prevYPos,   zPosIt }, spreaded, scale));
				output->addPoint(scaled(center, { prevXPos, prevYPos, prevZPos }, spreaded, scale));
				output->addPoint(scaled(center, {   xPosIt,   yPosIt, prevZPos }, spreaded, scale));
				output->addPoint(scaled(center, {   xPosIt,   yPosIt,   zPosIt }, spreaded, scale));
				output->addPoint(scaled(center, {   xPosIt, prevYPos,   zPosIt }, spreaded, scale));
				output->addPoint(scaled(center, {   xPosIt, prevYPos, prevZPos }, spreaded, scale));

				for (size_t i = 0; i < theCubeVertices8.size(); i++) {
					if (i % 3 == 0) {
						output->addTriangle(
							theCubeVertices8[i] + doneVertices,
							theCubeVertices8[i + 1] + doneVertices,
							theCubeVertices8[i + 2] + doneVertices
						);
					}
				}

				// shift next indices by 8
				doneVertices += theCubeNumPts8;

				prevZPos = zPosIt;
			}
			prevYPos = yPosIt; // previous Y value
			prevZPos = minZPos; // next Z row
		}
		prevXPos = xPosIt; // previous X value
		prevYPos = minYPos; // next Y row
	}
}


void	ShapeGenerator::outputKDTree(const	OP_CHOPInput* input,
																	 float scale,
																	 float spread,
																	 SOP_Output* output) const
{
	// 0 - divide by x
	// 1 - divide by y
	// 2 - divide by z
	int dimension = 0;

	theCube initial = { 
		dimension,
		Position(0.f, 0.f, 0.f),
		theCubePos8
	};

	std::vector<theCube> theCubesFinal;
	std::vector<theCube> theCubes;
	
	// the size might differ, but better to allocate
	theCubesFinal.reserve(input->numSamples);
	theCubes.reserve(input->numSamples);
	
	theCubes.push_back(initial);
	
	// read points positions from input chop
	std::vector<Position> samples;
	samples.reserve(input->numSamples);
	for (int i = 0; i < input->numSamples; i++) {
		samples.emplace_back(
			input->getChannelData(0)[i],
			input->getChannelData(1)[i],
			input->getChannelData(2)[i]
		);
	}


	while (samples.size()) {
		int size = samples.size();
		
		// we will save processed cubes here for the next cycle...
		std::vector<theCube> theCubesNext;

		for (auto cubesIt = theCubes.begin(); cubesIt != theCubes.end(); cubesIt++) {
			
			// ************************************* //
			// find the point closest to cube center //
			// ************************************* //
			bool found(false);
			float minDistance(5.0f);
			std::vector<Position>::iterator middleSamplesIt = samples.begin();
			for (auto samplesIt = samples.begin(); samplesIt != samples.end(); samplesIt++)
			{
				if ( // this sample is inside our box
					(cubesIt->coords[3].x <= samplesIt->x) &&
					(cubesIt->coords[3].y <= samplesIt->y) &&
					(cubesIt->coords[3].z <= samplesIt->z) &&
					(cubesIt->coords[5].x >= samplesIt->x) &&
					(cubesIt->coords[5].y >= samplesIt->y) &&
					(cubesIt->coords[5].z >= samplesIt->z)) {

					found = true;

					// is our sample closer to center?
					float currentDistance = square_distance(*samplesIt, cubesIt->center);
					if (currentDistance < minDistance) {
						minDistance = currentDistance;
						middleSamplesIt = samplesIt;
					}
				} // if inside cube
			} // for samples
			
			// there are no samples/points in this cube - push this cube to the FINALcubes
			if (!found) {
				theCubesFinal.push_back(*cubesIt);
			
			// divide this cube, and create two cubes for the next cycle
			} else {
				// save the sample and erase it from the input array
				Position middleSample = *middleSamplesIt;
				samples.erase(middleSamplesIt);
				// the cube will be divided to two by sample point
				theCube left, right;

				switch (dimension) {
				// divide by X coordinate
				case 0: 
					left = {
						dimension,
						Position {
							(middleSample.x       + cubesIt->coords[3].x) / 2,
							(cubesIt->coords[5].y + cubesIt->coords[3].y) / 2,
							(cubesIt->coords[5].z + cubesIt->coords[3].z) / 2
						},
						{ cubesIt->coords[0],
							cubesIt->coords[1],
							cubesIt->coords[2],
							cubesIt->coords[3],
							Position(middleSample.x, cubesIt->coords[4].y, cubesIt->coords[4].z),
							Position(middleSample.x, cubesIt->coords[5].y, cubesIt->coords[5].z),
							Position(middleSample.x, cubesIt->coords[6].y, cubesIt->coords[6].z),
							Position(middleSample.x, cubesIt->coords[7].y, cubesIt->coords[7].z)
						}
					};
					right = {
						dimension,
						Position {
							(cubesIt->coords[5].x +       middleSample.x) / 2,
							(cubesIt->coords[5].y + cubesIt->coords[3].y) / 2,
							(cubesIt->coords[5].z + cubesIt->coords[3].z) / 2
						},
						{ Position(middleSample.x, cubesIt->coords[0].y, cubesIt->coords[0].z),
							Position(middleSample.x, cubesIt->coords[1].y, cubesIt->coords[1].z),
							Position(middleSample.x, cubesIt->coords[2].y, cubesIt->coords[2].z),
							Position(middleSample.x, cubesIt->coords[3].y, cubesIt->coords[3].z),
							cubesIt->coords[4],
							cubesIt->coords[5],
							cubesIt->coords[6],
							cubesIt->coords[7]
						}
					};
					break;
				// divide by Y coordinate
				case 1:
					left = { // top half
						dimension,
						Position {
							(cubesIt->coords[5].x + cubesIt->coords[3].x) / 2,
							(cubesIt->coords[5].y + middleSample.y) / 2,
							(cubesIt->coords[5].z + cubesIt->coords[3].z) / 2
						},
						{ cubesIt->coords[0],
							cubesIt->coords[1],
							Position(cubesIt->coords[2].x, middleSample.y, cubesIt->coords[2].z),
							Position(cubesIt->coords[3].x, middleSample.y, cubesIt->coords[3].z),
							cubesIt->coords[4],
							cubesIt->coords[5],
							Position(cubesIt->coords[6].x, middleSample.y, cubesIt->coords[6].z),
							Position(cubesIt->coords[7].x, middleSample.y, cubesIt->coords[7].z)
						}
					};
					right = { // bottom half
						dimension,
						Position {
							(cubesIt->coords[5].x + cubesIt->coords[3].x) / 2,
							(middleSample.y       + cubesIt->coords[3].y) / 2,
							(cubesIt->coords[5].z + cubesIt->coords[3].z) / 2
						},
						{ 
							Position(cubesIt->coords[0].x, middleSample.y, cubesIt->coords[0].z),
							Position(cubesIt->coords[1].x, middleSample.y, cubesIt->coords[1].z),
							cubesIt->coords[2],
							cubesIt->coords[3],
							Position(cubesIt->coords[4].x, middleSample.y, cubesIt->coords[4].z),
							Position(cubesIt->coords[5].x, middleSample.y, cubesIt->coords[5].z),
							cubesIt->coords[6],
							cubesIt->coords[7]
						}
					};
					break;
				// divide by Z coordinate
				case 2:
				default:
					left = { // back half
						dimension,
						Position {
							(cubesIt->coords[5].x + cubesIt->coords[3].x) / 2,
							(cubesIt->coords[5].y + cubesIt->coords[3].y) / 2,
							(middleSample.z       + cubesIt->coords[3].z) / 2
						},
						{ cubesIt->coords[0],
							Position(cubesIt->coords[1].x, cubesIt->coords[1].y, middleSample.z),
							Position(cubesIt->coords[2].x, cubesIt->coords[2].y, middleSample.z),
							cubesIt->coords[3],
							cubesIt->coords[4],
							Position(cubesIt->coords[5].x, cubesIt->coords[5].y, middleSample.z),
							Position(cubesIt->coords[6].x, cubesIt->coords[6].y, middleSample.z),
							cubesIt->coords[7]
						}
					};
					right = { // front half
						dimension,
						Position {
							(cubesIt->coords[5].x + cubesIt->coords[3].x) / 2,
							(cubesIt->coords[5].y + cubesIt->coords[3].y) / 2,
							(cubesIt->coords[5].z +       middleSample.z) / 2
						},
						{	Position(cubesIt->coords[0].x, cubesIt->coords[0].y, middleSample.z),
							cubesIt->coords[1],
							cubesIt->coords[2],
							Position(cubesIt->coords[3].x, cubesIt->coords[3].y, middleSample.z),
							Position(cubesIt->coords[4].x, cubesIt->coords[4].y, middleSample.z),
							cubesIt->coords[5],
							cubesIt->coords[6],
							Position(cubesIt->coords[7].x, cubesIt->coords[7].y, middleSample.z)
						}
					};
					break;
				}

				// move the cubes to final if no samples left
				if (samples.size()) {
					theCubesNext.push_back(left);
					theCubesNext.push_back(right);
				} else {
					theCubesFinal.push_back(left);
					theCubesFinal.push_back(right);
				}
			}
		} // for each cubes

		theCubes = std::move(theCubesNext);

		dimension++;
		if (dimension > 2)
			dimension = 0;

		// we did not process any samples - this should not
		// happen, maybe the samples are out of bounds?
		// exit to avoid infinite loop.
		if (size == samples.size()) break;
	} // while samples > 0

	// we have some cubes left, but no samples - copy these to final
	if (theCubes.size()) {
		theCubesFinal.insert(theCubesFinal.end(), theCubes.begin(), theCubes.end());
	}

	// OUTPUT
	int doneVertices(0);
	for (auto &theCube : theCubesFinal) {
		Position spreaded = {0, 0, 0};
		if (spread > 0)
			spreaded = theCube.center * spread;
	
		output->addPoint(scaled(theCube.center, theCube.coords[0], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[1], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[2], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[3], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[4], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[5], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[6], spreaded, scale));
		output->addPoint(scaled(theCube.center, theCube.coords[7], spreaded, scale));

		for (size_t i = 0; i < theCubeVertices8.size(); i++) {
			if (i % 3 == 0) {
				output->addTriangle(
					theCubeVertices8[i] + doneVertices,
					theCubeVertices8[i + 1] + doneVertices,
					theCubeVertices8[i + 2] + doneVertices
				);
			}
		}

		// shift next indices by 8
		doneVertices += theCubeNumPts8;


		/*
		// convert KDTree cube's 8 points to Cube mesh 24 points
		// front
		output->addPoint(scaled(theCube.center, theCube.coords[2], spreaded, scale)); // 0
		output->addPoint(scaled(theCube.center, theCube.coords[1], spreaded, scale)); // 1
		output->addPoint(scaled(theCube.center, theCube.coords[6], spreaded, scale)); // 2
		output->addPoint(scaled(theCube.center, theCube.coords[5], spreaded, scale)); // 3
		// back
		output->addPoint(scaled(theCube.center, theCube.coords[3], spreaded, scale)); // 4
		output->addPoint(scaled(theCube.center, theCube.coords[0], spreaded, scale)); // 5
		output->addPoint(scaled(theCube.center, theCube.coords[7], spreaded, scale)); // 6
		output->addPoint(scaled(theCube.center, theCube.coords[4], spreaded, scale)); // 7
		// top
		output->addPoint(scaled(theCube.center, theCube.coords[0], spreaded, scale)); // 8
		output->addPoint(scaled(theCube.center, theCube.coords[4], spreaded, scale)); // 9
		output->addPoint(scaled(theCube.center, theCube.coords[1], spreaded, scale)); // 10
		output->addPoint(scaled(theCube.center, theCube.coords[5], spreaded, scale)); // 11
		// bottom
		output->addPoint(scaled(theCube.center, theCube.coords[3], spreaded, scale)); // 12
		output->addPoint(scaled(theCube.center, theCube.coords[7], spreaded, scale)); // 13
		output->addPoint(scaled(theCube.center, theCube.coords[2], spreaded, scale)); // 14
		output->addPoint(scaled(theCube.center, theCube.coords[6], spreaded, scale)); // 15
		// right
		output->addPoint(scaled(theCube.center, theCube.coords[7], spreaded, scale)); // 16
		output->addPoint(scaled(theCube.center, theCube.coords[6], spreaded, scale)); // 17
		output->addPoint(scaled(theCube.center, theCube.coords[4], spreaded, scale)); // 18
		output->addPoint(scaled(theCube.center, theCube.coords[5], spreaded, scale)); // 19
		// left
		output->addPoint(scaled(theCube.center, theCube.coords[3], spreaded, scale)); // 20
		output->addPoint(scaled(theCube.center, theCube.coords[2], spreaded, scale)); // 21
		output->addPoint(scaled(theCube.center, theCube.coords[0], spreaded, scale)); // 22
		output->addPoint(scaled(theCube.center, theCube.coords[1], spreaded, scale)); // 23
		*/

	} // for each final cube
}

void
ShapeGenerator::outputVoronoi(const OP_CHOPInput* input, 
															float scale,
															float spread,
															SOP_Output* output) const
{
	// Set up constants for the container geometry
	const double x_min = -1, x_max = 1;
	const double y_min = -1, y_max = 1;
	const double z_min = -1, z_max = 1;
	// const double cvol = (x_max - x_min) * (y_max - y_min) * (x_max - x_min);
	
	// optimal container division is 5 cells per block
	// get fast ceiling of numSamples/5
	const int division = input->numSamples / 5 + (input->numSamples % 5 != 0);

	// remember how much vertices we already processed;
	// we are combining all cells to 1 shape, so we need
	// to shift the indices and normals number
	int doneVertices(0);
	int doneIndices(0);
	int currentIndices(0);

	// create the Voronoi container
	container con(
		x_min, x_max, 
		y_min, y_max, 
		z_min, z_max, 
		division, division, division,
		false, false, false, 
		8
	);

	// read voronoi points positions from input chop
	for (int i = 0; i < input->numSamples; i++) {
		con.put(i,
			input->getChannelData(0)[i],
			input->getChannelData(1)[i],
			input->getChannelData(2)[i]
		);
	}
	
	voronoicell c; 
	c_loop_all cl(con);
	if (cl.start()) {
		do {
			if (con.compute_cell(c, cl)) {
				
				// get the position of the cell central point
				double x, y, z;
				cl.pos(x, y, z);
				Position center = { (float)x, (float)y, (float)z };

				Position spreaded = { 0, 0, 0 };
				if (spread > 0)
					spreaded = center * spread;
		
				// -------------------------------------------------------------------------
				// output vertices
				// NB! Voro++ shares vertexes between edges. So we currently can not add
				// normals, because the only way to do it now is to have separate vertexes
				// for each edge and add normal for each of these vertexes.
				// -------------------------------------------------------------------------
				double* ptsp = c.pts;
				for (int i = 0; i < c.p; i++, ptsp += 3) {
					Position pos = { 
						(float)(x + *ptsp * 0.5),
						(float)(y + ptsp[1] * 0.5), 
						(float)(z + ptsp[2] * 0.5) 
					};
					output->addPoint(scaled(center, pos, spreaded, scale));
				}

				// -------------------------------------------------------------------------
				// output indices
				// -------------------------------------------------------------------------
				// c.pts : the position vectors x_0, x_1, ..., x_{ p - 1 } of the polyhedron vertices.
				// c.nu  : the number of other vertices to which each is connected.
				// c.ed  : table of edges and relations. For the i-th vertex, ed[i] has 2n_i + 1 elements:
				//		A. The first n_i elements are the edges e(j, i), where e(j, i) is the j-th neighbor of
				//       vertex i. The edges are ordered according to a right - hand rule with respect to an
				//       outward - pointing normal. 
				//    B. The next n_i elements are the relations l(j, i) which satisfy
				//       the property e(l(j, i), e(j, i)) = i. 
				//    C. The final element of the ed[i] list is a back pointer used in memory allocation.
				// -------------------------------------------------------------------------
				// for vertices
				for (int i = 1; i < c.p; i++) 
					// for each of vertice connections
					for (int j = 0; j < c.nu[i]; j++) {
						int k, l, m, n;
						// current (A) edge vertex
						k = c.ed[i][j];
						// if connection not processed
						if (k >= 0) {
							// invert it (which means it's scanned)
							c.ed[i][j] = -1 - k;
							// get index of next connection (B)
							l = c.cycle_up(c.ed[i][c.nu[i] + j], k);
							// get next connected vertice and mark as processed 
							m = c.ed[k][l]; c.ed[k][l] = -1 - m;
							// repeat untill we get back to first vertex
							while (m != i) {
								n = c.cycle_up(c.ed[k][c.nu[k] + l], m);

								output->addTriangle(
									i + doneVertices,
									k + doneVertices,
									m + doneVertices
								);
								k = m; l = n;
								m = c.ed[k][l]; c.ed[k][l] = -1 - m;
								currentIndices++;
							}
						}
					}

			
				// -------------------------------------------------------------------------
				// output vertice normals
				// -------------------------------------------------------------------------
				/* std::vector<int> faces;
				c.face_orders(faces);
				get normals for each face
				std::vector<double> norm;
				c.normals(norm);
				
				for (int i = 0; i < c.p; i++) {
					Vector theVector(
						norm[3 * i],
						norm[3 * i + 1],
						norm[3 * i + 2]
					);
					output->setNormal(theVector, i + doneVertices);
				}*/
				
				// remember how much vertices we already processed
				// we are combining all cells to 1 shape, so we need
				// to shift the indices and normal numbers
				doneVertices += c.p;
				doneIndices += currentIndices;
				currentIndices = 0;

			} // if compute_cell
		} while (cl.inc());
	} // if cl.start()...
}

void 
ShapeGenerator::outputDotVBO(SOP_VBOOutput* output)
{
	output->allocVBO(1, 1, VBOBufferMode::Static);
	output->getPos()[0] = thePointPos;
	output->getNormals()[0] = thePointNormal;
	output->getTexCoords()[0] = thePointTexture;
	output->addParticleSystem(1)[0] = 0;
}

void 
ShapeGenerator::outputLineVBO(SOP_VBOOutput* output)
{
	output->allocVBO(theLineNumPts, theLineNumPts, VBOBufferMode::Static);
	myLastVBOAllocVertices = theLineNumPts;
	memcpy(output->getPos(), theLinePos.data(), theLineNumPts * sizeof(Position));
	memcpy(output->getNormals(), theLineNormals.data(), theLineNumPts * sizeof(Vector));
	memcpy(output->getTexCoords(), theLineTexture.data(), theLineNumPts * sizeof(TexCoord));
	memcpy(output->addLines(theLineNumPts), theLineVertices.data(), theLineNumPts * sizeof(int32_t));
}

void 
ShapeGenerator::outputSquareVBO(SOP_VBOOutput* output)
{
	output->allocVBO(theSquareNumPts, theSquareNumPrim * 3, VBOBufferMode::Static);
	myLastVBOAllocVertices = theSquareNumPts;
	memcpy(output->getPos(), theSquarePos.data(), theSquareNumPts * sizeof(Position));
	// Cannote memcpy normals since GPU is in the other direction
	Vector* outN = output->getNormals();
	for (int i = 0; i < theSquareNormals.size(); i += 1)
	{
		outN[i] = Vector(theSquareNormals.at(i)) * -1.0f;
	}
	memcpy(output->getTexCoords(), theSquareTexture.data(), theSquareNumPts * sizeof(TexCoord));
	memcpy(output->addTriangles(theSquareNumPrim), theSquareVertices.data(), theSquareNumPts * 3 * sizeof(int32_t));
}

void 
ShapeGenerator::outputCubeVBO(SOP_VBOOutput* output)
{
	output->allocVBO(theCubeNumPts, theCubeNumPrim * 3, VBOBufferMode::Static);
	myLastVBOAllocVertices = theCubeNumPts;
	memcpy(output->getPos(), theCubePos.data(), theCubeNumPts * sizeof(Position));
	// Cannote memcpy normals since GPU is in the other direction
	Vector* outN = output->getNormals();
	for (int i = 0; i < theCubeNormals.size(); i += 1)
	{
		outN[i] = Vector(theCubeNormals.at(i)) * -1.0f;
	}
	memcpy(output->getTexCoords(), theCubeTexture.data(), theCubeNumPts * sizeof(TexCoord));
	memcpy(output->addTriangles(theCubeNumPrim), theCubeVertices.data(), theCubeNumPrim * 3 * sizeof(int32_t));
}

int 
ShapeGenerator::getLastVBONumVertices() const
{
	return myLastVBOAllocVertices;
}

void 
ShapeGenerator::setPointTexCoords(SOP_Output* output, const TexCoord* t, int32_t numPts) const
{
	for (int i = 0; i < numPts; ++i)
	{
		output->setTexCoord(t + i, 1, i);
	}
}
