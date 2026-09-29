#include "ShapeGenerator.h"
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include "voronoi/voro++.hh"

using namespace TD;

namespace
{
	using Vec3 = std::array<float, 3>;

	// Axis-aligned box, used for Divider and KDTree cells
	struct Box
	{
		Vec3 lo;
		Vec3 hi;
	};

	// The [-1, 1] box all generated cells fill; samples outside it are ignored
	const Box theUnitBox = { { -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f } };

	// Box corner order used by emitBox():
	// 0 (lo.x, hi.y, lo.z)  1 (lo.x, hi.y, hi.z)  2 (lo.x, lo.y, hi.z)  3 (lo.x, lo.y, lo.z)
	// 4 (hi.x, hi.y, lo.z)  5 (hi.x, hi.y, hi.z)  6 (hi.x, lo.y, hi.z)  7 (hi.x, lo.y, lo.z)
	constexpr int32_t theBoxNumPts = 8;
	constexpr int32_t theBoxNumPrim = 12;

	// Wound the same way as ShapeGenerator::theCubeVertices
	constexpr std::array<int32_t, theBoxNumPrim * 3> theBoxVertices = {
		0, 1, 2,  0, 2, 3,	// left
		4, 6, 5,  4, 7, 6,	// right
		0, 5, 1,  0, 4, 5,	// top
		3, 2, 6,  3, 6, 7,	// bottom
		1, 6, 2,  1, 5, 6,	// front
		0, 3, 7,  0, 7, 4	// back
	};

	float
	squareDistance(const Vec3& a, const Vec3& b)
	{
		const float dx = a[0] - b[0];
		const float dy = a[1] - b[1];
		const float dz = a[2] - b[2];
		return dx * dx + dy * dy + dz * dz;
	}

	// True if p lies in the [-1, 1] box; NaN fails every comparison, so it is rejected
	bool
	insideUnitBox(const Vec3& p)
	{
		return p[0] >= -1.0f && p[0] <= 1.0f &&
			p[1] >= -1.0f && p[1] <= 1.0f &&
			p[2] >= -1.0f && p[2] <= 1.0f;
	}

	// Scale pos around center, then push it away from the origin by offset
	Position
	scaled(const Position& center, const Position& pos, const Position& offset, float scale)
	{
		return Position(
			center.x + (pos.x - center.x) * scale + offset.x,
			center.y + (pos.y - center.y) * scale + offset.y,
			center.z + (pos.z - center.z) * scale + offset.z
		);
	}

	// Collects points and triangles and hands them to SOP_Output in bulk, so large
	// outputs don't pay one virtual call per point and triangle. Triangle indices
	// are absolute output point indices; call flushIfFull() only between shapes.
	class BatchedMesh
	{
	public:
		explicit BatchedMesh(SOP_Output* output) :
			myOutput(output),
			myPointBase(output->getNumPoints())
		{
			myPoints.reserve(theFlushPoints);
			myTriangles.reserve(theFlushPoints * 3);
		}

		int32_t
		nextPointIndex() const
		{
			return myPointBase + static_cast<int32_t>(myPoints.size());
		}

		void
		addPoint(const Position& p)
		{
			myPoints.push_back(p);
		}

		void
		addTriangle(int32_t a, int32_t b, int32_t c)
		{
			myTriangles.push_back(a);
			myTriangles.push_back(b);
			myTriangles.push_back(c);
		}

		void
		flushIfFull()
		{
			if (myPoints.size() >= theFlushPoints)
				flush();
		}

		void
		flush()
		{
			if (!myPoints.empty())
				myOutput->addPoints(myPoints.data(), static_cast<int32_t>(myPoints.size()));
			if (!myTriangles.empty())
				myOutput->addTriangles(myTriangles.data(), static_cast<int32_t>(myTriangles.size() / 3));
			myPointBase += static_cast<int32_t>(myPoints.size());
			myPoints.clear();
			myTriangles.clear();
		}

	private:
		static constexpr size_t theFlushPoints = 8192;

		SOP_Output*				myOutput;
		int32_t					myPointBase;
		std::vector<Position>	myPoints;
		std::vector<int32_t>	myTriangles;
	};

	// Adds one box as 8 points and 12 triangles, scaled around its center and
	// moved away from the origin by spread
	void
	emitBox(const Box& box, float scale, float spread, BatchedMesh& mesh)
	{
		const Vec3& lo = box.lo;
		const Vec3& hi = box.hi;
		Position center(
			(lo[0] + hi[0]) * 0.5f,
			(lo[1] + hi[1]) * 0.5f,
			(lo[2] + hi[2]) * 0.5f
		);
		const Position offset = spread > 0.0f ? center * spread : Position();

		const std::array<Position, theBoxNumPts> corners = {
			Position(lo[0], hi[1], lo[2]),
			Position(lo[0], hi[1], hi[2]),
			Position(lo[0], lo[1], hi[2]),
			Position(lo[0], lo[1], lo[2]),
			Position(hi[0], hi[1], lo[2]),
			Position(hi[0], hi[1], hi[2]),
			Position(hi[0], lo[1], hi[2]),
			Position(hi[0], lo[1], lo[2])
		};

		const int32_t base = mesh.nextPointIndex();
		for (const Position& corner : corners)
			mesh.addPoint(scaled(center, corner, offset, scale));
		for (size_t i = 0; i < theBoxVertices.size(); i += 3)
			mesh.addTriangle(base + theBoxVertices[i], base + theBoxVertices[i + 1], base + theBoxVertices[i + 2]);

		mesh.flushIfFull();
	}
}

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
	// Each channel divides only its own axis: channel 0 values are x cut planes,
	// channel 1 values y cuts, channel 2 values z cuts. Cuts outside the [-1, 1]
	// box are ignored so the outer boundary never moves; the negated test also
	// rejects NaN.
	// To keep the cell count comparable with KDTree (N + 1) and Voronoi (N), cuts
	// are taken sample by sample, x, y, z in turn, and one is only accepted while
	// the grid stays within numSamples cells.
	const auto inside = [](float v) { return v > -1.0f && v < 1.0f; };
	const int64_t maxCells = std::max<int64_t>(input->numSamples, 1);
	std::array<std::vector<float>, 3> cuts;
	for (int32_t i = 0; i < input->numSamples; i++) {
		for (int32_t axis = 0; axis < 3; axis++) {
			const float v = input->channelData[axis][i];
			std::vector<float>& axisCuts = cuts[axis];
			if (!inside(v) || std::find(axisCuts.begin(), axisCuts.end(), v) != axisCuts.end())
				continue;

			int64_t cells = 1;
			for (int32_t a = 0; a < 3; a++)
				cells *= static_cast<int64_t>(cuts[a].size()) + 1 + (a == axis ? 1 : 0);
			if (cells <= maxCells)
				axisCuts.push_back(v);
		}
	}

	// Cuts are unique and strictly inside (-1, 1), so after sorting and adding the
	// upper boundary every consecutive pair bounds a cell of non-zero width.
	for (std::vector<float>& axisCuts : cuts) {
		std::sort(axisCuts.begin(), axisCuts.end());
		axisCuts.push_back(1.0f);
	}

	BatchedMesh mesh(output);
	Box cell;
	cell.lo[0] = -1.0f;
	for (const float xUpper : cuts[0]) {
		cell.hi[0] = xUpper;
		cell.lo[1] = -1.0f;
		for (const float yUpper : cuts[1]) {
			cell.hi[1] = yUpper;
			cell.lo[2] = -1.0f;
			for (const float zUpper : cuts[2]) {
				cell.hi[2] = zUpper;
				emitBox(cell, scale, spread, mesh);
				cell.lo[2] = zUpper;
			}
			cell.lo[1] = yUpper;
		}
		cell.lo[0] = xUpper;
	}
	mesh.flush();
}

void
ShapeGenerator::outputKDTree(const	OP_CHOPInput* input,
															float scale,
															float spread,
															SOP_Output* output) const
{
	// read points positions from input chop; samples outside the [-1, 1] box
	// can never split a cell, so drop them up front
	std::vector<Vec3> samples;
	samples.reserve(input->numSamples);
	for (int32_t i = 0; i < input->numSamples; i++) {
		const Vec3 sample = {
			input->channelData[0][i],
			input->channelData[1][i],
			input->channelData[2][i]
		};
		if (insideUnitBox(sample))
			samples.push_back(sample);
	}

	// Each pending cell owns the samples in [begin, end) of 'samples'. A cell is
	// split at its sample closest to the cell center, along axis depth % 3
	// (x, y, z), so a split plane on axis a always comes from channel a. Its
	// remaining samples are partitioned in place between the
	// two halves, so each sample is visited once per tree level. A cell with no
	// samples left is final.
	struct Cell
	{
		Box		box;
		int32_t	depth;
		size_t	begin;
		size_t	end;
	};
	std::vector<Cell> pending = { { theUnitBox, 0, 0, samples.size() } };
	BatchedMesh mesh(output);

	while (!pending.empty()) {
		const Cell cell = pending.back();
		pending.pop_back();

		if (cell.begin == cell.end) {
			emitBox(cell.box, scale, spread, mesh);
			continue;
		}

		const Vec3 center = {
			(cell.box.lo[0] + cell.box.hi[0]) * 0.5f,
			(cell.box.lo[1] + cell.box.hi[1]) * 0.5f,
			(cell.box.lo[2] + cell.box.hi[2]) * 0.5f
		};
		const auto first = samples.begin() + cell.begin;
		const auto last = samples.begin() + cell.end;
		const auto closest = std::min_element(first, last, [&](const Vec3& a, const Vec3& b) {
			return squareDistance(a, center) < squareDistance(b, center);
		});

		// move the splitting sample to the end of the range and drop it from there
		std::iter_swap(closest, last - 1);
		const int32_t axis = cell.depth % 3;
		const float split = (*(last - 1))[axis];
		const auto middle = std::partition(first, last - 1, [&](const Vec3& s) {
			return s[axis] <= split;
		});
		const size_t mid = cell.begin + static_cast<size_t>(middle - first);

		Cell lower = { cell.box, cell.depth + 1, cell.begin, mid };
		Cell upper = { cell.box, cell.depth + 1, mid, cell.end - 1 };
		lower.box.hi[axis] = split;
		upper.box.lo[axis] = split;
		pending.push_back(upper);
		pending.push_back(lower);
	}
	mesh.flush();
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

	// voro++ works best with about 5 particles per block, so aim for
	// numSamples / 5 blocks in total, split evenly over the three axes
	const int division = std::max(1, static_cast<int>(std::ceil(std::cbrt(input->numSamples / 5.0))));

	// create the Voronoi container
	voro::container con(
		x_min, x_max,
		y_min, y_max,
		z_min, z_max,
		division, division, division,
		false, false, false,
		8
	);

	// read voronoi points positions from input chop;
	// voro++ ignores points outside the container
	for (int i = 0; i < input->numSamples; i++) {
		con.put(i,
			input->channelData[0][i],
			input->channelData[1][i],
			input->channelData[2][i]
		);
	}

	BatchedMesh mesh(output);
	voro::voronoicell c;
	voro::c_loop_all cl(con);
	if (cl.start()) {
		do {
			if (con.compute_cell(c, cl)) {

				// get the position of the cell central point
				double x, y, z;
				cl.pos(x, y, z);
				Position center = { (float)x, (float)y, (float)z };
				const Position offset = spread > 0.0f ? center * spread : Position();

				// all cells are combined into one shape, so indices are shifted
				// by the number of points already added
				const int32_t base = mesh.nextPointIndex();

				// -------------------------------------------------------------------------
				// output vertices
				// NB! Voro++ shares vertexes between edges. So we currently can not add
				// normals, because the only way to do it now is to have separate vertexes
				// for each edge and add normal for each of these vertexes.
				// voro++ stores vertex positions doubled and relative to the cell center.
				// -------------------------------------------------------------------------
				double* ptsp = c.pts;
				for (int i = 0; i < c.p; i++, ptsp += 3) {
					Position pos = {
						(float)(x + ptsp[0] * 0.5),
						(float)(y + ptsp[1] * 0.5),
						(float)(z + ptsp[2] * 0.5)
					};
					mesh.addPoint(scaled(center, pos, offset, scale));
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
								mesh.addTriangle(base + i, base + k, base + m);
								k = m; l = n;
								m = c.ed[k][l]; c.ed[k][l] = -1 - m;
							}
						}
					}

				mesh.flushIfFull();
			} // if compute_cell
		} while (cl.inc());
	} // if cl.start()...
	mesh.flush();
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
	memcpy(output->addTriangles(theSquareNumPrim), theSquareVertices.data(), theSquareNumPrim * 3 * sizeof(int32_t));
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
