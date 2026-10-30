#include "CubeLattice.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

namespace sb
{

namespace
{

// Reorders constraints of K points each (edges, tets) by a greedy colouring: constraints of one colour
// share no point. In lattice order every constraint would wait for the previous one's result,
// grouped they can be solved overlapped.
template <int K, typename T>
void
groupByColour(std::vector<T>& items, size_t numPoints, const int32_t* (*pointsOf)(const T&))
{
	if (items.empty())
		return;
	std::vector<uint64_t> used(numPoints, 0);
	std::vector<uint8_t> colour(items.size());
	for (size_t i = 0; i < items.size(); ++i)
	{
		const int32_t* p = pointsOf(items[i]);
		uint64_t taken = 0;
		for (int k = 0; k < K; ++k)
			taken |= used[p[k]];
		uint8_t c = 0;
		while (c < 63 && ((taken >> c) & 1))
			++c;
		colour[i] = c;
		for (int k = 0; k < K; ++k)
			used[p[k]] |= 1ull << c;
	}
	std::vector<T> grouped;
	grouped.reserve(items.size());
	for (uint8_t c = 0, last = *std::max_element(colour.begin(), colour.end()); c <= last; ++c)
		for (size_t i = 0; i < items.size(); ++i)
			if (colour[i] == c)
				grouped.push_back(items[i]);
	items.swap(grouped);
}

CubeLattice
makeLattice(int n)
{
	CubeLattice l;
	l.resolution = n;
	const int m = n + 1;
	auto index = [m](int i, int j, int k) { return static_cast<int32_t>(i + m * (j + m * k)); };

	l.points.reserve(static_cast<size_t>(m) * m * m);
	for (int k = 0; k < m; ++k)
		for (int j = 0; j < m; ++j)
			for (int i = 0; i < m; ++i)
				l.points.emplace_back(2.0f * i / n - 1.0f, 2.0f * j / n - 1.0f, 2.0f * k / n - 1.0f);

	// 6 tets per cell, one per path from corner 000 to corner 111 along the three axes in some order.
	const int perms[6][3] = { {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0} };
	l.tets.reserve(static_cast<size_t>(n) * n * n * 24);
	for (int k = 0; k < n; ++k)
		for (int j = 0; j < n; ++j)
			for (int i = 0; i < n; ++i)
				for (const auto& p : perms)
				{
					int c[3] = { i, j, k };
					int32_t t[4];
					t[0] = index(c[0], c[1], c[2]);
					for (int s = 0; s < 3; ++s)
					{
						++c[p[s]];
						t[s + 1] = index(c[0], c[1], c[2]);
					}
					const Vec3& x0 = l.points[t[0]];
					if (dot(cross(l.points[t[1]] - x0, l.points[t[2]] - x0), l.points[t[3]] - x0) < 0.0f)
						std::swap(t[2], t[3]);
					l.tets.insert(l.tets.end(), t, t + 4);
				}

	// Edges: lattice edges, one diagonal per face and one per cell.
	std::vector<uint64_t> keys;
	keys.reserve(l.tets.size() / 4 * 6);
	for (size_t t = 0; t < l.tets.size(); t += 4)
		for (int a = 0; a < 4; ++a)
			for (int b = a + 1; b < 4; ++b)
			{
				uint64_t lo = static_cast<uint32_t>(std::min(l.tets[t + a], l.tets[t + b]));
				uint64_t hi = static_cast<uint32_t>(std::max(l.tets[t + a], l.tets[t + b]));
				keys.push_back(lo << 32 | hi);
			}
	std::sort(keys.begin(), keys.end());
	keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
	l.edges.reserve(keys.size());
	for (uint64_t key : keys)
		l.edges.emplace_back(static_cast<int32_t>(key >> 32), static_cast<int32_t>(key & 0xffffffffu));

	using EdgePair = std::pair<int32_t, int32_t>;
	groupByColour<2>(l.edges, l.points.size(), +[](const EdgePair& e) { return &e.first; });

	std::vector<std::array<int32_t, 4>> tets(l.tets.size() / 4);
	std::memcpy(tets.data(), l.tets.data(), l.tets.size() * sizeof(int32_t));
	groupByColour<4>(tets, l.points.size(), +[](const std::array<int32_t, 4>& t) { return t.data(); });
	std::memcpy(l.tets.data(), tets.data(), l.tets.size() * sizeof(int32_t));

	// Surface: per side, a grid of surface vertices. Axes (u, v) are chosen so u x v points along +axis;
	// quads are split along the diagonal that increases both u and v, the same one the tets use.
	std::vector<uint8_t> onSurface(l.points.size(), 0);
	for (int axis = 0; axis < 3; ++axis)
	{
		const int u = (axis + 1) % 3, v = (axis + 2) % 3;
		for (int side = 0; side < 2; ++side)
		{
			const int32_t first = static_cast<int32_t>(l.surfaceVertexPoint.size());
			for (int b = 0; b < m; ++b)
				for (int a = 0; a < m; ++a)
				{
					int c[3];
					c[axis] = side * n;
					c[u] = a;
					c[v] = b;
					const int32_t p = index(c[0], c[1], c[2]);
					l.surfaceVertexPoint.push_back(p);
					onSurface[p] = 1;
				}
			auto vert = [first, m](int a, int b) { return first + a + m * b; };
			for (int b = 0; b < n; ++b)
				for (int a = 0; a < n; ++a)
				{
					int32_t p00 = vert(a, b), p10 = vert(a + 1, b), p11 = vert(a + 1, b + 1), p01 = vert(a, b + 1);
					if (side == 1)
						l.surfaceTriangles.insert(l.surfaceTriangles.end(), { p00, p10, p11, p00, p11, p01 });
					else
						l.surfaceTriangles.insert(l.surfaceTriangles.end(), { p00, p11, p10, p00, p01, p11 });
				}
		}
	}
	for (size_t p = 0; p < l.points.size(); ++p)
		if (onSurface[p])
			l.surfacePoints.push_back(static_cast<int32_t>(p));

	l.maxSurfaceEdge = 2.0f * std::sqrt(2.0f) / static_cast<float>(n);
	return l;
}

}

const CubeLattice&
getUnitCubeLattice(int resolution)
{
	static std::map<int, std::unique_ptr<CubeLattice>> cache;
	static std::mutex lock;

	resolution = std::clamp(resolution, 1, MaxLatticeResolution);
	std::lock_guard<std::mutex> guard(lock);
	std::unique_ptr<CubeLattice>& entry = cache[resolution];
	if (!entry)
		entry = std::make_unique<CubeLattice>(makeLattice(resolution));
	return *entry;
}

int
latticeResolutionAtLeast(int points)
{
	int n = 1;
	while (n < MaxLatticeResolution && latticePointCount(n) < points)
		++n;
	return n;
}

int
latticeResolutionAtMost(int points)
{
	int n = 1;
	while (n < MaxLatticeResolution && latticePointCount(n + 1) <= points)
		++n;
	return n;
}

int
latticeResolutionFor(float radius, float targetEdge, int minResolution, int maxResolution)
{
	minResolution = std::clamp(minResolution, 1, MaxLatticeResolution);
	maxResolution = std::clamp(maxResolution, minResolution, MaxLatticeResolution);
	if (targetEdge <= 0.0f || radius <= 0.0f)
		return minResolution;

	int n = static_cast<int>(std::ceil(2.0f * radius / targetEdge - 1e-4f));
	return std::clamp(n, minResolution, maxResolution);
}

}
