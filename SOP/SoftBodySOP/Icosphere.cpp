#include "Icosphere.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>

namespace sb
{

namespace
{

const float		T = (1.0f + std::sqrt(5.0f)) * 0.5f;

const float		IcoPoints[12][3] = {
	{-1,  T,  0}, { 1,  T,  0}, {-1, -T,  0}, { 1, -T,  0},
	{ 0, -1,  T}, { 0,  1,  T}, { 0, -1, -T}, { 0,  1, -T},
	{ T,  0, -1}, { T,  0,  1}, {-T,  0, -1}, {-T,  0,  1}
};

const int32_t	IcoFaces[20][3] = {
	{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
	{1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
	{3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
	{4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
};

Icosphere
makeGeodesic(int n)
{
	Icosphere s;

	// Points on shared edges and corners must be created once. A point of face (A, B, C) at grid
	// position (i, j) is A + (B - A) i/n + (C - A) j/n; identify it exactly by its barycentric weights
	// on the icosahedron corners, sorted by corner index.
	using Key = std::tuple<int32_t, int32_t, int32_t, int32_t, int32_t, int32_t>;
	std::map<Key, int32_t> lookup;

	auto pointIndex = [&](const int32_t corner[3], int i, int j) -> int32_t
	{
		int w[3] = { n - i - j, i, j };
		std::pair<int32_t, int32_t> cw[3] = {
			{ corner[0], w[0] }, { corner[1], w[1] }, { corner[2], w[2] }
		};
		std::sort(std::begin(cw), std::end(cw));
		// Drop zero weights so a point on an edge or corner gets the same key from every face.
		int32_t k[6] = { -1, 0, -1, 0, -1, 0 };
		int m = 0;
		for (const auto& c : cw)
		{
			if (c.second == 0)
				continue;
			k[2 * m] = c.first;
			k[2 * m + 1] = c.second;
			++m;
		}
		Key key{ k[0], k[1], k[2], k[3], k[4], k[5] };
		auto it = lookup.find(key);
		if (it != lookup.end())
			return it->second;

		Vec3 p;
		for (int c = 0; c < 3; ++c)
		{
			const float* q = IcoPoints[corner[c]];
			p += Vec3(q[0], q[1], q[2]) * static_cast<float>(w[c]);
		}
		int32_t idx = static_cast<int32_t>(s.points.size());
		s.points.push_back(normalize(p));
		lookup[key] = idx;
		return idx;
	};

	for (const auto& face : IcoFaces)
	{
		for (int j = 0; j < n; ++j)
		{
			for (int i = 0; i + j < n; ++i)
			{
				int32_t a = pointIndex(face, i, j);
				int32_t b = pointIndex(face, i + 1, j);
				int32_t c = pointIndex(face, i, j + 1);
				s.triangles.insert(s.triangles.end(), { a, b, c });
				if (i + j + 1 < n)
				{
					int32_t d = pointIndex(face, i + 1, j + 1);
					s.triangles.insert(s.triangles.end(), { b, d, c });
				}
			}
		}
	}
	return s;
}

}

const Icosphere&
getUnitGeodesicSphere(int frequency)
{
	static std::map<int, std::unique_ptr<Icosphere>> cache;
	static std::mutex lock;

	frequency = std::clamp(frequency, 1, MaxGeodesicFrequency);
	std::lock_guard<std::mutex> guard(lock);
	std::unique_ptr<Icosphere>& entry = cache[frequency];
	if (!entry)
		entry = std::make_unique<Icosphere>(makeGeodesic(frequency));
	return *entry;
}

int
geodesicFrequencyAtLeast(int points)
{
	int f = 1;
	while (f < MaxGeodesicFrequency && geodesicPointCount(f) < points)
		++f;
	return f;
}

int
geodesicFrequencyAtMost(int points)
{
	int f = 1;
	while (f < MaxGeodesicFrequency && geodesicPointCount(f + 1) <= points)
		++f;
	return f;
}

int
geodesicFrequencyFor(float radius, float targetEdge, int minFrequency, int maxFrequency)
{
	minFrequency = std::clamp(minFrequency, 1, MaxGeodesicFrequency);
	maxFrequency = std::clamp(maxFrequency, minFrequency, MaxGeodesicFrequency);
	if (targetEdge <= 0.0f || radius <= 0.0f)
		return minFrequency;

	// Splitting each icosahedron edge into n parts gives edges of about IcosahedronEdge * r / n.
	int f = static_cast<int>(std::ceil(IcosahedronEdge * radius / targetEdge - 1e-4f));
	return std::clamp(f, minFrequency, maxFrequency);
}

}
