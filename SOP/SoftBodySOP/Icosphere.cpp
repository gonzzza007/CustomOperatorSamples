#include "Icosphere.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <utility>

namespace sb
{

namespace
{

Icosphere
makeIcosahedron()
{
	Icosphere s;
	const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
	const float p[12][3] = {
		{-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
		{ 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
		{ t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
	};
	for (const auto& v : p)
		s.points.push_back(normalize(Vec3(v[0], v[1], v[2])));

	const int32_t f[20][3] = {
		{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
		{1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
		{3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
		{4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
	};
	for (const auto& tri : f)
		s.triangles.insert(s.triangles.end(), { tri[0], tri[1], tri[2] });
	return s;
}

Icosphere
subdivide(const Icosphere& in)
{
	Icosphere out;
	out.points = in.points;
	std::map<std::pair<int32_t, int32_t>, int32_t> midpoints;

	auto mid = [&](int32_t a, int32_t b) -> int32_t
	{
		auto key = std::make_pair(std::min(a, b), std::max(a, b));
		auto it = midpoints.find(key);
		if (it != midpoints.end())
			return it->second;
		int32_t idx = static_cast<int32_t>(out.points.size());
		out.points.push_back(normalize(out.points[a] + out.points[b]));
		midpoints[key] = idx;
		return idx;
	};

	for (size_t i = 0; i < in.triangles.size(); i += 3)
	{
		int32_t a = in.triangles[i], b = in.triangles[i + 1], c = in.triangles[i + 2];
		int32_t ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
		out.triangles.insert(out.triangles.end(), { a, ab, ca });
		out.triangles.insert(out.triangles.end(), { b, bc, ab });
		out.triangles.insert(out.triangles.end(), { c, ca, bc });
		out.triangles.insert(out.triangles.end(), { ab, bc, ca });
	}
	return out;
}

constexpr int MaxLevel = 7;

}

const Icosphere&
getUnitIcosphere(int level)
{
	static std::array<std::unique_ptr<Icosphere>, MaxLevel + 1> cache;
	static std::mutex lock;

	level = std::clamp(level, 0, MaxLevel);
	std::lock_guard<std::mutex> guard(lock);
	if (!cache[level])
	{
		if (level == 0)
			cache[0] = std::make_unique<Icosphere>(makeIcosahedron());
		else
		{
			// Build the chain up to this level.
			for (int l = 0; l <= level; ++l)
			{
				if (cache[l])
					continue;
				cache[l] = std::make_unique<Icosphere>(l == 0 ? makeIcosahedron() : subdivide(*cache[l - 1]));
			}
		}
	}
	return *cache[level];
}

int
icosphereLevelFor(float radius, float targetEdge, int minLevel, int maxLevel)
{
	minLevel = std::clamp(minLevel, 0, MaxLevel);
	maxLevel = std::clamp(maxLevel, minLevel, MaxLevel);
	if (targetEdge <= 0.0f || radius <= 0.0f)
		return minLevel;

	// Each subdivision halves the edge length.
	float	ratio = IcosahedronEdge * radius / targetEdge;
	int		level = ratio <= 1.0f ? 0 : static_cast<int>(std::ceil(std::log2(ratio)));
	return std::clamp(level, minLevel, maxLevel);
}

}
