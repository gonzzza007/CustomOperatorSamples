#include "ContainerSDF.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sb
{

Vec3
closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c)
{
	Vec3 ab = b - a, ac = c - a, ap = p - a;
	float d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
		return a;

	Vec3 bp = p - b;
	float d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
		return b;

	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		return a + ab * (d1 / (d1 - d3));

	Vec3 cp = p - c;
	float d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
		return c;

	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		return a + ac * (d2 / (d2 - d6));

	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom, w = vc * denom;
	return a + ab * v + ac * w;
}

void
ContainerSDF::clear()
{
	myPhi.clear();
	myNx = myNy = myNz = 0;
	myDx = 0.0f;
}

bool
ContainerSDF::build(const std::vector<Vec3>& pts, const std::vector<int32_t>& tris, int resolution)
{
	clear();
	const size_t numTris = tris.size() / 3;
	if (pts.empty() || numTris == 0)
		return false;

	Vec3 bmin = pts[0], bmax = pts[0];
	for (const Vec3& p : pts)
	{
		bmin = vmin(bmin, p);
		bmax = vmax(bmax, p);
	}
	Vec3	size = bmax - bmin;
	float	longest = std::max(size.x, std::max(size.y, size.z));
	if (longest <= 0.0f)
		return false;

	resolution = std::clamp(resolution, 8, 512);
	const int pad = 3;
	myDx = longest / static_cast<float>(resolution);
	myOrigin = bmin - Vec3(1.0f, 1.0f, 1.0f) * (myDx * pad);
	myNx = static_cast<int>(std::ceil(size.x / myDx)) + 2 * pad + 1;
	myNy = static_cast<int>(std::ceil(size.y / myDx)) + 2 * pad + 1;
	myNz = static_cast<int>(std::ceil(size.z / myDx)) + 2 * pad + 1;

	const size_t numCells = static_cast<size_t>(myNx) * myNy * myNz;
	const float	 farAway = (myNx + myNy + myNz) * myDx;
	myPhi.assign(numCells, farAway);
	std::vector<int32_t> closest(numCells, -1);

	auto index = [&](int i, int j, int k) { return (static_cast<size_t>(k) * myNy + j) * myNx + i; };
	auto cellPos = [&](int i, int j, int k)
	{
		return myOrigin + Vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) * myDx;
	};
	auto triDist = [&](const Vec3& p, int32_t t)
	{
		const Vec3& a = pts[tris[3 * t]];
		const Vec3& b = pts[tris[3 * t + 1]];
		const Vec3& c = pts[tris[3 * t + 2]];
		return length(p - closestPointOnTriangle(p, a, b, c));
	};
	auto toCell = [&](float v, float o, int n)
	{
		return std::clamp(static_cast<int>(std::floor((v - o) / myDx)), 0, n - 1);
	};

	// 1. Exact distances in a narrow band around every triangle.
	const int band = 2;
	for (size_t t = 0; t < numTris; ++t)
	{
		const Vec3& a = pts[tris[3 * t]];
		const Vec3& b = pts[tris[3 * t + 1]];
		const Vec3& c = pts[tris[3 * t + 2]];
		Vec3 tmin = vmin(a, vmin(b, c)), tmax = vmax(a, vmax(b, c));

		int i0 = std::max(toCell(tmin.x, myOrigin.x, myNx) - band, 0);
		int j0 = std::max(toCell(tmin.y, myOrigin.y, myNy) - band, 0);
		int k0 = std::max(toCell(tmin.z, myOrigin.z, myNz) - band, 0);
		int i1 = std::min(toCell(tmax.x, myOrigin.x, myNx) + band + 1, myNx - 1);
		int j1 = std::min(toCell(tmax.y, myOrigin.y, myNy) + band + 1, myNy - 1);
		int k1 = std::min(toCell(tmax.z, myOrigin.z, myNz) + band + 1, myNz - 1);

		for (int k = k0; k <= k1; ++k)
			for (int j = j0; j <= j1; ++j)
				for (int i = i0; i <= i1; ++i)
				{
					float d = triDist(cellPos(i, j, k), static_cast<int32_t>(t));
					size_t idx = index(i, j, k);
					if (d < myPhi[idx])
					{
						myPhi[idx] = d;
						closest[idx] = static_cast<int32_t>(t);
					}
				}
	}

	// 2. Fast sweeping: propagate the closest triangle to the rest of the grid.
	auto check = [&](int i, int j, int k, int ni, int nj, int nk)
	{
		int32_t t = closest[index(ni, nj, nk)];
		size_t idx = index(i, j, k);
		if (t < 0 || t == closest[idx])
			return;
		float d = triDist(cellPos(i, j, k), t);
		if (d < myPhi[idx])
		{
			myPhi[idx] = d;
			closest[idx] = t;
		}
	};
	for (int pass = 0; pass < 2; ++pass)
	{
		for (int dir = 0; dir < 8; ++dir)
		{
			int di = (dir & 1) ? -1 : 1;
			int dj = (dir & 2) ? -1 : 1;
			int dk = (dir & 4) ? -1 : 1;
			int is = di > 0 ? 1 : myNx - 2, ie = di > 0 ? myNx : -1;
			int js = dj > 0 ? 1 : myNy - 2, je = dj > 0 ? myNy : -1;
			int ks = dk > 0 ? 1 : myNz - 2, ke = dk > 0 ? myNz : -1;
			for (int k = ks; k != ke; k += dk)
				for (int j = js; j != je; j += dj)
					for (int i = is; i != ie; i += di)
					{
						check(i, j, k, i - di, j, k);
						check(i, j, k, i, j - dj, k);
						check(i, j, k, i, j, k - dk);
						check(i, j, k, i - di, j - dj, k);
						check(i, j, k, i - di, j, k - dk);
						check(i, j, k, i, j - dj, k - dk);
						check(i, j, k, i - di, j - dj, k - dk);
					}
		}
	}

	// 3. Inside/outside by ray parity along +X. The rays are nudged by a tiny irrational
	// offset so that axis-aligned meshes (a Box SOP) never hit triangle edges exactly.
	const float jy = myDx * 1.37e-3f, jz = myDx * 2.71e-3f;
	std::vector<std::vector<float>> hits(static_cast<size_t>(myNy) * myNz);
	for (size_t t = 0; t < numTris; ++t)
	{
		const Vec3& a = pts[tris[3 * t]];
		const Vec3& b = pts[tris[3 * t + 1]];
		const Vec3& c = pts[tris[3 * t + 2]];
		Vec3 tmin = vmin(a, vmin(b, c)), tmax = vmax(a, vmax(b, c));

		int j0 = std::max(static_cast<int>(std::floor((tmin.y - myOrigin.y) / myDx)), 0);
		int k0 = std::max(static_cast<int>(std::floor((tmin.z - myOrigin.z) / myDx)), 0);
		int j1 = std::min(static_cast<int>(std::ceil((tmax.y - myOrigin.y) / myDx)), myNy - 1);
		int k1 = std::min(static_cast<int>(std::ceil((tmax.z - myOrigin.z) / myDx)), myNz - 1);

		for (int k = k0; k <= k1; ++k)
			for (int j = j0; j <= j1; ++j)
			{
				float py = myOrigin.y + j * myDx + jy;
				float pz = myOrigin.z + k * myDx + jz;

				// 2D barycentric test in the YZ plane.
				double ay = a.y - py, az = a.z - pz;
				double by = b.y - py, bz = b.z - pz;
				double cy = c.y - py, cz = c.z - pz;
				double wa = by * cz - bz * cy;
				double wb = cy * az - cz * ay;
				double wc = ay * bz - az * by;
				bool inside = (wa >= 0 && wb >= 0 && wc >= 0) || (wa <= 0 && wb <= 0 && wc <= 0);
				double sum = wa + wb + wc;
				if (!inside || std::abs(sum) < 1e-20)
					continue;
				double x = (wa * a.x + wb * b.x + wc * c.x) / sum;
				hits[static_cast<size_t>(k) * myNy + j].push_back(static_cast<float>(x));
			}
	}
	for (int k = 0; k < myNz; ++k)
		for (int j = 0; j < myNy; ++j)
		{
			std::vector<float>& row = hits[static_cast<size_t>(k) * myNy + j];
			if (row.empty())
				continue;
			std::sort(row.begin(), row.end());
			size_t h = 0;
			for (int i = 0; i < myNx; ++i)
			{
				float x = myOrigin.x + i * myDx;
				while (h < row.size() && row[h] < x)
					++h;
				if (h & 1)
					myPhi[index(i, j, k)] = -myPhi[index(i, j, k)];
			}
		}

	return true;
}

Vec3
ContainerSDF::boundsMax() const
{
	return myOrigin + Vec3(static_cast<float>(myNx - 1), static_cast<float>(myNy - 1), static_cast<float>(myNz - 1)) * myDx;
}

float
ContainerSDF::distance(const Vec3& p) const
{
	Vec3 g;
	return distanceAndGradient(p, g);
}

float
ContainerSDF::distanceAndGradient(const Vec3& p, Vec3& gradient) const
{
	if (myPhi.empty())
	{
		gradient = Vec3(0.0f, 1.0f, 0.0f);
		return -std::numeric_limits<float>::max();
	}

	// Clamp into the grid; account for the distance we were clamped by.
	Vec3 lo = myOrigin, hi = boundsMax();
	Vec3 q = vmin(vmax(p, lo), hi);
	float outside = length(p - q);

	float fx = (q.x - myOrigin.x) / myDx;
	float fy = (q.y - myOrigin.y) / myDx;
	float fz = (q.z - myOrigin.z) / myDx;
	int i = std::clamp(static_cast<int>(fx), 0, myNx - 2);
	int j = std::clamp(static_cast<int>(fy), 0, myNy - 2);
	int k = std::clamp(static_cast<int>(fz), 0, myNz - 2);
	float tx = fx - i, ty = fy - j, tz = fz - k;

	float c000 = at(i, j, k), c100 = at(i + 1, j, k), c010 = at(i, j + 1, k), c110 = at(i + 1, j + 1, k);
	float c001 = at(i, j, k + 1), c101 = at(i + 1, j, k + 1), c011 = at(i, j + 1, k + 1), c111 = at(i + 1, j + 1, k + 1);

	float c00 = c000 + (c100 - c000) * tx, c10 = c010 + (c110 - c010) * tx;
	float c01 = c001 + (c101 - c001) * tx, c11 = c011 + (c111 - c011) * tx;
	float c0 = c00 + (c10 - c00) * ty, c1 = c01 + (c11 - c01) * ty;
	float phi = c0 + (c1 - c0) * tz;

	// Analytic derivative of the trilinear interpolant.
	float dx = ((c100 - c000) * (1 - ty) + (c110 - c010) * ty) * (1 - tz)
			 + ((c101 - c001) * (1 - ty) + (c111 - c011) * ty) * tz;
	float dy = (c10 - c00) * (1 - tz) + (c11 - c01) * tz;
	float dz = c1 - c0;
	gradient = normalize(Vec3(dx, dy, dz));

	if (outside > 0.0f)
	{
		// Outside the grid: push back towards the grid.
		gradient = normalize(p - q);
		return std::max(phi, 0.0f) + outside;
	}
	return phi;
}

}
