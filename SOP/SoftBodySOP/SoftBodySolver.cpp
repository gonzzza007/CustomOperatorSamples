#include "SoftBodySolver.h"
#include "Icosphere.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace sb
{

namespace
{

constexpr float Pi = 3.14159265358979f;

// --- Small quaternion helpers for shape matching ---------------------------------------

void
quatToMat(const float q[4], Mat3& r)
{
	float x = q[0], y = q[1], z = q[2], w = q[3];
	r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - z * w);     r.m[0][2] = 2 * (x * z + y * w);
	r.m[1][0] = 2 * (x * y + z * w);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - x * w);
	r.m[2][0] = 2 * (x * z - y * w);     r.m[2][1] = 2 * (y * z + x * w);     r.m[2][2] = 1 - 2 * (x * x + y * y);
}

void
quatMul(const float a[4], const float b[4], float out[4])
{
	float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
	float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
	float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
	float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	out[0] = x; out[1] = y; out[2] = z; out[3] = w;
}

// Rotational part of A, warm-started from q.
// Mueller et al. 2016, "A Robust Method to Extract the Rotational Part of Deformations".
void
extractRotation(const Mat3& A, float q[4], int maxIter)
{
	for (int iter = 0; iter < maxIter; ++iter)
	{
		Mat3 R;
		quatToMat(q, R);
		Vec3 omega = cross(R.col(0), A.col(0)) + cross(R.col(1), A.col(1)) + cross(R.col(2), A.col(2));
		float denom = std::abs(dot(R.col(0), A.col(0)) + dot(R.col(1), A.col(1)) + dot(R.col(2), A.col(2))) + 1e-9f;
		omega = omega / denom;
		float w = length(omega);
		if (w < 1e-9f)
			break;
		Vec3 axis = omega / w;
		float s = std::sin(w * 0.5f);
		float dq[4] = { axis.x * s, axis.y * s, axis.z * s, std::cos(w * 0.5f) };
		float nq[4];
		quatMul(dq, q, nq);
		float len = std::sqrt(nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3]);
		for (int i = 0; i < 4; ++i)
			q[i] = nq[i] / len;
	}
}

// Moves p by dpTangential to cancel sliding, Coulomb-style (Macklin et al. 2014).
Vec3
frictionCorrection(const Vec3& relDisp, const Vec3& n, float penetration, float friction)
{
	if (friction <= 0.0f || penetration <= 0.0f)
		return Vec3();
	Vec3 dt = relDisp - n * dot(relDisp, n);
	float lt = length(dt);
	if (lt < 1e-12f)
		return Vec3();
	float staticLimit = friction * penetration;
	if (lt < staticLimit)
		return dt;
	return dt * std::min(friction * penetration / lt, 1.0f);
}

int64_t
cellKey(int x, int y, int z)
{
	return (static_cast<int64_t>(x) * 73856093) ^ (static_cast<int64_t>(y) * 19349663) ^ (static_cast<int64_t>(z) * 83492791);
}

}

// ---------------------------------------------------------------------------------------

float
SoftBodySolver::complianceFromStiffness(float stiffness)
{
	// Mass-normalised compliance in s^2: 1e-2 (jelly) .. 1e-8 (nearly rigid), 0 at stiffness 1.
	stiffness = std::clamp(stiffness, 0.0f, 1.0f);
	if (stiffness >= 0.999f)
		return 0.0f;
	return 1e-2f * std::pow(1e-6f, stiffness);
}

ResetReport
SoftBodySolver::reset(const std::vector<Vec3>& seeds, const std::vector<SeedOverrides>& overrides,
						const GenerationSettings& gen, const ContainerSDF* container)
{
	ResetReport report;

	myX.clear(); myPrev.clear(); myV.clear(); myVPre.clear(); myRest.clear(); myContactN.clear();
	myW.clear(); myBlobOf.clear(); myInContact.clear();
	myTriangles.clear(); myEdges.clear(); myEdgeLambda.clear();
	myBlobs.clear(); myVolumeLambda.clear(); myPairs.clear();

	const float edgeLength = std::max(gen.edgeLength, 1e-4f);
	myParticleRadius = gen.collisionRadius > 0.0f ? gen.collisionRadius : edgeLength * 0.5f;
	const float pr = myParticleRadius;

	float rMin = std::max(std::min(gen.radiusMin, gen.radiusMax), 1e-4f);
	float rMax = std::max(gen.radiusMin, gen.radiusMax);
	const float smallest = std::max(rMin * 0.25f, pr * 2.0f);	// below this a blob is not worth creating

	// 1. Desired radii (random draws happen for every seed so the sequence is stable).
	std::mt19937 rng(gen.seed);
	std::uniform_real_distribution<float> uni(rMin, rMax);
	const size_t n = seeds.size();
	std::vector<float> radius(n);
	for (size_t i = 0; i < n; ++i)
	{
		float r = uni(rng);
		if (i < overrides.size() && overrides[i].radius > 0.0f)
			r = overrides[i].radius;
		radius[i] = r;
	}

	// 2. Shrink radii so blobs neither overlap each other nor the container walls.
	std::vector<uint8_t> accepted(n, 0);
	for (size_t i = 0; i < n; ++i)
	{
		float r = radius[i];
		if (container && container->isValid())
		{
			float phi = container->distance(seeds[i]);
			if (phi >= 0.0f)
			{
				++report.seedsOutside;
				continue;
			}
			r = std::min(r, -phi - pr);
		}
		for (size_t j = 0; j < n && r > 0.0f; ++j)
		{
			if (j == i)
				continue;
			float d = length(seeds[i] - seeds[j]);
			if (j < i)
			{
				if (accepted[j])
					r = std::min(r, d - radius[j] - 2.0f * pr);
			}
			else
				r = std::min(r, 0.5f * d - pr);
		}
		if (r < smallest)
		{
			++report.seedsTooTight;
			continue;
		}
		radius[i] = r;
		accepted[i] = 1;
	}

	// 3. Build the blobs.
	for (size_t i = 0; i < n; ++i)
	{
		if (!accepted[i])
			continue;

		const float	r = radius[i];
		const int	level = icosphereLevelFor(r, edgeLength, gen.minLevel, gen.maxLevel);
		const Icosphere& ico = getUnitIcosphere(level);

		Blob blob;
		blob.pointStart = static_cast<int32_t>(myX.size());
		blob.pointCount = static_cast<int32_t>(ico.points.size());
		blob.triStart = static_cast<int32_t>(myTriangles.size() / 3);
		blob.triCount = static_cast<int32_t>(ico.triangles.size() / 3);

		float mass = gen.massMode == MassMode::Density ? gen.mass * (4.0f / 3.0f) * Pi * r * r * r : gen.mass;
		if (i < overrides.size())
		{
			if (overrides[i].mass > 0.0f)
				mass = overrides[i].mass;
			if (overrides[i].stiffness >= 0.0f)
				blob.stiffness = std::clamp(overrides[i].stiffness, 0.0f, 1.0f);
		}
		blob.mass = std::max(mass, 1e-6f);
		const float w = static_cast<float>(blob.pointCount) / blob.mass;
		const int32_t blobId = static_cast<int32_t>(myBlobs.size());

		for (const Vec3& u : ico.points)
		{
			myX.push_back(seeds[i] + u * r);
			myRest.push_back(u * r);
			myW.push_back(w);
			myBlobOf.push_back(blobId);
		}
		for (int32_t idx : ico.triangles)
			myTriangles.push_back(idx + blob.pointStart);

		// Each edge of a closed, consistently wound mesh appears once in each direction:
		// keeping only a < b gives every edge exactly once.
		for (size_t t = 0; t < ico.triangles.size(); t += 3)
		{
			for (int e = 0; e < 3; ++e)
			{
				int32_t a = ico.triangles[t + e], b = ico.triangles[t + (e + 1) % 3];
				if (a < b)
				{
					Edge edge;
					edge.a = a + blob.pointStart;
					edge.b = b + blob.pointStart;
					edge.rest = length(ico.points[a] - ico.points[b]) * r;
					edge.blob = blobId;
					myEdges.push_back(edge);
				}
			}
		}

		myBlobs.push_back(blob);
		myBlobs.back().restVolume = blobVolume(myBlobs.back());
		++report.blobsCreated;
	}

	const size_t np = myX.size();
	myPrev = myX;
	myV.assign(np, Vec3());
	myVPre.assign(np, Vec3());
	myContactN.assign(np, Vec3());
	myInContact.assign(np, 0);
	myEdgeLambda.assign(myEdges.size(), 0.0f);
	myVolumeLambda.assign(myBlobs.size(), 0.0f);
	return report;
}

float
SoftBodySolver::blobVolume(const Blob& b) const
{
	float v = 0.0f;
	for (int32_t t = b.triStart; t < b.triStart + b.triCount; ++t)
	{
		const Vec3& p0 = myX[myTriangles[3 * t]];
		const Vec3& p1 = myX[myTriangles[3 * t + 1]];
		const Vec3& p2 = myX[myTriangles[3 * t + 2]];
		v += dot(p0, cross(p1, p2));
	}
	return v / 6.0f;
}

float
SoftBodySolver::currentVolumeRatio() const
{
	if (myBlobs.empty())
		return 1.0f;
	float sum = 0.0f;
	for (const Blob& b : myBlobs)
		sum += b.restVolume != 0.0f ? blobVolume(b) / b.restVolume : 1.0f;
	return sum / static_cast<float>(myBlobs.size());
}

void
SoftBodySolver::step(float dt, const SimSettings& sim, const ContainerSDF* container)
{
	if (myX.empty() || dt <= 0.0f)
		return;

	dt = std::min(dt, std::max(sim.maxTimeStep, 1e-4f));
	const int	substeps = std::clamp(sim.substeps, 1, 64);
	const int	iterations = std::clamp(sim.iterations, 1, 100);
	const float	h = dt / static_cast<float>(substeps);
	const float	h2 = h * h;
	const size_t np = myX.size();

	const float edgeCompliance = complianceFromStiffness(sim.stiffness);
	// Volume Stiffness 0 should still give a squashy blob, not a deflated bag: use the upper part of the range.
	const float volCompliance = complianceFromStiffness(0.4f + 0.6f * std::clamp(sim.volumeStiffness, 0.0f, 1.0f));
	const float shapeK = std::clamp(sim.shapeMemory, 0.0f, 1.0f);
	const float shapeKIter = shapeK > 0.0f ? 1.0f - std::pow(1.0f - std::min(shapeK, 0.999f), 1.0f / static_cast<float>(iterations)) : 0.0f;
	const float friction = std::clamp(sim.friction, 0.0f, 1.0f);
	const float restitution = std::clamp(sim.restitution, 0.0f, 1.0f);
	const float dampFactor = std::exp(-std::max(sim.damping, 0.0f) * h);
	const bool	useContainer = container && container->isValid();

	for (int s = 0; s < substeps; ++s)
	{
		// Predict
		for (size_t i = 0; i < np; ++i)
		{
			myVPre[i] = myV[i];
			myV[i] += sim.gravity * h;
			myPrev[i] = myX[i];
			myX[i] += myV[i] * h;
			myInContact[i] = 0;
		}

		buildContactPairs();

		// Only particles that start the substep near a wall can reach it during the substep.
		if (useContainer)
		{
			myNearWall.clear();
			const float reach = 3.0f * myParticleRadius;
			for (size_t i = 0; i < np; ++i)
			{
				float travel = length(myX[i] - myPrev[i]);
				if (container->distance(myX[i]) > -(reach + 2.0f * travel))
					myNearWall.push_back(static_cast<uint32_t>(i));
			}
		}
		std::fill(myEdgeLambda.begin(), myEdgeLambda.end(), 0.0f);
		std::fill(myVolumeLambda.begin(), myVolumeLambda.end(), 0.0f);

		for (int it = 0; it < iterations; ++it)
		{
			solveEdges(h2, edgeCompliance);
			solveVolumes(h2, volCompliance, std::max(sim.pressure, 0.01f));
			if (shapeKIter > 0.0f)
				solveShapeMatching(shapeKIter);
			solveContacts(friction);
			if (useContainer)
				solveContainer(*container, friction);
		}

		// Update velocities
		const float bounceThreshold = 2.0f * length(sim.gravity) * h;
		for (size_t i = 0; i < np; ++i)
		{
			myV[i] = (myX[i] - myPrev[i]) / h;

			if (myInContact[i] && restitution > 0.0f)
			{
				const Vec3& n = myContactN[i];			// points out of the container
				float vnPre = dot(myVPre[i], n);		// > 0: was moving into the wall
				if (vnPre > bounceThreshold)
				{
					float vn = dot(myV[i], n);
					myV[i] += n * (-restitution * vnPre - vn);
				}
			}
			myV[i] *= dampFactor;
		}
	}
}

void
SoftBodySolver::buildContactPairs()
{
	myPairs.clear();
	if (myBlobs.size() < 2)
		return;

	const size_t np = myX.size();
	const float	searchDist = 2.0f * myParticleRadius * 1.5f;	// margin: pairs stay valid for the whole substep
	const float	searchDist2 = searchDist * searchDist;
	const float	inv = 1.0f / searchDist;						// cell size = search distance

	// Counting sort of particles into a hash table.
	const uint32_t tableSize = static_cast<uint32_t>(std::max<size_t>(np, 64));
	myCellStart.assign(tableSize + 1, 0);
	myCellEntries.resize(np);

	auto hashOf = [&](int cx, int cy, int cz)
	{
		return static_cast<uint32_t>(static_cast<uint64_t>(cellKey(cx, cy, cz)) % tableSize);
	};
	auto cellOf = [&](const Vec3& p, int& cx, int& cy, int& cz)
	{
		cx = static_cast<int>(std::floor(p.x * inv));
		cy = static_cast<int>(std::floor(p.y * inv));
		cz = static_cast<int>(std::floor(p.z * inv));
	};

	// Broad phase: only particles inside the (padded) bounding box of another blob can touch it.
	const size_t nb = myBlobs.size();
	myBlobMin.resize(nb);
	myBlobMax.resize(nb);
	for (size_t b = 0; b < nb; ++b)
	{
		const Blob& blob = myBlobs[b];
		Vec3 lo = myX[blob.pointStart], hi = lo;
		for (int32_t i = blob.pointStart + 1; i < blob.pointStart + blob.pointCount; ++i)
		{
			lo = vmin(lo, myX[i]);
			hi = vmax(hi, myX[i]);
		}
		Vec3 pad(searchDist, searchDist, searchDist);
		myBlobMin[b] = lo - pad;
		myBlobMax[b] = hi + pad;
	}
	myActive.clear();
	for (size_t b = 0; b < nb; ++b)
	{
		// Union of the other blobs' boxes that overlap this one.
		std::vector<size_t>& others = myScratchOthers;
		others.clear();
		for (size_t o = 0; o < nb; ++o)
		{
			if (o == b)
				continue;
			const Vec3 &a0 = myBlobMin[b], &a1 = myBlobMax[b], &b0 = myBlobMin[o], &b1 = myBlobMax[o];
			if (a0.x <= b1.x && a1.x >= b0.x && a0.y <= b1.y && a1.y >= b0.y && a0.z <= b1.z && a1.z >= b0.z)
				others.push_back(o);
		}
		if (others.empty())
			continue;
		const Blob& blob = myBlobs[b];
		for (int32_t i = blob.pointStart; i < blob.pointStart + blob.pointCount; ++i)
		{
			const Vec3& p = myX[i];
			for (size_t o : others)
			{
				const Vec3 &lo = myBlobMin[o], &hi = myBlobMax[o];
				if (p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z)
				{
					myActive.push_back(static_cast<uint32_t>(i));
					break;
				}
			}
		}
	}
	if (myActive.empty())
		return;

	std::vector<uint32_t>& hashes = myScratchHashes;
	hashes.assign(np, 0);
	for (uint32_t i : myActive)
	{
		int cx, cy, cz;
		cellOf(myX[i], cx, cy, cz);
		hashes[i] = hashOf(cx, cy, cz);
		myCellStart[hashes[i]]++;
	}
	uint32_t sum = 0;
	for (uint32_t c = 0; c < tableSize; ++c)
	{
		sum += myCellStart[c];
		myCellStart[c] = sum;
	}
	myCellStart[tableSize] = sum;
	for (uint32_t i : myActive)
		myCellEntries[--myCellStart[hashes[i]]] = i;

	// Query the 27 neighbouring cells.
	for (uint32_t i : myActive)
	{
		int cx, cy, cz;
		cellOf(myX[i], cx, cy, cz);
		const int32_t bi = myBlobOf[i];
		for (int dz = -1; dz <= 1; ++dz)
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					uint32_t hsh = hashOf(cx + dx, cy + dy, cz + dz);
					for (uint32_t e = myCellStart[hsh]; e < myCellStart[hsh + 1]; ++e)
					{
						uint32_t j = myCellEntries[e];
						if (j <= i || myBlobOf[j] == bi)
							continue;
						if (length2(myX[i] - myX[j]) < searchDist2)
							myPairs.emplace_back(static_cast<int32_t>(i), static_cast<int32_t>(j));
					}
				}
	}

	// Hash collisions can report the same pair from different cells.
	std::sort(myPairs.begin(), myPairs.end());
	myPairs.erase(std::unique(myPairs.begin(), myPairs.end()), myPairs.end());
}

void
SoftBodySolver::solveEdges(float h2, float globalCompliance)
{
	// Per-blob compliance (blobs may override the global stiffness).
	thread_local std::vector<float> blobCompliance;
	blobCompliance.resize(myBlobs.size());
	for (size_t b = 0; b < myBlobs.size(); ++b)
		blobCompliance[b] = myBlobs[b].stiffness >= 0.0f ? complianceFromStiffness(myBlobs[b].stiffness) : globalCompliance;

	for (size_t e = 0; e < myEdges.size(); ++e)
	{
		const Edge& edge = myEdges[e];
		Vec3	d = myX[edge.a] - myX[edge.b];
		float	len = length(d);
		if (len < 1e-9f)
			continue;
		Vec3	n = d / len;
		float	wa = myW[edge.a], wb = myW[edge.b];
		float	wsum = wa + wb;
		if (wsum <= 0.0f)
			continue;

		float	C = len - edge.rest;
		float	alpha = wsum * blobCompliance[edge.blob] / h2;
		float	dl = (-C - alpha * myEdgeLambda[e]) / (wsum + alpha);
		myEdgeLambda[e] += dl;
		myX[edge.a] += n * (wa * dl);
		myX[edge.b] -= n * (wb * dl);
	}
}

void
SoftBodySolver::solveVolumes(float h2, float compliance, float pressure)
{
	for (size_t bi = 0; bi < myBlobs.size(); ++bi)
	{
		const Blob& b = myBlobs[bi];
		myScratchGrad.assign(static_cast<size_t>(b.pointCount), Vec3());

		float vol = 0.0f;
		for (int32_t t = b.triStart; t < b.triStart + b.triCount; ++t)
		{
			int32_t i0 = myTriangles[3 * t], i1 = myTriangles[3 * t + 1], i2 = myTriangles[3 * t + 2];
			const Vec3& p0 = myX[i0];
			const Vec3& p1 = myX[i1];
			const Vec3& p2 = myX[i2];
			vol += dot(p0, cross(p1, p2));
			myScratchGrad[i0 - b.pointStart] += cross(p1, p2);
			myScratchGrad[i1 - b.pointStart] += cross(p2, p0);
			myScratchGrad[i2 - b.pointStart] += cross(p0, p1);
		}
		vol /= 6.0f;

		float wsum = 0.0f;
		for (int32_t k = 0; k < b.pointCount; ++k)
		{
			myScratchGrad[k] *= 1.0f / 6.0f;
			wsum += myW[b.pointStart + k] * length2(myScratchGrad[k]);
		}
		if (wsum <= 1e-20f)
			continue;

		float C = vol - pressure * b.restVolume;
		float alpha = wsum * compliance / h2;
		float dl = (-C - alpha * myVolumeLambda[bi]) / (wsum + alpha);
		myVolumeLambda[bi] += dl;
		for (int32_t k = 0; k < b.pointCount; ++k)
			myX[b.pointStart + k] += myScratchGrad[k] * (myW[b.pointStart + k] * dl);
	}
}

void
SoftBodySolver::solveShapeMatching(float k)
{
	for (Blob& b : myBlobs)
	{
		// Particles within a blob share the same mass, so the centre of mass is the mean.
		Vec3 cm;
		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
			cm += myX[i];
		cm = cm / static_cast<float>(b.pointCount);

		Mat3 A;
		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
			A.addOuter(myX[i] - cm, myRest[i], 1.0f);

		extractRotation(A, b.quat, 10);
		Mat3 R;
		quatToMat(b.quat, R);

		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
		{
			Vec3 goal = cm + R * myRest[i];
			myX[i] += (goal - myX[i]) * k;
		}
	}
}

void
SoftBodySolver::solveContacts(float friction)
{
	const float minDist = 2.0f * myParticleRadius;
	for (const auto& pr : myPairs)
	{
		int32_t i = pr.first, j = pr.second;
		Vec3	d = myX[i] - myX[j];
		float	dist2 = length2(d);
		if (dist2 >= minDist * minDist || dist2 < 1e-18f)
			continue;
		float	dist = std::sqrt(dist2);
		Vec3	n = d / dist;
		float	wi = myW[i], wj = myW[j];
		float	wsum = wi + wj;
		if (wsum <= 0.0f)
			continue;

		float	pen = minDist - dist;
		myX[i] += n * (pen * wi / wsum);
		myX[j] -= n * (pen * wj / wsum);

		Vec3 rel = (myX[i] - myPrev[i]) - (myX[j] - myPrev[j]);
		Vec3 f = frictionCorrection(rel, n, pen, friction);
		myX[i] -= f * (wi / wsum);
		myX[j] += f * (wj / wsum);
	}
}

void
SoftBodySolver::solveContainer(const ContainerSDF& sdf, float friction)
{
	const float r = myParticleRadius;
	for (uint32_t i : myNearWall)
	{
		Vec3	g;
		float	phi = sdf.distanceAndGradient(myX[i], g);
		if (phi <= -r)
			continue;

		float pen = phi + r;
		myX[i] -= g * pen;
		myX[i] -= frictionCorrection(myX[i] - myPrev[i], g, pen, friction);
		myInContact[i] = 1;
		myContactN[i] = g;
	}
}

void
SoftBodySolver::computeNormals(std::vector<Vec3>& normals) const
{
	normals.assign(myX.size(), Vec3());
	for (size_t t = 0; t + 2 < myTriangles.size(); t += 3)
	{
		int32_t a = myTriangles[t], b = myTriangles[t + 1], c = myTriangles[t + 2];
		Vec3 n = cross(myX[b] - myX[a], myX[c] - myX[a]);	// area weighted
		normals[a] += n;
		normals[b] += n;
		normals[c] += n;
	}
	for (Vec3& n : normals)
		n = normalize(n);
}

}
