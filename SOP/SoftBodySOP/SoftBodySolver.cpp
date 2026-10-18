#include "SoftBodySolver.h"
#include "Icosphere.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <random>

namespace sb
{

namespace
{

constexpr float Pi = 3.14159265358979f;
constexpr float MaxStretch = 1.3f;	// edges may stretch at most this much beyond their rest length

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

uint64_t
cellKey(int x, int y, int z)
{
	uint64_t k = (static_cast<uint64_t>(static_cast<uint32_t>(x)) * 0x9E3779B97F4A7C15ull)
			   ^ (static_cast<uint64_t>(static_cast<uint32_t>(y)) * 0xC2B2AE3D27D4EB4Full)
			   ^ (static_cast<uint64_t>(static_cast<uint32_t>(z)) * 0x165667B19E3779F9ull);
	k ^= k >> 29;
	k *= 0xBF58476D1CE4E5B9ull;
	return k ^ (k >> 32);
}

}

// ---------------------------------------------------------------------------------------

// Largest scale s in (0, 1] so that all blobs at radius s * target, plus their collision skin,
// take at most `budget` volume. The skin keeps blobs apart, so it occupies space too.
static float
budgetScale(const std::vector<float>& targets, const std::vector<float>& unitVolumes, float skin, float budget)
{
	auto occupied = [&](float scale)
	{
		float v = 0.0f;
		for (size_t i = 0; i < targets.size(); ++i)
		{
			float r = targets[i] * scale + skin;
			v += unitVolumes[i] * r * r * r;
		}
		return v;
	};
	if (targets.empty() || occupied(1.0f) <= budget)
		return 1.0f;
	float lo = 0.0f, hi = 1.0f;
	for (int i = 0; i < 30; ++i)
	{
		float mid = 0.5f * (lo + hi);
		(occupied(mid) > budget ? hi : lo) = mid;
	}
	return lo;
}

float
SoftBodySolver::complianceFromStiffness(float stiffness)
{
	// Mass-normalised compliance in s^2: 1e-2 (jelly) .. 1e-8 (nearly rigid), 0 at stiffness 1.
	stiffness = std::clamp(stiffness, 0.0f, 1.0f);
	if (stiffness >= 0.999f)
		return 0.0f;
	return 1e-2f * std::pow(1e-6f, stiffness);
}

static std::vector<float>
radiusFractions(uint32_t seed, size_t count)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<float> uni(0.0f, 1.0f);
	std::vector<float> f(count);
	for (float& v : f)
		v = uni(rng);
	return f;
}

float
SoftBodySolver::targetRadius(const Blob& b, const std::vector<float>& fractions) const
{
	if (b.radiusOverride > 0.0f)
		return b.radiusOverride;
	float lo = std::max(std::min(myRadiusMin, myRadiusMax), 1e-4f);
	float hi = std::max(myRadiusMin, myRadiusMax);
	float u = static_cast<size_t>(b.seedIndex) < fractions.size() ? fractions[b.seedIndex] : 0.5f;
	return lo + (hi - lo) * u;
}

void
SoftBodySolver::setTargetRadii(float radiusMin, float radiusMax, uint32_t seed)
{
	if (radiusMin == myRadiusMin && radiusMax == myRadiusMax && seed == mySeed)
		return;
	if (seed != mySeed)
		myRadiusFractions = radiusFractions(seed, myRadiusFractions.size());	// one per seed point
	myRadiusMin = radiusMin;
	myRadiusMax = radiusMax;
	mySeed = seed;
	for (Blob& b : myBlobs)
		b.target = targetRadius(b, myRadiusFractions);
}

void
SoftBodySolver::updateBlobScale(Blob& b, int32_t)
{
	const float r = b.radius;
	b.restVolume = b.unitVolume * r * r * r;

	float mass = myMassMode == MassMode::Density ? myMassValue * (4.0f / 3.0f) * Pi * r * r * r : myMassValue;
	if (b.massOverride > 0.0f)
		mass = b.massOverride;
	const float w = static_cast<float>(b.pointCount) / std::max(mass, 1e-6f);
	for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
		myW[i] = w;
}

ResetReport
SoftBodySolver::reset(const std::vector<Vec3>& seeds, const std::vector<SeedOverrides>& overrides,
						const GenerationSettings& gen, const ContainerSDF* container)
{
	ResetReport report;

	myX.clear(); myPrev.clear(); myV.clear(); myVPre.clear(); myRestUnit.clear(); myContactN.clear();
	myW.clear(); myBlobOf.clear(); myInContact.clear();
	myTriangles.clear(); myEdges.clear(); myEdgeLambda.clear(); myBends.clear(); myBendLambda.clear();
	myBlobs.clear(); myVolumeLambda.clear(); myPairs.clear();

	const float edgeLength = std::max(gen.edgeLength, 1e-4f);
	myParticleRadius = gen.collisionRadius > 0.0f ? gen.collisionRadius : edgeLength * 0.5f;
	myMassMode = gen.massMode;
	myMassValue = gen.mass;
	const float pr = myParticleRadius;
	const bool hasContainer = container && container->isValid();

	// Target radii: a random fraction of the radius range per seed (or the seed's radius attribute).
	const size_t n = seeds.size();
	myRadiusMin = gen.radiusMin;
	myRadiusMax = gen.radiusMax;
	mySeed = gen.seed;
	myRadiusFractions = radiusFractions(gen.seed, n);

	std::vector<Blob> candidates(n);
	std::vector<uint8_t> inside(n, 1);
	size_t numInside = 0;
	for (size_t i = 0; i < n; ++i)
	{
		Blob& b = candidates[i];
		b.seedIndex = static_cast<int32_t>(i);
		if (i < overrides.size())
		{
			b.radiusOverride = overrides[i].radius;
			b.massOverride = overrides[i].mass;
			if (overrides[i].stiffness >= 0.0f)
				b.stiffness = std::clamp(overrides[i].stiffness, 0.0f, 1.0f);
		}
		b.target = targetRadius(b, myRadiusFractions);
		if (hasContainer && container->distance(seeds[i]) >= 0.0f)
		{
			inside[i] = 0;
			++report.seedsOutside;
		}
		else
			++numInside;
	}

	// Mesh resolution is planned for the size a blob will actually reach: its target, scaled down
	// the same way the Fill budget will scale it during the simulation. Without a container, fall
	// back to a share of nothing in particular: the target itself.
	float planScale = 1.0f;
	if (hasContainer && numInside > 0)
	{
		std::vector<float> targets, unitVolumes;
		for (size_t i = 0; i < n; ++i)
		{
			if (!inside[i])
				continue;
			targets.push_back(candidates[i].target);
			unitVolumes.push_back(4.0f / 3.0f * Pi);
		}
		const float budget = std::clamp(gen.fill, 0.01f, 0.99f) * container->insideVolume() / std::max(gen.pressure, 0.01f);
		planScale = budgetScale(targets, unitVolumes, pr, budget);
	}

	// Spawn radius: the target, shrunk so blobs neither overlap each other nor the walls.
	// Blobs that don't fit start small and grow into place, squeezing their neighbours.
	const float smallest = std::max(1.5f * pr, 1e-4f);
	std::vector<float> spawn(n, 0.0f);
	for (size_t i = 0; i < n; ++i)
	{
		if (!inside[i])
			continue;
		const float finalRadius = candidates[i].target * planScale;
		float r = finalRadius;
		if (hasContainer)
			r = std::min(r, -container->distance(seeds[i]) - pr);
		for (size_t j = 0; j < n; ++j)
		{
			if (j == i || !inside[j])
				continue;
			float d = length(seeds[i] - seeds[j]);
			if (j < i)
				r = std::min(r, d - spawn[j] - 2.0f * pr);
			else
				r = std::min(r, 0.5f * d - pr);
		}
		if (r < finalRadius * 0.999f)
			++report.blobsStartedSmall;
		spawn[i] = std::max(r, smallest);
	}

	// Build the blobs.
	for (size_t i = 0; i < n; ++i)
	{
		if (!inside[i])
			continue;

		Blob blob = candidates[i];
		const float	planRadius = std::max(blob.target * planScale, spawn[i]);
		const int	frequency = geodesicFrequencyFor(planRadius, edgeLength,
								geodesicFrequencyAtLeast(gen.minPoints), geodesicFrequencyAtMost(gen.maxPoints));
		const Icosphere& ico = getUnitGeodesicSphere(frequency);
		const int32_t blobId = static_cast<int32_t>(myBlobs.size());

		blob.pointStart = static_cast<int32_t>(myX.size());
		blob.pointCount = static_cast<int32_t>(ico.points.size());
		blob.triStart = static_cast<int32_t>(myTriangles.size() / 3);
		blob.triCount = static_cast<int32_t>(ico.triangles.size() / 3);
		blob.radius = spawn[i];

		for (const Vec3& u : ico.points)
		{
			myX.push_back(seeds[i] + u * blob.radius);
			myRestUnit.push_back(u);
			myW.push_back(1.0f);
			myBlobOf.push_back(blobId);
		}
		for (int32_t idx : ico.triangles)
			myTriangles.push_back(idx + blob.pointStart);

		// Each edge of a closed, consistently wound mesh appears once in each direction:
		// keeping only a < b gives every edge exactly once.
		const size_t edgeStart = myEdges.size();
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
					edge.restUnit = length(ico.points[a] - ico.points[b]);
					edge.blob = blobId;
					blob.maxUnitEdge = std::max(blob.maxUnitEdge, edge.restUnit);
					myEdges.push_back(edge);
				}
			}
		}

		// Group the edges by a greedy colouring: edges of one colour share no particle. In mesh order
		// every edge would wait for the previous one's result, grouped they can be solved overlapped.
		{
			std::vector<uint64_t> used(ico.points.size(), 0);
			std::vector<uint8_t> colour(myEdges.size() - edgeStart);
			for (size_t e = 0; e < colour.size(); ++e)
			{
				const Edge& edge = myEdges[edgeStart + e];
				uint64_t& ua = used[edge.a - blob.pointStart];
				uint64_t& ub = used[edge.b - blob.pointStart];
				const uint64_t taken = ua | ub;
				uint8_t c = 0;
				while (c < 63 && ((taken >> c) & 1))
					++c;
				colour[e] = c;
				ua |= 1ull << c;
				ub |= 1ull << c;
			}
			std::vector<Edge> grouped;
			grouped.reserve(colour.size());
			for (uint8_t c = 0, last = *std::max_element(colour.begin(), colour.end()); c <= last; ++c)
				for (size_t e = 0; e < colour.size(); ++e)
					if (colour[e] == c)
						grouped.push_back(myEdges[edgeStart + e]);
			std::copy(grouped.begin(), grouped.end(), myEdges.begin() + edgeStart);
		}

		// Bending: for every edge shared by two triangles, a soft distance constraint between the two
		// far corners. It resists sharp folds (wrinkles) but hardly changes when a region flattens.
		{
			std::map<std::pair<int32_t, int32_t>, int32_t> firstOpposite;
			for (size_t t = 0; t < ico.triangles.size(); t += 3)
			{
				for (int e = 0; e < 3; ++e)
				{
					int32_t a = ico.triangles[t + e], b = ico.triangles[t + (e + 1) % 3], c = ico.triangles[t + (e + 2) % 3];
					auto key = std::make_pair(std::min(a, b), std::max(a, b));
					auto it = firstOpposite.find(key);
					if (it == firstOpposite.end())
					{
						firstOpposite[key] = c;
						continue;
					}
					Edge bend;
					bend.a = it->second + blob.pointStart;
					bend.b = c + blob.pointStart;
					bend.restUnit = length(ico.points[it->second] - ico.points[c]);
					bend.blob = blobId;
					myBends.push_back(bend);
				}
			}
		}

		float unitVolume = 0.0f;
		for (size_t t = 0; t < ico.triangles.size(); t += 3)
			unitVolume += dot(ico.points[ico.triangles[t]], cross(ico.points[ico.triangles[t + 1]], ico.points[ico.triangles[t + 2]]));
		blob.unitVolume = unitVolume / 6.0f;

		myBlobs.push_back(blob);
		updateBlobScale(myBlobs.back(), blobId);
		++report.blobsCreated;
	}

	const size_t np = myX.size();
	myPrev = myX;
	myV.assign(np, Vec3());
	myVPre.assign(np, Vec3());
	myContactN.assign(np, Vec3());
	myInContact.assign(np, 0);
	myEdgeLambda.assign(myEdges.size(), 0.0f);
	myBendLambda.assign(myBends.size(), 0.0f);
	myVolumeLambda.assign(myBlobs.size(), 0.0f);
	myBlobRadius.assign(myBlobs.size(), myParticleRadius);
	myMaxBlobRadius = myParticleRadius;
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

void
SoftBodySolver::updateCollisionRadii()
{
	// A particle of blob A slips through a triangle of blob B if the gap at the triangle centre
	// (edge / sqrt 3) is wider than rA + rB. Edges are strain limited to MaxStretch x rest length,
	// so rB = 0.58 * longest possible edge - base keeps rA + rB >= edge / sqrt 3 for any blob A.
	const size_t nb = myBlobs.size();
	myBlobRadius.resize(nb);
	myMaxBlobRadius = myParticleRadius;
	for (size_t b = 0; b < nb; ++b)
	{
		const float longest = MaxStretch * myBlobs[b].maxUnitEdge * myBlobs[b].radius;
		myBlobRadius[b] = std::max(myParticleRadius, 0.58f * longest - myParticleRadius);
		myMaxBlobRadius = std::max(myMaxBlobRadius, myBlobRadius[b]);
	}
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

bool
SoftBodySolver::growBlobs(float growK, float targetScale)
{
	// Each blob approaches its target radius in log space: the relative change per substep is
	// growK * ln(target / radius), capped, so a tiny blob that has to grow 6x doesn't race ahead.
	// The particles are scaled with the rest shape around the blob's centre (before the velocity
	// prediction), so growing moves the surface smoothly instead of making the constraints yank
	// it outwards, which used to show up as a ripple on fast-growing small blobs.
	constexpr float MaxStep = 0.02f;	// at most 2% size change per substep
	bool changed = false;
	for (size_t bi = 0; bi < myBlobs.size(); ++bi)
	{
		Blob& b = myBlobs[bi];
		const float target = std::max(b.target * targetScale, 1e-4f);
		if (b.radius == target)
			continue;

		float step = std::clamp(growK * std::log(target / b.radius), -MaxStep, MaxStep);
		float newRadius = b.radius * std::exp(step);
		if ((target - newRadius) * (target - b.radius) <= 0.0f || std::abs(target - newRadius) < 1e-5f * target)
			newRadius = target;
		const float f = newRadius / b.radius;

		Vec3 c;
		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
			c += myX[i];
		c = c / static_cast<float>(b.pointCount);
		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
			myX[i] = c + (myX[i] - c) * f;

		b.radius = newRadius;
		updateBlobScale(b, static_cast<int32_t>(bi));
		changed = true;
	}
	return changed;
}

float
SoftBodySolver::growRatio() const
{
	if (myBlobs.empty())
		return 1.0f;
	float sum = 0.0f;
	for (const Blob& b : myBlobs)
		sum += b.target > 0.0f ? b.radius / (b.target * myTargetScale) : 1.0f;
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
	// Bending stays out of the stiffest part of the range: even 1 allows gentle curvature changes.
	const float bendCompliance = sim.bending > 0.0f ? complianceFromStiffness(0.8f * std::clamp(sim.bending, 0.0f, 1.0f)) : -1.0f;
	// Volume Stiffness 0 should still give a squashy blob, not a deflated bag: use the upper part of the range.
	const float volCompliance = complianceFromStiffness(0.4f + 0.6f * std::clamp(sim.volumeStiffness, 0.0f, 1.0f));
	const float shapeK = std::clamp(sim.shapeMemory, 0.0f, 1.0f);
	const float shapeKIter = shapeK > 0.0f ? 1.0f - std::pow(1.0f - std::min(shapeK, 0.999f), 1.0f / static_cast<float>(iterations)) : 0.0f;
	const float friction = std::clamp(sim.friction, 0.0f, 1.0f);
	const float restitution = std::clamp(sim.restitution, 0.0f, 1.0f);
	const float dampFactor = std::exp(-std::max(sim.damping, 0.0f) * h);
	const bool	useContainer = container && container->isValid();

	// Grow (or shrink) the blobs' rest size towards their target radius. The targets are scaled
	// down uniformly so that all blobs together, including their collision skin, take at most Fill of
	// the container volume. Asking for more than fits is what makes blobs overlap: the constraints
	// can't all be met. Near full packing the volume constraints flatten the blobs against each other.
	float targetScale = 1.0f;
	if (useContainer && !myBlobs.empty())
	{
		thread_local std::vector<float> targets, unitVolumes;
		targets.clear();
		unitVolumes.clear();
		for (const Blob& b : myBlobs)
		{
			targets.push_back(b.target);
			unitVolumes.push_back(b.unitVolume);
		}
		const float budget = std::clamp(sim.fill, 0.01f, 0.99f) * container->insideVolume() / std::max(sim.pressure, 0.01f);
		targetScale = budgetScale(targets, unitVolumes, myParticleRadius, budget);
	}
	myTargetScale = targetScale;

	// Growth happens a little every substep (see growBlobs) so small blobs don't pop.
	const float growK = 1.0f - std::exp(-std::max(sim.growSpeed, 0.0f) * h);

	updateCollisionRadii();

	for (int s = 0; s < substeps; ++s)
	{
		if (growK > 0.0f && growBlobs(growK, targetScale))
			updateCollisionRadii();

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
		std::fill(myBendLambda.begin(), myBendLambda.end(), 0.0f);
		std::fill(myVolumeLambda.begin(), myVolumeLambda.end(), 0.0f);

		for (int it = 0; it < iterations; ++it)
		{
			solveEdges(h2, edgeCompliance);
			if (bendCompliance >= 0.0f)
				solveBends(h2, bendCompliance);
			solveVolumes(h2, volCompliance, std::max(sim.pressure, 0.01f));
			if (shapeKIter > 0.0f)
				solveShapeMatching(shapeKIter);
			solveContacts(friction);
			if (useContainer)
				solveContainer(*container, friction);
		}

		// Collisions get the last word: when blobs are packed tighter than their rest volume allows,
		// the volume gives way (and growth stops) instead of blobs tunnelling into each other.
		for (int it = 0; it < 4; ++it)
		{
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
	const float	searchDist = 2.0f * myMaxBlobRadius * 1.5f;	// margin: pairs stay valid for the whole substep
	const float	searchDist2 = searchDist * searchDist;
	const float	inv = 1.0f / searchDist;						// cell size = search distance

	// Counting sort of particles into a hash table.
	uint32_t tableSize = 64;
	while (tableSize < 2 * np)
		tableSize <<= 1;
	myCellStart.assign(tableSize + 1, 0);
	myCellEntries.resize(np);

	auto hashOf = [&](int cx, int cy, int cz)
	{
		return static_cast<uint32_t>(cellKey(cx, cy, cz)) & (tableSize - 1);
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

	// Copies in bucket order, so the query below reads contiguous memory.
	const size_t numActive = myActive.size();
	myCellX.resize(numActive);
	myCellBlob.resize(numActive);
	for (size_t e = 0; e < numActive; ++e)
	{
		myCellX[e] = myX[myCellEntries[e]];
		myCellBlob[e] = myBlobOf[myCellEntries[e]];
	}

	// Query the 27 neighbouring cells. Cells that hash to the same bucket are visited once,
	// so every pair is found exactly once.
	for (uint32_t i : myActive)
	{
		int cx, cy, cz;
		cellOf(myX[i], cx, cy, cz);
		const int32_t bi = myBlobOf[i];
		const Vec3 xi = myX[i];

		uint32_t buckets[27];
		int numBuckets = 0;
		for (int dz = -1; dz <= 1; ++dz)
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					const uint32_t hsh = hashOf(cx + dx, cy + dy, cz + dz);
					if (std::find(buckets, buckets + numBuckets, hsh) == buckets + numBuckets)
						buckets[numBuckets++] = hsh;
				}

		for (int c = 0; c < numBuckets; ++c)
		{
			const uint32_t hsh = buckets[c];
			for (uint32_t e = myCellStart[hsh]; e < myCellStart[hsh + 1]; ++e)
			{
				if (myCellBlob[e] == bi || length2(xi - myCellX[e]) >= searchDist2)
					continue;
				const uint32_t j = myCellEntries[e];
				if (j > i)
					myPairs.emplace_back(static_cast<int32_t>(i), static_cast<int32_t>(j));
			}
		}
	}
}

void
SoftBodySolver::solveBends(float h2, float compliance)
{
	Vec3*			x = myX.data();
	const float*	w = myW.data();
	float*			lambda = myBendLambda.data();
	const float		alphaScale = compliance / h2;
	for (size_t e = 0; e < myBends.size(); ++e)
	{
		const Edge& bend = myBends[e];
		Vec3	d = x[bend.a] - x[bend.b];
		float	len2 = length2(d);
		if (len2 < 1e-18f)
			continue;
		float	len = std::sqrt(len2);
		float	wa = w[bend.a], wb = w[bend.b];
		float	wsum = wa + wb;
		if (wsum <= 0.0f)
			continue;

		const float rest = bend.restUnit * myBlobs[bend.blob].radius;
		float	alpha = wsum * alphaScale;
		float	dl = (-(len - rest) - alpha * lambda[e]) / (wsum + alpha);
		lambda[e] += dl;
		const Vec3 c = d * (dl / len);
		x[bend.a] += c * wa;
		x[bend.b] -= c * wb;
	}
}

void
SoftBodySolver::solveEdges(float h2, float globalCompliance)
{
	// Per-blob compliance (blobs may override the global stiffness).
	thread_local std::vector<float> blobCompliance;
	blobCompliance.resize(myBlobs.size());
	thread_local std::vector<float> blobRadius;
	blobCompliance.resize(myBlobs.size());
	blobRadius.resize(myBlobs.size());
	for (size_t b = 0; b < myBlobs.size(); ++b)
	{
		const float compliance = myBlobs[b].stiffness >= 0.0f ? complianceFromStiffness(myBlobs[b].stiffness) : globalCompliance;
		blobCompliance[b] = compliance / h2;
		blobRadius[b] = myBlobs[b].radius;
	}

	Vec3*			x = myX.data();
	const float*	w = myW.data();
	float*			lambda = myEdgeLambda.data();
	const size_t	numEdges = myEdges.size();
	for (size_t e = 0; e < numEdges; ++e)
	{
		const Edge& edge = myEdges[e];
		Vec3	d = x[edge.a] - x[edge.b];
		float	len2 = length2(d);
		if (len2 < 1e-18f)
			continue;
		float	len = std::sqrt(len2);
		float	wa = w[edge.a], wb = w[edge.b];
		float	wsum = wa + wb;
		if (wsum <= 0.0f)
			continue;

		const float rest = edge.restUnit * blobRadius[edge.blob];
		float	alpha = wsum * blobCompliance[edge.blob];
		float	dl = (-(len - rest) - alpha * lambda[e]) / (wsum + alpha);
		lambda[e] += dl;

		// Strain limit: soft edges may not stretch beyond MaxStretch, which keeps the gaps between
		// particles small enough for other blobs not to slip through. The correction acts along d,
		// so the corrected length is len + wsum * dl.
		const float newLen = len + wsum * dl, maxLen = MaxStretch * rest;
		if (newLen > maxLen)
			dl -= (newLen - maxLen) / wsum;

		const Vec3 c = d * (dl / len);
		x[edge.a] += c * wa;
		x[edge.b] -= c * wb;
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

		myBlobs[bi].volumeRatio = b.restVolume > 0.0f ? vol / (pressure * b.restVolume) : 1.0f;
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
			A.addOuter(myX[i] - cm, myRestUnit[i], b.radius);

		extractRotation(A, b.quat, 10);
		Mat3 R;
		quatToMat(b.quat, R);

		for (int32_t i = b.pointStart; i < b.pointStart + b.pointCount; ++i)
		{
			Vec3 goal = cm + R * (myRestUnit[i] * b.radius);
			myX[i] += (goal - myX[i]) * k;
		}
	}
}

void
SoftBodySolver::solveContacts(float friction)
{
	for (const auto& pr : myPairs)
	{
		int32_t i = pr.first, j = pr.second;
		const float minDist = myBlobRadius[myBlobOf[i]] + myBlobRadius[myBlobOf[j]];
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
	const float r = myParticleRadius;	// the wall is a continuous surface, stretching doesn't matter here
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
