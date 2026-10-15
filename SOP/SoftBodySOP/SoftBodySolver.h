// XPBD soft body solver for sphere-like blobs trapped inside a closed container.
//
// Every blob is an icosphere whose vertices are particles. Per substep it solves:
//   - edge distance constraints        (Stiffness)
//   - one volume constraint per blob   (Volume Stiffness, Pressure)
//   - optional shape matching          (Shape Memory)
//   - particle contacts between blobs  (spatial hash, with friction)
//   - container collisions             (signed distance field, friction, restitution)
//
// No TouchDesigner dependency: the SOP wrapper feeds it plain arrays.

#pragma once

#include "ContainerSDF.h"
#include "Vec3.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace sb
{

enum class MassMode
{
	Density = 0,	// blob mass = Mass * blob volume, so bigger blobs are heavier
	PerBlob			// every blob weighs Mass
};

// Parameters used when blobs are (re)generated. Changing these requires a reset.
struct GenerationSettings
{
	float		radiusMin = 0.05f;
	float		radiusMax = 0.12f;
	float		edgeLength = 0.025f;	// target spacing between particles
	int			minLevel = 1;
	int			maxLevel = 4;
	uint32_t	seed = 1;
	MassMode	massMode = MassMode::Density;
	float		mass = 1000.0f;			// density (kg/m^3) or kg per blob, see massMode
	float		collisionRadius = 0.0f;	// particle radius, 0 = half the edge length
};

// Optional per-seed overrides, taken from point attributes of the seed input. Negative = unset.
struct SeedOverrides
{
	float		radius = -1.0f;
	float		stiffness = -1.0f;
	float		mass = -1.0f;
};

// Parameters that can change at any time while simulating.
struct SimSettings
{
	float		stiffness = 0.5f;		// 0..1, edge constraints
	float		volumeStiffness = 0.9f;	// 0..1, how strongly a blob keeps its volume
	float		pressure = 1.0f;		// rest volume multiplier, >1 inflates
	float		shapeMemory = 0.0f;		// 0..1, pull towards the rest shape (shape matching)
	float		damping = 0.5f;			// velocity damping per second
	float		friction = 0.3f;		// 0..1
	float		restitution = 0.2f;		// 0..1, bounce off the container
	Vec3		gravity{ 0.0f, -9.8f, 0.0f };
	int			substeps = 4;
	int			iterations = 8;
	float		maxTimeStep = 1.0f / 30.0f;	// clamp for dropped frames
};

struct ResetReport
{
	int			blobsCreated = 0;
	int			seedsOutside = 0;		// seed point outside the container
	int			seedsTooTight = 0;		// no room left for a blob of minimal size
};

class SoftBodySolver
{
public:
	ResetReport	reset(const std::vector<Vec3>& seeds, const std::vector<SeedOverrides>& overrides,
						const GenerationSettings& gen, const ContainerSDF* container);

	void		step(float dt, const SimSettings& sim, const ContainerSDF* container);

	void		computeNormals(std::vector<Vec3>& normals) const;

	bool		isEmpty() const { return myX.empty(); }

	const std::vector<Vec3>&	positions() const { return myX; }
	const std::vector<int32_t>&	triangles() const { return myTriangles; }
	const std::vector<int32_t>&	pointBlobIds() const { return myBlobOf; }

	int			numBlobs() const { return static_cast<int>(myBlobs.size()); }
	int			numParticles() const { return static_cast<int>(myX.size()); }
	int			numContacts() const { return static_cast<int>(myPairs.size()); }
	float		particleRadius() const { return myParticleRadius; }
	float		currentVolumeRatio() const;	// average current/rest volume, for diagnostics

private:
	struct Edge
	{
		int32_t		a, b;
		float		rest;
		int32_t		blob;
	};

	struct Blob
	{
		int32_t		pointStart = 0, pointCount = 0;
		int32_t		triStart = 0, triCount = 0;		// in triangles, not indices
		float		restVolume = 0.0f;
		float		mass = 1.0f;
		float		stiffness = -1.0f;				// < 0 = use global
		float		quat[4] = { 0, 0, 0, 1 };		// shape matching rotation (x, y, z, w)
	};

	void		buildContactPairs();
	void		solveEdges(float h2, float globalCompliance);
	void		solveVolumes(float h2, float compliance, float pressure);
	void		solveShapeMatching(float k);
	void		solveContacts(float friction);
	void		solveContainer(const ContainerSDF& sdf, float friction);
	float		blobVolume(const Blob& b) const;

	static float	complianceFromStiffness(float stiffness);

	// Particles
	std::vector<Vec3>		myX, myPrev, myV, myVPre, myRest, myContactN;
	std::vector<float>		myW;			// inverse mass
	std::vector<int32_t>	myBlobOf;
	std::vector<uint8_t>	myInContact;

	// Topology
	std::vector<int32_t>	myTriangles;
	std::vector<Edge>		myEdges;
	std::vector<float>		myEdgeLambda;
	std::vector<Blob>		myBlobs;
	std::vector<float>		myVolumeLambda;
	std::vector<Vec3>		myScratchGrad;

	// Collision
	std::vector<std::pair<int32_t, int32_t>>	myPairs;
	std::vector<uint32_t>	myCellStart, myCellEntries, myActive, myScratchHashes, myNearWall;
	std::vector<Vec3>		myBlobMin, myBlobMax;
	std::vector<size_t>		myScratchOthers;
	float					myParticleRadius = 0.01f;
};

}
