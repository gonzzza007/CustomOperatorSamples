// XPBD soft body solver for blobs trapped inside a closed container.
//
// A blob is either
//   - a sphere: a hollow icosphere whose vertices are particles, kept inflated by one volume constraint, or
//   - a cube: a solid lattice of particles split into tetrahedra (see CubeLattice), one volume constraint per tet.
// Per substep it solves:
//   - edge distance constraints        (Stiffness)
//   - sphere: bending between neighbouring triangles (Bending)
//   - volume constraints               (Volume Stiffness, Pressure)
//   - optional shape matching          (Shape Memory)
//   - particle contacts between blobs  (surface particles only, spatial hash, with friction)
//   - container collisions             (surface particles only, signed distance field, friction, restitution)
//
// Macklin, Mueller, Chentanez 2016, "XPBD: Position-Based Simulation of Compliant Constrained Dynamics";
// the tet volume constraint as in Mueller's "Ten Minute Physics" soft body tutorial.
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

enum class BlobShape
{
	Sphere = 0,
	Cube			// radius = half the edge length
};

enum class MassMode
{
	Density = 0,	// blob mass = Mass * blob volume, so bigger blobs are heavier
	PerBlob			// every blob weighs Mass
};

// Parameters used when blobs are (re)generated. Changing these requires a reset,
// except the radius range and seed: they only set the target radii the blobs grow towards.
struct GenerationSettings
{
	BlobShape	shape = BlobShape::Sphere;
	float		radiusMin = 0.05f;
	float		radiusMax = 0.12f;
	uint32_t	seed = 1;
	bool		randomOrientation = true;	// cubes only
	float		edgeLength = 0.025f;	// target spacing between particles
	int			minPoints = 42;			// points per blob are chosen from its final size,
	int			maxPoints = 2562;		// clamped to this range
	float		fill = 0.85f;			// used to predict the final blob sizes (see SimSettings::fill)
	float		pressure = 1.0f;
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
	float		bending = 0.5f;			// 0..1, spheres: resistance to folds and wrinkles (0 = off)
	float		volumeStiffness = 0.9f;	// 0..1, how strongly a blob keeps its volume
	float		pressure = 1.0f;		// rest volume multiplier, >1 inflates
	float		growSpeed = 2.0f;		// 1/s, how fast blobs approach their target radius (0 = frozen)
	float		fill = 0.85f;			// 0..0.99, fraction of the container the blobs may occupy together
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
	int			seedsOutside = 0;		// seed point outside the container, skipped
	int			blobsStartedSmall = 0;	// spawned smaller than their target, they grow into place
};

class SoftBodySolver
{
public:
	ResetReport	reset(const std::vector<Vec3>& seeds, const std::vector<SeedOverrides>& overrides,
						const GenerationSettings& gen, const ContainerSDF* container);

	// Sets the radii the blobs grow towards. Cheap to call every frame; does not reset anything.
	void		setTargetRadii(float radiusMin, float radiusMax, uint32_t seed);

	void		step(float dt, const SimSettings& sim, const ContainerSDF* container);

	// Positions and normals of the surface vertices (see surfaceTriangles). A sphere has one vertex per particle;
	// every side of a cube has its own vertices, so its edges stay hard. Normals are angle weighted, ignore
	// triangles the simulation has folded over and get smoothPasses rounds of averaging with their neighbours.
	// With offset, the surface is pushed out by the collision radius along the normals (cube edges and corners
	// diagonally, to where the offset sides meet), so touching blobs look touching.
	void		computeSurface(std::vector<Vec3>& positions, std::vector<Vec3>& normals, bool offset,
								int smoothPasses) const;

	bool		isEmpty() const { return myX.empty(); }

	const std::vector<int32_t>&	surfaceTriangles() const { return mySurfaceTriangles; }	// counter-clockwise seen from outside
	const std::vector<int32_t>&	surfaceVertexBlobIds() const { return mySurfaceVertexBlob; }

	int			numBlobs() const { return static_cast<int>(myBlobs.size()); }
	int			numParticles() const { return static_cast<int>(myX.size()); }
	int			numTets() const { return static_cast<int>(myTets.size()); }
	int			numContacts() const { return static_cast<int>(myPairs.size()); }
	float		particleRadius() const { return myParticleRadius; }
	float		currentVolumeRatio() const;	// average current/rest volume, for diagnostics
	float		growRatio() const;			// average current/target radius, 1 = fully grown
	float		targetScale() const { return myTargetScale; }	// < 1 when the targets don't fit

private:
	struct Edge
	{
		int32_t		a, b;
		float		restUnit;		// rest length at radius 1, scaled by the blob radius
		int32_t		blob;
	};

	struct Tet
	{
		int32_t		p[4];
		float		restUnit;		// rest volume at radius 1, scaled by radius^3
		int32_t		blob;
	};

	struct Blob
	{
		int32_t		pointStart = 0, pointCount = 0;
		int32_t		surfaceStart = 0, surfaceCount = 0;	// in mySurfacePoints
		int32_t		triStart = 0, triCount = 0;			// in surface triangles, not indices
		int32_t		seedIndex = 0;
		float		radius = 1.0f;					// current rest radius
		float		target = 1.0f;					// radius it grows towards
		float		radiusOverride = -1.0f;			// from the seed's radius attribute
		float		unitVolume = 0.0f;				// rest volume of the mesh at radius 1
		float		maxUnitEdge = 0.0f;				// longest surface edge at radius 1
		float		restVolume = 0.0f;
		float		volumeRatio = 1.0f;				// current / rest volume at the end of the last step
		float		massOverride = -1.0f;
		float		stiffness = -1.0f;				// < 0 = use global
		float		quat[4] = { 0, 0, 0, 1 };		// shape matching rotation (x, y, z, w)
	};

	void		addSphere(Blob& blob, const Vec3& centre, int frequency);
	void		addCube(Blob& blob, const Vec3& centre, int resolution);
	void		buildContactPairs();
	void		solveEdges(float h2, float globalCompliance);
	void		solveBends(float h2, float compliance);
	void		solveVolumes(float h2, float compliance, float pressure);
	void		solveTets(float h2, float compliance, float pressure);
	void		solveShapeMatching(float k);
	void		solveContacts(float friction);
	void		solveContainer(const ContainerSDF& sdf, float friction);
	float		blobVolume(const Blob& b) const;
	float		shapeUnitVolume() const;
	void		updateBlobScale(Blob& b);
	void		updateCollisionRadii();
	bool		growBlobs(float growK, float targetScale);
	float		targetRadius(const Blob& b, const std::vector<float>& fractions) const;

	static float	complianceFromStiffness(float stiffness);

	BlobShape				myShape = BlobShape::Sphere;

	// Particles
	std::vector<Vec3>		myX, myPrev, myV, myVPre, myRestUnit, myContactN;
	std::vector<float>		myW;			// inverse mass
	std::vector<int32_t>	myBlobOf;
	std::vector<uint8_t>	myInContact;

	// Topology
	std::vector<Edge>		myEdges;
	std::vector<float>		myEdgeLambda;
	std::vector<Edge>		myBends;		// spheres: between the far corners of two triangles sharing an edge
	std::vector<float>		myBendLambda;
	std::vector<Tet>		myTets;			// cubes
	std::vector<float>		myTetLambda;
	std::vector<Blob>		myBlobs;
	std::vector<float>		myVolumeLambda;	// spheres: one per blob
	std::vector<Vec3>		myScratchGrad;

	// Surface
	std::vector<int32_t>	mySurfacePoints;		// particles on the blobs' surfaces, by blob (spheres: all)
	std::vector<int32_t>	mySurfaceVertex;		// particle of each surface vertex
	std::vector<int32_t>	mySurfaceVertexBlob;
	std::vector<int32_t>	mySurfaceTriangles;		// surface vertex indices
	mutable std::vector<Vec3>	myScratchOffset;

	// Collision
	std::vector<std::pair<int32_t, int32_t>>	myPairs;
	std::vector<uint32_t>	myCellStart, myCellEntries, myActive, myScratchHashes, myNearWall;
	std::vector<Vec3>		myBlobMin, myBlobMax;
	std::vector<Vec3>		myCellX;					// positions of myCellEntries, in bucket order
	std::vector<int32_t>	myCellBlob;					// blob ids of myCellEntries, in bucket order
	std::vector<size_t>		myScratchOthers;
	float					myParticleRadius = 0.01f;	// from the parameters
	std::vector<float>		myBlobRadius;				// per blob collision radius, follows stretched edges
	float					myMaxBlobRadius = 0.01f;

	// Target radii
	MassMode				myMassMode = MassMode::Density;
	float					myMassValue = 1000.0f;
	std::vector<float>		myRadiusFractions;			// per seed, random 0..1
	float					myRadiusMin = 0.0f, myRadiusMax = 0.0f;
	float					myTargetScale = 1.0f;		// targets scaled down to fit the Fill budget
	uint32_t				mySeed = 0;
};

}
