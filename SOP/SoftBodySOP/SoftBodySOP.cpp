/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "SoftBodySOP.h"

#include <array>
#include <chrono>
#include <cstring>

using namespace TD;

extern "C"
{

DLLEXPORT
void
FillSOPPluginInfo(SOP_PluginInfo* info)
{
	// Check to make sure the running TD version supports our API version.
	if (!info->setAPIVersion(SOPCPlusPlusAPIVersion))
		return;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString("Softbody");
	customInfo.opLabel->setString("Soft Body");
	customInfo.opIcon->setString("SFT");
	customInfo.authorName->setString("Valentin");
	customInfo.authorEmail->setString("");

	// Input 0: container mesh, input 1: seed points
	customInfo.minInputs = 2;
	customInfo.maxInputs = 2;

	customInfo.majorVersion = 0;
	customInfo.minorVersion = 1;
}

DLLEXPORT
SOP_CPlusPlusBase*
CreateSOPInstance(const OP_NodeInfo* info)
{
	return new SoftBodySOP(info);
}

DLLEXPORT
void
DestroySOPInstance(SOP_CPlusPlusBase* instance)
{
	delete (SoftBodySOP*)instance;
}

};

namespace
{

// FNV-1a, used to detect when input geometry or generation parameters actually change.
struct Hasher
{
	uint64_t h = 1469598103934665603ull;

	void bytes(const void* data, size_t size)
	{
		const unsigned char* p = static_cast<const unsigned char*>(data);
		for (size_t i = 0; i < size; ++i)
		{
			h ^= p[i];
			h *= 1099511628211ull;
		}
	}
	template <typename T> void value(const T& v) { bytes(&v, sizeof(T)); }
};

const SOP_CustomAttribData*
findFloatAttrib(const OP_SOPInput* sop, const char* name)
{
	const SOP_CustomAttribData* a = sop->getCustomAttribute(name);
	if (a && a->attribType == AttribType::Float && a->floatData && a->numComponents >= 1)
		return a;
	return nullptr;
}

}

SoftBodySOP::SoftBodySOP(const OP_NodeInfo*)
{
}

SoftBodySOP::~SoftBodySOP()
{
}

void
SoftBodySOP::getGeneralInfo(SOP_GeneralInfo* ginfo, const OP_Inputs*, void*)
{
	// A simulation has to advance every frame.
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->directToGPU = false;
}

bool
SoftBodySOP::updateContainer(const OP_SOPInput* sop, int resolution)
{
	const int32_t numPoints = sop->getNumPoints();
	const Position* pos = sop->getPointPositions();

	Hasher hash;
	hash.value(resolution);
	hash.value(numPoints);
	hash.bytes(pos, sizeof(Position) * static_cast<size_t>(numPoints));

	std::vector<int32_t> tris;
	for (int32_t i = 0; i < sop->getNumPrimitives(); ++i)
	{
		const SOP_PrimitiveInfo& prim = sop->getPrimitive(i);
		if (prim.type != PrimitiveType::Polygon || prim.numVertices < 3)
			continue;
		// Fan triangulation; winding does not matter for the SDF (inside test is ray parity).
		for (int32_t v = 1; v + 1 < prim.numVertices; ++v)
			tris.insert(tris.end(), { prim.pointIndices[0], prim.pointIndices[v], prim.pointIndices[v + 1] });
	}
	hash.bytes(tris.data(), tris.size() * sizeof(int32_t));

	if (hash.h == myContainerHash && myContainer.isValid())
		return false;
	myContainerHash = hash.h;

	std::vector<sb::Vec3> pts(static_cast<size_t>(numPoints));
	for (int32_t i = 0; i < numPoints; ++i)
		pts[i] = sb::Vec3(pos[i].x, pos[i].y, pos[i].z);

	auto t0 = std::chrono::steady_clock::now();
	myContainer.build(pts, tris, resolution);
	mySdfMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
	return true;
}

uint64_t
SoftBodySOP::readSeeds(const OP_SOPInput* sop)
{
	const int32_t numPoints = sop->getNumPoints();
	const Position* pos = sop->getPointPositions();

	const SOP_CustomAttribData* radius = findFloatAttrib(sop, "radius");
	if (!radius)
		radius = findFloatAttrib(sop, "pscale");
	const SOP_CustomAttribData* stiffness = findFloatAttrib(sop, "stiffness");
	const SOP_CustomAttribData* mass = findFloatAttrib(sop, "mass");

	mySeeds.resize(static_cast<size_t>(numPoints));
	myOverrides.assign(static_cast<size_t>(numPoints), sb::SeedOverrides());
	for (int32_t i = 0; i < numPoints; ++i)
	{
		mySeeds[i] = sb::Vec3(pos[i].x, pos[i].y, pos[i].z);
		if (radius)
			myOverrides[i].radius = radius->floatData[static_cast<size_t>(i) * radius->numComponents];
		if (stiffness)
			myOverrides[i].stiffness = stiffness->floatData[static_cast<size_t>(i) * stiffness->numComponents];
		if (mass)
			myOverrides[i].mass = mass->floatData[static_cast<size_t>(i) * mass->numComponents];
	}

	Hasher hash;
	hash.bytes(mySeeds.data(), mySeeds.size() * sizeof(sb::Vec3));
	hash.bytes(myOverrides.data(), myOverrides.size() * sizeof(sb::SeedOverrides));
	return hash.h;
}

void
SoftBodySOP::execute(SOP_Output* output, const OP_Inputs* inputs, void*)
{
	const OP_SOPInput* containerSop = inputs->getInputSOP(0);
	const OP_SOPInput* seedSop = inputs->getInputSOP(1);
	if (!containerSop || !seedSop)
	{
		myErrorString = "Connect a closed container mesh to input 1 and seed points to input 2.";
		return;
	}

	// Container: rebuilt only when its geometry or resolution changes.
	updateContainer(containerSop, Parameters::evalSdfresolution(inputs));
	if (!myContainer.isValid())
	{
		myErrorString = "Container input has no polygons.";
		return;
	}

	// Regenerate blobs when seeds or generation parameters change, or on Reset.
	const sb::GenerationSettings gen = Parameters::evalGenerationSettings(inputs);
	Hasher genHash;
	// Radius range and seed are not part of the hash: they only change the target radii,
	// and the blobs grow or shrink towards them without a reset.
	genHash.value(gen.shape);
	genHash.value(gen.randomOrientation);
	genHash.value(gen.edgeLength);
	genHash.value(gen.minPoints);
	genHash.value(gen.maxPoints);
	genHash.value(gen.massMode);
	genHash.value(gen.mass);
	genHash.value(gen.collisionRadius);

	const uint64_t seedHash = readSeeds(seedSop);
	if (myResetRequested || seedHash != mySeedHash || genHash.h != myGenHash)
	{
		myLastReport = mySolver.reset(mySeeds, myOverrides, gen, &myContainer);
		mySeedHash = seedHash;
		myGenHash = genHash.h;
		myResetRequested = false;
	}
	else
		mySolver.setTargetRadii(gen.radiusMin, gen.radiusMax, gen.seed);

	if (Parameters::evalSimulate(inputs))
	{
		const OP_TimeInfo* time = inputs->getTimeInfo();
		float dt = (time && time->rate > 0.0) ? static_cast<float>(time->deltaFrames / time->rate) : 0.0f;
		if (dt > 0.0f)
		{
			auto t0 = std::chrono::steady_clock::now();
			mySolver.step(dt, Parameters::evalSimSettings(inputs), &myContainer);
			myStepMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
		}
	}

	if (myLastReport.seedsOutside > 0)
		myWarningString = std::to_string(myLastReport.seedsOutside) + " seed(s) outside the container were skipped.";
	if (mySolver.isEmpty())
		myWarningString = "No blobs: check that the seed points are inside the container.";

	myOffsetSurface = Parameters::evalSurfaceoffset(inputs);
	myNormalSmoothing = Parameters::evalNormalsmoothing(inputs);
	writeOutput(output);
}

void
SoftBodySOP::writeOutput(SOP_Output* output)
{
	if (mySolver.isEmpty())
		return;

	// The particles are spheres of the collision radius, so the blobs actually touch that far outside
	// the simulated points. With Surface Offset the drawn surface is pushed out to there, so touching
	// blobs look touching instead of leaving a gap of twice the collision radius.
	mySolver.computeSurface(myPositions, myNormals, myOffsetSurface, myNormalSmoothing);
	const int32_t numPoints = static_cast<int32_t>(myPositions.size());

	std::vector<Position> pos(myPositions.size());
	for (size_t i = 0; i < myPositions.size(); ++i)
		pos[i] = Position(myPositions[i].x, myPositions[i].y, myPositions[i].z);
	output->addPoints(pos.data(), numPoints);

	std::vector<Vector> normals(myNormals.size());
	for (size_t i = 0; i < myNormals.size(); ++i)
		normals[i] = Vector(myNormals[i].x, myNormals[i].y, myNormals[i].z);
	output->setNormals(normals.data(), numPoints, 0);

	// The solver's triangles wind counter-clockwise seen from outside (the volume constraint relies
	// on that), but TouchDesigner treats clockwise as the front face. Emit them reversed so the
	// faces point outward and agree with the normals set above.
	const std::vector<int32_t>& tris = mySolver.surfaceTriangles();
	myOutTriangles.resize(tris.size());
	for (size_t t = 0; t + 2 < tris.size(); t += 3)
	{
		myOutTriangles[t] = tris[t];
		myOutTriangles[t + 1] = tris[t + 2];
		myOutTriangles[t + 2] = tris[t + 1];
	}
	output->addTriangles(myOutTriangles.data(), static_cast<int32_t>(myOutTriangles.size() / 3));

	const std::vector<int32_t>& ids = mySolver.surfaceVertexBlobIds();
	SOP_CustomAttribData blobId("blobid", 1, AttribType::Int);
	blobId.intData = const_cast<int32_t*>(ids.data());
	output->setCustomAttribute(&blobId, numPoints);
}

void
SoftBodySOP::executeVBO(SOP_VBOOutput*, const OP_Inputs*, void*)
{
	// Not called since directToGPU is false.
}

void
SoftBodySOP::setupParameters(OP_ParameterManager* manager, void*)
{
	Parameters::setup(manager);
}

void
SoftBodySOP::pulsePressed(const char* name, void*)
{
	if (std::strcmp(name, ResetName) == 0)
		myResetRequested = true;
}

int32_t
SoftBodySOP::getNumInfoCHOPChans(void*)
{
	return 11;
}

void
SoftBodySOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void*)
{
	static const std::array<const char*, 11> names = {
		"blobs", "particles", "contacts", "seeds_outside", "started_small", "step_ms", "container_ms", "volume_ratio", "grow_ratio",
		"target_scale", "tets"
	};
	if (index < 0 || index >= static_cast<int32_t>(names.size()))
		return;
	chan->name->setString(names[index]);
	switch (index)
	{
		case 0: chan->value = static_cast<float>(mySolver.numBlobs()); break;
		case 1: chan->value = static_cast<float>(mySolver.numParticles()); break;
		case 2: chan->value = static_cast<float>(mySolver.numContacts()); break;
		case 3: chan->value = static_cast<float>(myLastReport.seedsOutside); break;
		case 4: chan->value = static_cast<float>(myLastReport.blobsStartedSmall); break;
		case 5: chan->value = myStepMs; break;
		case 6: chan->value = mySdfMs; break;
		case 7: chan->value = mySolver.currentVolumeRatio(); break;
		case 8: chan->value = mySolver.growRatio(); break;
		case 9: chan->value = mySolver.targetScale(); break;
		case 10: chan->value = static_cast<float>(mySolver.numTets()); break;
		default: break;
	}
}

void
SoftBodySOP::getWarningString(OP_String* warning, void*)
{
	warning->setString(myWarningString.c_str());
	myWarningString.clear();
}

void
SoftBodySOP::getErrorString(OP_String* error, void*)
{
	error->setString(myErrorString.c_str());
	myErrorString.clear();
}
