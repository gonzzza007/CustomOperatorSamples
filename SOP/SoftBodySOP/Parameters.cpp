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

#include <cstdint>
#include "CPlusPlus_Common.h"
#include "Parameters.h"

#include <array>
#include <cassert>
#include <cstring>
#include <string>

namespace
{

void
appendFloat(TD::OP_ParameterManager* manager, const char* name, const char* label, const char* page,
			double def, double sliderMin, double sliderMax, bool clampMin = false, double minValue = 0.0,
			bool clampMax = false, double maxValue = 1.0)
{
	TD::OP_NumericParameter p;
	p.name = name;
	p.label = label;
	p.page = page;
	p.defaultValues[0] = def;
	p.minSliders[0] = sliderMin;
	p.maxSliders[0] = sliderMax;
	p.clampMins[0] = clampMin;
	p.minValues[0] = minValue;
	p.clampMaxes[0] = clampMax;
	p.maxValues[0] = maxValue;
	TD::OP_ParAppendResult res = manager->appendFloat(p);
	assert(res == TD::OP_ParAppendResult::Success);
	(void)res;
}

void
appendInt(TD::OP_ParameterManager* manager, const char* name, const char* label, const char* page,
			int def, int sliderMin, int sliderMax, int minValue, int maxValue)
{
	TD::OP_NumericParameter p;
	p.name = name;
	p.label = label;
	p.page = page;
	p.defaultValues[0] = def;
	p.minSliders[0] = sliderMin;
	p.maxSliders[0] = sliderMax;
	p.clampMins[0] = true;
	p.minValues[0] = minValue;
	p.clampMaxes[0] = true;
	p.maxValues[0] = maxValue;
	TD::OP_ParAppendResult res = manager->appendInt(p);
	assert(res == TD::OP_ParAppendResult::Success);
	(void)res;
}

}

#pragma region Evals

bool
Parameters::evalSimulate(const TD::OP_Inputs* inputs)
{
	return inputs->getParInt(SimulateName) != 0;
}

sb::SimSettings
Parameters::evalSimSettings(const TD::OP_Inputs* inputs)
{
	sb::SimSettings s;
	s.stiffness = static_cast<float>(inputs->getParDouble(StiffnessName));
	s.volumeStiffness = static_cast<float>(inputs->getParDouble(VolumestiffnessName));
	s.pressure = static_cast<float>(inputs->getParDouble(PressureName));
	s.shapeMemory = static_cast<float>(inputs->getParDouble(ShapememoryName));
	s.damping = static_cast<float>(inputs->getParDouble(DampingName));
	s.friction = static_cast<float>(inputs->getParDouble(FrictionName));
	s.restitution = static_cast<float>(inputs->getParDouble(RestitutionName));

	double gx = 0.0, gy = 0.0, gz = 0.0;
	inputs->getParDouble3(GravityName, gx, gy, gz);
	s.gravity = sb::Vec3(static_cast<float>(gx), static_cast<float>(gy), static_cast<float>(gz));

	s.substeps = inputs->getParInt(SubstepsName);
	s.iterations = inputs->getParInt(IterationsName);
	return s;
}

sb::GenerationSettings
Parameters::evalGenerationSettings(const TD::OP_Inputs* inputs)
{
	sb::GenerationSettings g;
	g.radiusMin = static_cast<float>(inputs->getParDouble(RadiusminName));
	g.radiusMax = static_cast<float>(inputs->getParDouble(RadiusmaxName));
	g.seed = static_cast<uint32_t>(inputs->getParInt(SeedName));
	g.edgeLength = static_cast<float>(inputs->getParDouble(EdgelengthName));
	g.minLevel = inputs->getParInt(MinlevelName);
	g.maxLevel = inputs->getParInt(MaxlevelName);

	const char* mode = inputs->getParString(MassmodeName);
	g.massMode = (mode && std::strcmp(mode, "Perblob") == 0) ? sb::MassMode::PerBlob : sb::MassMode::Density;
	g.mass = static_cast<float>(inputs->getParDouble(MassName));
	g.collisionRadius = static_cast<float>(inputs->getParDouble(CollisionradiusName));
	return g;
}

int
Parameters::evalSdfresolution(const TD::OP_Inputs* inputs)
{
	return inputs->getParInt(SdfresolutionName);
}

#pragma endregion

#pragma region Setup

void
Parameters::setup(TD::OP_ParameterManager* manager)
{
	const char* sim = "Simulation";
	const char* blobs = "Blobs";

	// --- Simulation page ---
	{
		TD::OP_NumericParameter p;
		p.name = SimulateName;
		p.label = SimulateLabel;
		p.page = sim;
		p.defaultValues[0] = 1.0;
		TD::OP_ParAppendResult res = manager->appendToggle(p);
		assert(res == TD::OP_ParAppendResult::Success);
		(void)res;
	}
	{
		TD::OP_NumericParameter p;
		p.name = ResetName;
		p.label = ResetLabel;
		p.page = sim;
		TD::OP_ParAppendResult res = manager->appendPulse(p);
		assert(res == TD::OP_ParAppendResult::Success);
		(void)res;
	}

	appendFloat(manager, StiffnessName, StiffnessLabel, sim, 0.5, 0.0, 1.0, true, 0.0, true, 1.0);
	appendFloat(manager, VolumestiffnessName, VolumestiffnessLabel, sim, 0.9, 0.0, 1.0, true, 0.0, true, 1.0);
	appendFloat(manager, PressureName, PressureLabel, sim, 1.0, 0.2, 2.0, true, 0.01);
	appendFloat(manager, ShapememoryName, ShapememoryLabel, sim, 0.0, 0.0, 1.0, true, 0.0, true, 1.0);
	appendFloat(manager, DampingName, DampingLabel, sim, 0.5, 0.0, 5.0, true, 0.0);
	appendFloat(manager, FrictionName, FrictionLabel, sim, 0.3, 0.0, 1.0, true, 0.0, true, 1.0);
	appendFloat(manager, RestitutionName, RestitutionLabel, sim, 0.2, 0.0, 1.0, true, 0.0, true, 1.0);

	{
		TD::OP_NumericParameter p;
		p.name = GravityName;
		p.label = GravityLabel;
		p.page = sim;
		const double def[3] = { 0.0, -9.8, 0.0 };
		for (int i = 0; i < 3; ++i)
		{
			p.defaultValues[i] = def[i];
			p.minSliders[i] = -20.0;
			p.maxSliders[i] = 20.0;
		}
		TD::OP_ParAppendResult res = manager->appendXYZ(p);
		assert(res == TD::OP_ParAppendResult::Success);
		(void)res;
	}

	appendInt(manager, SubstepsName, SubstepsLabel, sim, 4, 1, 16, 1, 64);
	appendInt(manager, IterationsName, IterationsLabel, sim, 8, 1, 30, 1, 100);

	// --- Blobs page (changes regenerate the blobs) ---
	appendFloat(manager, RadiusminName, RadiusminLabel, blobs, 0.06, 0.005, 0.5, true, 0.001);
	appendFloat(manager, RadiusmaxName, RadiusmaxLabel, blobs, 0.12, 0.005, 0.5, true, 0.001);
	appendInt(manager, SeedName, SeedLabel, blobs, 1, 0, 100, 0, 1 << 30);
	appendFloat(manager, EdgelengthName, EdgelengthLabel, blobs, 0.025, 0.005, 0.1, true, 0.001);
	appendInt(manager, MinlevelName, MinlevelLabel, blobs, 1, 0, 7, 0, 7);
	appendInt(manager, MaxlevelName, MaxlevelLabel, blobs, 4, 0, 7, 0, 7);

	{
		TD::OP_StringParameter p;
		p.name = MassmodeName;
		p.label = MassmodeLabel;
		p.page = blobs;
		p.defaultValue = "Density";
		std::array<const char*, 2> names = { "Density", "Perblob" };
		std::array<const char*, 2> labels = { "Density (kg/m^3)", "Per Blob (kg)" };
		TD::OP_ParAppendResult res = manager->appendMenu(p, static_cast<int32_t>(names.size()), names.data(), labels.data());
		assert(res == TD::OP_ParAppendResult::Success);
		(void)res;
	}

	appendFloat(manager, MassName, MassLabel, blobs, 1000.0, 1.0, 5000.0, true, 0.0001);
	appendFloat(manager, CollisionradiusName, CollisionradiusLabel, blobs, 0.0, 0.0, 0.05, true, 0.0);
	appendInt(manager, SdfresolutionName, SdfresolutionLabel, blobs, 64, 16, 256, 8, 512);
}

#pragma endregion
