#include <string>
#include <array>
#include "CPlusPlus_Common.h"
#include "Parameters.h"

using namespace TD;

#pragma region Evals

ModifierMenuItems
Parameters::evalModifier(const TD::OP_Inputs* input)
{
	return static_cast<ModifierMenuItems>(input->getParInt(ModifierName));
}

double
Parameters::evalRadius(const TD::OP_Inputs* input)
{
	return input->getParDouble(RadiusName);
}

double
Parameters::evalEdgeAngle(const TD::OP_Inputs* input)
{
	return input->getParDouble(EdgeAngleName);
}

double
Parameters::evalDeflection(const TD::OP_Inputs* input)
{
	return input->getParDouble(DeflectionName);
}

double
Parameters::evalAngularDeflection(const TD::OP_Inputs* input)
{
	return input->getParDouble(AngularDeflectionName);
}

NormalsMenuItems
Parameters::evalNormals(const TD::OP_Inputs* input)
{
	return static_cast<NormalsMenuItems>(input->getParInt(NormalsName));
}

#pragma endregion

#pragma region Setup

void
Parameters::setup(TD::OP_ParameterManager* manager)
{
	{
		OP_StringParameter p;
		p.name = ModifierName;
		p.label = ModifierLabel;
		p.page = "OpenCascade";
		p.defaultValue = "Fillet";
		std::array<const char*, 3> Names =
		{
			"Fillet",
			"Chamfer",
			"Planecut"
		};
		std::array<const char*, 3> Labels =
		{
			"Fillet",
			"Chamfer",
			"Chamfer (Plane Cut)"
		};
		OP_ParAppendResult res = manager->appendMenu(p, int(Names.size()), Names.data(), Labels.data());
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = RadiusName;
		p.label = RadiusLabel;
		p.page = "OpenCascade";
		p.defaultValues[0] = 0.05;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 0.5;
		p.minValues[0] = 0.0;
		p.clampMins[0] = true;
		OP_ParAppendResult res = manager->appendFloat(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = EdgeAngleName;
		p.label = EdgeAngleLabel;
		p.page = "OpenCascade";
		p.defaultValues[0] = 30.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 180.0;
		p.minValues[0] = 0.0;
		p.maxValues[0] = 180.0;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendFloat(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = DeflectionName;
		p.label = DeflectionLabel;
		p.page = "OpenCascade";
		p.defaultValues[0] = 0.002;
		p.minSliders[0] = 0.0001;
		p.maxSliders[0] = 0.05;
		p.minValues[0] = 0.00001;
		p.clampMins[0] = true;
		OP_ParAppendResult res = manager->appendFloat(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = AngularDeflectionName;
		p.label = AngularDeflectionLabel;
		p.page = "OpenCascade";
		p.defaultValues[0] = 15.0;
		p.minSliders[0] = 1.0;
		p.maxSliders[0] = 45.0;
		p.minValues[0] = 0.1;
		p.maxValues[0] = 90.0;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendFloat(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_StringParameter p;
		p.name = NormalsName;
		p.label = NormalsLabel;
		p.page = "OpenCascade";
		p.defaultValue = "Mesh";
		std::array<const char*, 3> Names =
		{
			"Off",
			"Mesh",
			"Surface"
		};
		std::array<const char*, 3> Labels =
		{
			"Off",
			"Mesh",
			"Surface"
		};
		OP_ParAppendResult res = manager->appendMenu(p, int(Names.size()), Names.data(), Labels.data());
		assert(res == OP_ParAppendResult::Success);
	}
}

#pragma endregion
