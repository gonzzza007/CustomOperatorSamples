#include <string>
#include <array>
#include "CPlusPlus_Common.h"
#include "Parameters.h"

using namespace TD;

#pragma region Evals

ModeMenuItems
Parameters::evalMode(const TD::OP_Inputs* input)
{
	ModeMenuItems value = static_cast<ModeMenuItems>(input->getParInt(ModeName));
	return value;
}

double
Parameters::evalAlpha(const TD::OP_Inputs* inputs) {
	return inputs->getParDouble(AlphaName);
}

bool
Parameters::evalUseOptimalAlpha(const TD::OP_Inputs* inputs) {
	bool useOptimalAlpha = inputs->getParInt(UseOptimalAlphaName) ? true : false;
	inputs->enablePar(AlphaName, !useOptimalAlpha);
	return useOptimalAlpha;
}

bool
Parameters::evalSkipInteriorPoints(const TD::OP_Inputs* inputs) {
	return inputs->getParInt(SkipInteriorPointsName) ? true : false;
}




#pragma endregion

#pragma region Setup

void
Parameters::setup(TD::OP_ParameterManager* manager)
{
	{
		OP_StringParameter p;
		p.name = ModeName;
		p.label = ModeLabel;
		p.page = "Alpha Shapes";
		p.defaultValue = "Regularized";
		std::array<const char*, 2> Names =
		{
			"Regularized",
			"General"
		};
		std::array<const char*, 2> Labels =
		{
			"Regularized",
			"General"
		};
		OP_ParAppendResult res = manager->appendMenu(p, int(Names.size()), Names.data(), Labels.data());
		assert(res == OP_ParAppendResult::Success);
	}
	
	{
		OP_NumericParameter p;
		p.name = UseOptimalAlphaName;
		p.label = UseOptimalAlphaLabel;
		p.page = "Alpha Shapes";
		p.defaultValues[0] = false;
		OP_ParAppendResult res = manager->appendToggle(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = AlphaName;
		p.label = AlphaLabel;
		p.page = "Alpha Shapes";
		p.defaultValues[0] = 0.05;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 1.0;
		p.minValues[0] = 0.0;
		p.maxValues[0] = 1.0;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendFloat(p);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter p;
		p.name = SkipInteriorPointsName;
		p.label = SkipInteriorPointsLabel;
		p.page = "Alpha Shapes";
		p.defaultValues[0] = true;
		OP_ParAppendResult res = manager->appendToggle(p);
		assert(res == OP_ParAppendResult::Success);
	}

}

#pragma endregion