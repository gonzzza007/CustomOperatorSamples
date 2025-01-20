#include <string>
#include <array>
#include "CPlusPlus_Common.h"
#include "Parameters.h"

using namespace TD;

#pragma region Evals

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


#pragma endregion

#pragma region Setup

void
Parameters::setup(TD::OP_ParameterManager* manager)
{
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


}

#pragma endregion