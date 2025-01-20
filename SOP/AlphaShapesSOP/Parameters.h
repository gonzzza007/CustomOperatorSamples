#pragma once

class TD::OP_Inputs;
class TD::OP_ParameterManager;

#pragma region ParNames and ParLabels

// Names of the parameters
constexpr static char UseOptimalAlphaName[] = "Useoptimalalpha";
constexpr static char UseOptimalAlphaLabel[] = "Use optimal Alpha";

constexpr static char AlphaName[] = "Alpha";
constexpr static char AlphaLabel[] = "Alpha value";


#pragma endregion

#pragma region Menus
#pragma endregion

#pragma region Parameters
class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	// calculate and use optimal Alpha?
	static bool		evalUseOptimalAlpha(const TD::OP_Inputs* input);

	// set alpha manually
	static double evalAlpha(const TD::OP_Inputs* input);



};
#pragma endregion