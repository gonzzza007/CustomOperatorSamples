#pragma once

class TD::OP_Inputs;
class TD::OP_ParameterManager;

#pragma region ParNames and ParLabels

// Names of the parameters
constexpr static char ModeName[] = "Mode";
constexpr static char ModeLabel[] = "Mode";

constexpr static char UseOptimalAlphaName[] = "Useoptimalalpha";
constexpr static char UseOptimalAlphaLabel[] = "Use optimal Alpha";

constexpr static char AlphaName[] = "Alpha";
constexpr static char AlphaLabel[] = "Alpha value";

constexpr static char SkipInteriorPointsName[] = "Skipinterior";
constexpr static char SkipInteriorPointsLabel[] = "Skip interior points";

#pragma endregion

#pragma region Menus
enum class ModeMenuItems
{
	Regularized, // default
	General
};
#pragma endregion

#pragma region Parameters
class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	// Mode
	static ModeMenuItems	evalMode(const TD::OP_Inputs* input);

	// calculate and use optimal Alpha?
	static bool		evalUseOptimalAlpha(const TD::OP_Inputs* input);

	// set alpha manually
	static double evalAlpha(const TD::OP_Inputs* input);

	// skip interior points?
	static bool		evalSkipInteriorPoints(const TD::OP_Inputs* input);

};
#pragma endregion