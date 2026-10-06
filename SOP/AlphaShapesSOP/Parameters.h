#pragma once

namespace TD
{
	class OP_Inputs;
	class OP_ParameterManager;
}

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

constexpr static char NormalsName[] = "Normals";
constexpr static char NormalsLabel[] = "Normals";

#pragma endregion

#pragma region Menus
enum class ModeMenuItems
{
	Regularized, // default
	General
};

enum class NormalsMenuItems
{
	Off,
	Smooth,	// default, shared points, averaged over the triangles around each point
	Flat	// every triangle has its own points (like Facet SOP > Unique Points)
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

	// Normals
	static NormalsMenuItems	evalNormals(const TD::OP_Inputs* input);

};
#pragma endregion