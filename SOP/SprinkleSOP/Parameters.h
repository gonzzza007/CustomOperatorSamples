#pragma once

class TD::OP_Inputs;
class TD::OP_ParameterManager;

#pragma region ParNames and ParLabels

// Names of the parameters

constexpr static char SeedName[] = "Seed";
constexpr static char SeedLabel[] = "Seed";

constexpr static char GenerateName[] = "Generate";
constexpr static char GenerateLabel[] = "Generate";

constexpr static char PointcountName[] = "Pointcount";
constexpr static char PointcountLabel[] = "Point Count";

constexpr static char SeparatepointsName[] = "Separatepoints";
constexpr static char SeparatepointsLabel[] = "Separate Points";

constexpr static char MinimumdistanceName[] = "Minimumdistance";
constexpr static char MinimumdistanceLabel[] = "Minimum Distance";


#pragma endregion

#pragma region Menus
enum class GenerateMenuItems
{
	Area,
	Primitive,
	Boundingbox,
	Volume
};

#pragma endregion


#pragma region Parameters
class Parameters
{
public:
	static void		setup(TD::OP_ParameterManager*);

	// Seed
	static double		evalSeed(const TD::OP_Inputs* input);

	// Generate
	static GenerateMenuItems		evalGenerate(const TD::OP_Inputs* input);

	// Point Count
	static int		evalPointcount(const TD::OP_Inputs* input);

	// Separate Points
	static bool		evalSeparatepoints(const TD::OP_Inputs* input);

	// Minimum Distance
	static double		evalMinimumdistance(const TD::OP_Inputs* input);


};
#pragma endregion