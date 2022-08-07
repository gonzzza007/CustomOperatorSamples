# Generator SOP
This basic Generator SOP example can be used to generate SOP data such as point, lines, squares... etc.

## Parameters
* **Shape** - The shape of the SOP to generate.
  * **Point** - A single point.
  * **Line** - A simple 2 points line.
  * **Square** - A triangulated square.
  * **Cube** - A triangulated cube.
  * **Divider** - A cube with axis parallel division.
  * **Voronoi** - A cube with voronoi division.
  * **KDTree** - A cube with KDTree division.
* **Input points CHOP** - 3 channel CHOP where each sample is a 3D coordinate of point used for Divider/Voronoi/KDTree cube division.
* **Color** - The point color attribute value.
* **GPU Direct** - Load the geometry directly to the GPU.