# Soft Body SOP

Soft, squishy blobs trapped inside a closed container. Each blob is an icosphere whose
vertices are simulated as particles with an XPBD (extended position based dynamics) solver.

## Inputs

1. **Container** – any closed mesh, e.g. a 1×1×1 Box SOP. Polygons are fan-triangulated and turned
   into a signed distance field, so concave shapes work. The field is rebuilt only when the
   container geometry actually changes (rebuilding takes ~10–150 ms, so avoid animating the
   container's points every frame; move it with a transform instead).
2. **Seeds** – one point per blob, e.g. a Sprinkle SOP filling the volume of the container.
   Optional point attributes override the parameters per blob:
   * `radius` (or `pscale`) – blob radius
   * `stiffness` – 0..1
   * `mass` – kg

Every seed inside the container gets a blob. On reset a blob starts as large as fits between its
neighbours and the walls (possibly tiny) and then grows towards its target radius, squeezing its
neighbours. Seeds outside the container are skipped and reported as a warning.

## Output

Triangulated blob meshes with point normals and an integer point attribute `blobid`.

## Parameters

### Simulation
* **Simulate** – advance the simulation every frame.
* **Reset** – rebuild the blobs from the inputs. Changing the seeds or Edge Length, Min/Max Points, Mass Mode,
  Mass or Collision Radius also resets. Radius Min/Max, Seed and Fill do **not** reset: the blobs grow or shrink
  live, but keep the point count chosen at the last reset. Press Reset after a big size change to re-plan it.
* **Stiffness** – 0..1, how strongly the surface keeps its edge lengths (jelly → rubber).
* **Bending** – 0..1, resistance to sharp folds. Removes wrinkles that would otherwise stay after a blob was
  squeezed, while flat contact faces stay possible. 0 turns it off (cheaper, wrinkly, flabby).
* **Volume Stiffness** – 0..1, how strongly a blob keeps its volume.
* **Pressure** – rest volume multiplier; >1 inflates the blobs, <1 deflates them.
* **Fill** – 0..0.99, the fraction of the container the blobs may occupy together. When the target radii
  ask for more, all targets are scaled down uniformly to fit (Info CHOP `target_scale` < 1). Asking for
  more than fits would make blobs overlap. Use 0.9–0.99 with Gravity 0 for a tightly packed, Voronoi-like look.
* **Grow Speed** – 1/s, how fast blobs approach their target radius. 0 freezes their size.
* **Shape Memory** – 0..1, pull towards the original sphere shape (shape matching). Prevents crumpling at low stiffness.
* **Damping** – velocity damping per second.
* **Friction** – 0..1, against the walls and between blobs.
* **Restitution** – 0..1, bounce off the container walls.
* **Gravity** – gravity vector; export a CHOP to it to tilt or shake the box.
* **Substeps / Iterations** – solver quality vs. speed. More substeps help fast motion and collisions;
  more iterations make constraints stiffer.

### Blobs
* **Radius Min / Max** – random target radius range per blob. Sliders go to 100, larger values can be typed.
  Targets larger than the container are fine: Fill limits the actual size.
* **Seed** – random seed for the radii.
* **Edge Length** – target spacing between particles. Each blob is a geodesic sphere whose point count is
  chosen on reset from the size it will reach (its target radius after the Fill scaling), so all blobs get
  about the same particle spacing and behave alike regardless of size.
* **Min Points / Max Points** – clamp for the points per blob. Geodesic spheres come in
  10·n²+2 points: 12, 42, 92, 162, 252, 362, 492, 642, 812, 1002, … 2562 (n = 16).
* **Mass Mode / Mass** – density in kg/m³ (bigger blobs are heavier) or a fixed mass per blob.
* **Collision Radius** – particle radius for collisions, 0 = half the Edge Length. Blobs touch at this distance
  from their points; coarse blobs get a slightly larger radius so others can't slip between their points.
* **Surface Offset** – draw the surface pushed out by the collision radius, so touching blobs look touching.
  Off draws the simulated points themselves, which leaves a visible gap of 2 × Collision Radius between blobs.
* **Container Resolution** – cells along the longest side of the container's signed distance field.

## Info CHOP channels

`blobs`, `particles`, `contacts`, `seeds_outside`, `started_small`, `step_ms`, `container_ms`, `volume_ratio`,
`grow_ratio` (current / target radius, 1 = fully grown), `target_scale` (how much Fill scaled the targets down).

## Tight packing (Voronoi-like look)

* Fill 0.95–0.99, Gravity 0, Volume Stiffness 1, Stiffness 0.2–0.4, Shape Memory 0, low Friction.
* Many seeds (e.g. 20–50) and Radius Min/Max at or above the size that would fill the container.
* With Surface Offset on the cells touch visually; without it the gap between cells is 2 × Collision Radius.
  Edges are strain limited to 1.3× their rest length so blobs can't slip through each other.

## Notes

* Build in **Release**; a Debug build of the solver is many times slower.
* The solver is single-threaded. A few thousand particles at 4 substeps × 8 iterations run in a few ms.
* The solver files (`SoftBodySolver`, `ContainerSDF`, `Icosphere` (geodesic spheres), `Vec3.h`) have no TouchDesigner
  dependency and can be built and tested standalone.
