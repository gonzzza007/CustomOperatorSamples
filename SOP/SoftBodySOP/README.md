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

Radii are shrunk on reset so blobs neither overlap each other nor the container walls. Seeds outside
the container, or without room for a blob, are skipped and reported as a warning.

## Output

Triangulated blob meshes with point normals and an integer point attribute `blobid`.

## Parameters

### Simulation
* **Simulate** – advance the simulation every frame.
* **Reset** – rebuild the blobs from the inputs. Changing the seeds or any parameter on the Blobs page also resets.
* **Stiffness** – 0..1, how strongly the surface keeps its edge lengths (jelly → rubber).
* **Volume Stiffness** – 0..1, how strongly a blob keeps its volume.
* **Pressure** – rest volume multiplier; >1 inflates the blobs, <1 deflates them.
* **Shape Memory** – 0..1, pull towards the original sphere shape (shape matching). Prevents crumpling at low stiffness.
* **Damping** – velocity damping per second.
* **Friction** – 0..1, against the walls and between blobs.
* **Restitution** – 0..1, bounce off the container walls.
* **Gravity** – gravity vector; export a CHOP to it to tilt or shake the box.
* **Substeps / Iterations** – solver quality vs. speed. More substeps help fast motion and collisions;
  more iterations make constraints stiffer.

### Blobs
* **Radius Min / Max** – random radius range per blob.
* **Seed** – random seed for the radii.
* **Edge Length** – target spacing between particles. The icosphere level of each blob is chosen
  from its radius so that all blobs have about the same particle density.
* **Min Level / Max Level** – clamp for the icosphere subdivision level
  (level 1 = 42 points, 2 = 162, 3 = 642, 4 = 2562).
* **Mass Mode / Mass** – density in kg/m³ (bigger blobs are heavier) or a fixed mass per blob.
* **Collision Radius** – particle radius for collisions, 0 = half the Edge Length.
* **Container Resolution** – cells along the longest side of the container's signed distance field.

## Info CHOP channels

`blobs`, `particles`, `contacts`, `seeds_outside`, `seeds_too_tight`, `step_ms`, `container_ms`, `volume_ratio`.

## Notes

* Build in **Release**; a Debug build of the solver is many times slower.
* The solver is single-threaded. A few thousand particles at 4 substeps × 8 iterations run in a few ms.
* The solver files (`SoftBodySolver`, `ContainerSDF`, `Icosphere`, `Vec3.h`) have no TouchDesigner
  dependency and can be built and tested standalone.
