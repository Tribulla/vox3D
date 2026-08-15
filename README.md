# Vox3D

**A real-time voxel destruction physics engine.**

Vox3D is a fork of Erin Catto's [Box3D](https://github.com/erincatto/box3d)
specialized for **voxel structures**: it adds a built-in
structural-stress and fracture engine so voxel walls, towers, and machines bend,
crack, and shatter under load and impact — on top of Box3D's proven soft-step
rigid-body solver.

## What makes it "Vox3D"

Standard Box3D treats geometry as convex hulls, capsules, spheres, meshes, and
height fields. Vox3D adds a first-class notion of a **voxel lattice** — a body
made of unit cells on a grid — and reasons about it two ways:

- **Structurally** — a per-frame section-stress analysis over the lattice finds
  where a structure is overloaded and severs the bonds that fail, so cracks
  emerge from the actual load path instead of pre-authored seams.
- **Physically** — voxel bodies collide, stack, and break into independent rigid
  pieces, with the renderer instancing thousands of voxel cubes efficiently.

## Features

### Voxel stress + fracture (the core of Vox3D)

Public API in [`include/box3d/fracture.h`](include/box3d/fracture.h), implemented
in [`src/fracture.c`](src/fracture.c).

- **Voxel bodies** built from a set of lattice cells (`b3World_CreateFractureVoxels`,
  `b3World_CreateFractureBox`) with a configurable cell size.
- **Six built-in materials** — concrete, brick, stone, wood, glass, metal — each
  with density, strength, compressive factor, friction, and restitution.
- **Emergent fracturing.** A quasi-static section-stress solver (with d'Alembert
  inertial relief so tumbling bodies don't read as overloaded) drives
  self-weight bending failure; a separate impact path knocks loose a crater and
  severs boundary bonds on hard hits. Failed pieces split via connected-component
  analysis into new independent bodies.
- **Anchors** for fixing parts of a structure in place (`b3FractureAnchorFcn`).
- **Greedy-meshed collision** — solid runs of voxels can merge into fewer, larger
  box colliders (`def.merge`) to cut shape/broadphase/contact count.
- **Stress heatmap** and material/fragment colouring (`b3World_ApplyFractureColors`),
  and per-stage profiling (contact gather / analysis / sever / debris).
- **Tunable** through `b3FractureTuning`: strength scale, impact thresholds,
  minimum fragment size, debris caps, analysis stride, and parallel analysis
  (the contact gather and stress analysis parallelize across pieces).

### Voxel collision shape

`b3VoxelData` collision geometry (see
[`include/box3d/voxel.h`](include/box3d/voxel.h)) ported from the fast
KRUNCH_AVBD voxel collider: **one broadphase proxy per body** (not one per voxel)
plus a dedicated SAT narrowphase over the solid lattice, so large voxel bodies
collide cheaply and slot precisely into matching cavities. The data model and
query layer are implemented and tested; the SAT narrowphase and shape wiring are
the next phases.

### Samples

A voxel-only demo suite (~28 samples + a recording replay viewer), built with
sokol (D3D11 / Metal / OpenGL 4.5) and Dear ImGui, in four categories:

- **Voxel** — Sandbox, Materials, Shapes, Solid vs Shell, Anchors, Castle
- **Stress** — Cantilever, Tower, Bridge Span, Arch, Beam Snap, Overload, Jenga, Silo
- **Impact** — Wall Smash, Bullet Holes, Demolition, Crate Smash, Crush, Dominoes, Colonnade, Cratering
- **Benchmark** — Voxel Wall (Large), Voxel Rain, Collapse Storm, Stride Sweep, Parallel Analysis, Debris Storm

Every demo exposes the fracture tuning live, a stress/material/fragment colour
toggle, and a HUD profiler. Fire projectiles with **Space** or **Shift+Left-click**.

### Inherited from Box3D

- Robust *Soft Step* rigid-body solver, continuous collision, island-based sleep.
- Convex hulls, capsules, spheres, meshes, height fields; multiple shapes per body.
- Revolute, prismatic, distance, motor, weld, and wheel joints.
- Ray casts, shape casts, overlap queries, sensors, a character mover.
- Data-oriented C17 core, multithreading and SIMD, cross-platform determinism,
  recording and replay.

## Quick start (voxel API)

```c
#include "box3d/box3d.h"

b3WorldDef wd = b3DefaultWorldDef();
wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
b3WorldId world = b3CreateWorld( &wd );

// Enable the voxel stress + fracture subsystem: 1-unit cells, ground plane at y = 0.
b3World_EnableFracture( world, 1.0f, 0.0f );

// Build a wall out of concrete voxels on the integer lattice.
b3Vec3i cells[40 * 20];
int count = 0;
for ( int x = 0; x < 40; ++x )
    for ( int y = 0; y < 20; ++y )
        cells[count++] = ( b3Vec3i ){ x, y, 0 };

b3FractureDef def = b3DefaultFractureDef();
b3World_CreateFractureVoxels( world, cells, count, b3GetFractureMaterial( b3_fractureConcrete ), &def );

for ( int i = 0; i < 300; ++i )
    b3World_Step( world, 1.0f / 60.0f, 4 ); // the wall settles, stresses, and can fracture on impact
```

See [`docs/hello.md`](docs/hello.md) for the base Box3D hello-world.

## Building

Requires [CMake](https://cmake.org/) and a C17 compiler (plus C++20 for the
samples). The base engine builds on Windows, Linux, and macOS.

**CMake presets** (recommended):

- Windows: `cmake --preset windows` then `cmake --build --preset windows-release`
- Linux: `cmake --preset linux-release` then `cmake --build --preset linux-release`
- macOS: `cmake --preset macos` then `cmake --build --preset macos-release`
- Windows MinGW: `cmake --preset mingw-release` then `cmake --build --preset mingw-release`

**Visual Studio 2026** (the primary dev setup):

```bat
cmake -S . -B build -G "Visual Studio 18 2026"
cmake --build build --config Release --target samples -j
build\bin\Release\samples.exe
```

Run a demo headless (renders N frames then self-exits — used for smoke tests):

```bat
build\bin\Release\samples.exe --sample <index> --frames <N>
```

Unit tests build via the `test` target (`build/bin/Release/test.exe`), and include
the voxel/fracture suites.

SIMD (SSE2 / Neon) can be disabled with `BOX3D_DISABLE_SIMD`.

## Credits and license

Vox3D is a fork of **[Box3D](https://github.com/erincatto/box3d) by Erin Catto**.
All of the core rigid-body solver, collision, joints, and tooling are
Erin Catto's work; Vox3D adds the voxel stress/fracture engine and the
voxel collision shape on top.
The custom voxel collision shape is made by ThePlasticPotato (also known as tomato or handspoken)
and comes from the closed source Krunch_AVBD physics engine powering the
valkyrien skies minecraft mod.

Vox3D and Box3D are released under the [MIT license](LICENSE). The voxel collider
work is being ported from the KRUNCH_AVBD collider which is licensed as ARR but the voxel collider
code in vox3D is licensed under MIT with permission from the original creator to relicense it under MIT.

Please star and support upstream Box3D via
[GitHub Sponsors](https://github.com/sponsors/erincatto).

My development discord server where you can talk and ask questions about vox3D can be found here:
https://discord.gg/SJTxJeJBvb
