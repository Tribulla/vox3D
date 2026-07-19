#pragma once

#include "base.h"
#include "id.h"
#include "math_functions.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3HullData b3HullData;


typedef struct b3FractureMaterial
{
	float density;			 ///< kg/m^3
	float strength;			 ///< failure fibre stress
	float compressiveFactor; ///< compressive strength / tensile strength (>= 1)
	float restitution;
	float friction;
	uint32_t color; ///< 0xRRGGBB debug-draw colour
} b3FractureMaterial;

typedef enum b3FractureMaterialType
{
	b3_fractureConcrete,
	b3_fractureBrick,
	b3_fractureStone,
	b3_fractureWood,
	b3_fractureGlass,
	b3_fractureMetal,
} b3FractureMaterialType;

B3_API b3FractureMaterial b3GetFractureMaterial( b3FractureMaterialType type );

typedef bool b3FractureAnchorFcn( b3Vec3i cell, void* context );

/// Options for a destructible body.
typedef struct b3FractureDef
{
	b3Vec3 velocity;

	bool isStatic;

	bool merge;

	b3FractureAnchorFcn* anchor;
	void* anchorContext;

	int internalValue;
} b3FractureDef;

B3_API b3FractureDef b3DefaultFractureDef( void );

typedef struct b3FractureTuning
{
	float strengthScale;	 ///< global multiplier on every material strength
	float contactSmoothing;	 ///< EMA factor on contact forces (per-frame jitter filter), (0,1]
	float fractureRoughness; ///< voxels the crack surface wanders; 0 = flat plane
	float settleSpeed;		 ///< only fracture near-resting bodies, unless impacted
	float impactSpeed;		 ///< a contact approaching faster than this counts as an impact
	float impactBearing;	 ///< impact bearing stress must beat strength x this
	int impactRadius;		 ///< rings of voxels a hard hit knocks loose as a chunk
	int minFragment;		 ///< fragments smaller than this are deleted as dust
	int fractureHoldFrames;	 ///< a resting body must stay overloaded this long to snap
	int warmupFrames;		 ///< let a fresh scene settle before judging its stress
	int analysisStride;		 ///< stress-analyse each awake body every K frames (1 = every frame). Impacts are
							 ///< still detected every frame; a non-analysed body keeps its last stress values.
	int maxDebris;			 ///< cap on loose debris pieces (single chunk / single voxel) kept alive; the oldest
							 ///< are removed, a few per step, once over the cap. 0 = unlimited. Big collapses
							 ///< otherwise leave thousands of rubble bodies for the solver every step.
	bool fractureEnabled;	 ///< master switch: analyse only vs. actually sever
	bool contactStress;		 ///< fold Box3D contact forces into the analysis
	bool stressEnabled;		 ///< run the lattice stress analysis (b3_fractureStress severs). When false only
	bool parallelAnalysis;	 ///< run the per-body contact gathering and stress analysis on the world's
							 ///< worker threads (needs b3WorldDef.workerCount > 1; all analyses then see
							 ///< the same pre-fracture snapshot instead of earlier splits in the same step)
} b3FractureTuning;

B3_API b3FractureTuning b3DefaultFractureTuning( void );

typedef enum b3FractureColorMode
{
	b3_fractureColorMaterial,
	b3_fractureColorStress,	 ///< blue (safe) -> red (at the breaking point)
	b3_fractureColorFragment,
} b3FractureColorMode;

B3_API void b3World_EnableFracture( b3WorldId worldId, float voxel, float groundY );

B3_API bool b3World_IsFractureEnabled( b3WorldId worldId );

B3_API void b3World_SetFractureTuning( b3WorldId worldId, b3FractureTuning tuning );
B3_API b3FractureTuning b3World_GetFractureTuning( b3WorldId worldId );

B3_API int b3World_CreateFractureVoxels( b3WorldId worldId, const b3Vec3i* cells, int count,
										 b3FractureMaterial material, const b3FractureDef* def );

B3_API int b3World_CreateFractureBox( b3WorldId worldId, b3Vec3 center, b3Vec3 halfExtents,
									  b3FractureMaterial material, const b3FractureDef* def );

B3_API int b3World_CreateFractureConvex( b3WorldId worldId, const b3HullData* hull, b3WorldTransform transform,
										 int chunkCount, b3FractureMaterial material, const b3FractureDef* def );

B3_API int b3World_MakeBodyFracture( b3WorldId worldId, b3BodyId bodyId, b3FractureMaterial material,
									 const b3FractureDef* def );

/// Make a body carrying a single b3_voxelShape destructible, running the fracture solver
/// directly on that shape's cell grid (the SAME grid used for collision) — no parallel
/// voxel body is created. On fracture the severed cells are removed from the parent shape
/// in place (its collider, mass and AABB update automatically) and reported via
/// b3World_GetFractureEvents; vox3D does NOT spawn or simulate the fragment body — the host
/// materialises the new body from the event. Requires b3World_EnableFracture first (the
/// body's voxel size must match the fracture world's). The body must have exactly one
/// b3_voxelShape and (for impact fracture to gather contact forces) that shape should have
/// enableContactEvents = true. @return an internal piece index (>= 0), or -1 on failure.
B3_API int b3World_MakeVoxelBodyFracture( b3WorldId worldId, b3BodyId bodyId, b3FractureMaterial material,
										  const b3FractureDef* def );

/// Why a set of cells severed this step.
typedef enum b3FractureReason
{
	b3_fractureImpact, ///< a hard contact knocked the cells loose
	b3_fractureStress  ///< the cells failed under accumulated load (bending/tension)
} b3FractureReason;

/// One fragment that severed from a destructible b3_voxelShape this step. The @p cells are
/// in the SAME integer grid coordinates the host passed to b3CreateVoxelData, so they map
/// straight back to blocks. @p mass, @p centerOfMassWorld and the inherited velocities let
/// the host spawn the fragment with correct momentum (v = linearVelocity + angularVelocity
/// x (p - centerOfMassWorld) for a point p).
typedef struct b3FractureEvent
{
	b3BodyId parentBody;		 ///< the body that lost these cells (its shape was updated in place)
	b3BodyId fragmentBody;		 ///< the spawned fragment, or b3_nullBodyId when host-materialised
	uint64_t fragmentId;		 ///< stable id for this fragment this step
	b3FractureReason reason;	 ///< impact vs stress
	const b3Vec3i* cells;		 ///< severed cells, in the host's b3CreateVoxelData coordinates
	int cellCount;				 ///< number of entries in @p cells
	float mass;					 ///< fragment mass (sum of its cell masses)
	b3Vec3 centerOfMassWorld;	 ///< fragment centre of mass, world space
	b3Vec3 linearVelocity;		 ///< inherited world linear velocity of the fragment's centre of mass
	b3Vec3 angularVelocity;		 ///< inherited world angular velocity
} b3FractureEvent;

/// The set of fracture events produced by the most recent b3World_Step. The view (and the
/// @p cells pointers inside each event) is valid only until the next b3World_Step; do not
/// store it. Empty when fracture is disabled.
typedef struct b3FractureEvents
{
	b3FractureEvent* events;
	int count;
} b3FractureEvents;

/// Get the fracture events for the current time step. Transient — see b3FractureEvents.
B3_API b3FractureEvents b3World_GetFractureEvents( b3WorldId worldId );

B3_API void b3World_ApplyFractureColors( b3WorldId worldId, b3FractureColorMode mode );

B3_API int b3World_GetFractureBodyCount( b3WorldId worldId );

B3_API int b3World_GetFractureVoxelCount( b3WorldId worldId );

B3_API float b3World_GetFractureMaxStress( b3WorldId worldId );

/**@}*/
