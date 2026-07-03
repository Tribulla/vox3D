#pragma once

#include "base.h"
#include "id.h"
#include "math_functions.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3HullData b3HullData;

typedef struct b3Vec3i
{
	int32_t x, y, z;
} b3Vec3i;

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
	bool fractureEnabled;	 ///< master switch: analyse only vs. actually sever
	bool contactStress;		 ///< fold Box3D contact forces into the analysis
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

B3_API void b3World_ApplyFractureColors( b3WorldId worldId, b3FractureColorMode mode );

B3_API int b3World_GetFractureBodyCount( b3WorldId worldId );

B3_API int b3World_GetFractureVoxelCount( b3WorldId worldId );

B3_API float b3World_GetFractureMaxStress( b3WorldId worldId );

/**@}*/
