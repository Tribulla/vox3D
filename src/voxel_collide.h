#pragma once

#include "box3d/math_functions.h"
#include "box3d/voxel.h"

typedef struct b3VoxelOBB
{
	b3Vec3 center;
	b3Vec3 axes[3]; // unit
	b3Vec3 half;
} b3VoxelOBB;

typedef struct b3VoxelContact
{
	b3Vec3 normal;			  // unit, points from box1 (B) toward box0 (A)
	float initialPenetration; // signed: > 0 overlapping, <= 0 speculative gap
	float penetrationDepth;	  // ranking only (>= 0)
	b3Vec3 body0Point;		  // world witness on box0 (A) surface
	b3Vec3 body1Point;		  // world witness on box1 (B) surface
	uint32_t featureId;		  // stable id (the colliding cell pair) for warm-starting; 0 from the raw box primitives
} b3VoxelContact;

b3AABB b3VoxelOBB_Bounds( const b3VoxelOBB* box );

int b3VoxelCollideOBB( const b3VoxelOBB* box0, const b3VoxelOBB* box1, float contactDistance, int maxContacts,
					   b3VoxelContact* out );

int b3VoxelCollideAABB( const b3AABB* box0, const b3AABB* box1, float contactDistance, int maxContacts, b3VoxelContact* out );

int b3VoxelCollide( const b3VoxelData* v0, b3Transform xf0, const b3VoxelData* v1, b3Transform xf1, float contactDistance,
					int maxContacts, b3VoxelContact* out );

void b3Voxel_addReduced( const b3VoxelContact* c, b3VoxelContact* acc, int* nacc, int maxContacts );
