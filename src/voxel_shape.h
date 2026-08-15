#pragma once

#include "box3d/math_functions.h"
#include "box3d/types.h"
#include "box3d/voxel.h"

#include <stdbool.h>
#include <stdint.h>

float b3Voxel_GetVoxelSize( const b3VoxelData* v );
int b3Voxel_GetCellCount( const b3VoxelData* v );
uint32_t b3Voxel_GetHash( const b3VoxelData* v );

bool b3Voxel_GetLocalBounds( const b3VoxelData* v, b3AABB* out );

int b3Voxel_QueryCells( const b3VoxelData* v, b3AABB queryLocal, b3Vec3i* out, int cap );

int b3Voxel_GetCells( const b3VoxelData* v, b3Vec3i* out, int cap );

b3MassData b3Voxel_ComputeMass( const b3VoxelData* v, float density );

void b3Voxel_RemoveCells( b3VoxelData* v, const b3Vec3i* cells, int count );

void b3Voxel_AddCells( b3VoxelData* v, const b3Vec3i* cells, int count );

void b3Voxel_AddCellsEx( b3VoxelData* v, const b3Vec3i* cells, const uint16_t* geomIndices, int count );

const b3VoxelSubBox* b3Voxel_geomBoxesFor( const b3VoxelData* v, int geomIndex, int* countOut );

int b3Voxel_cellGeomIndex( const b3VoxelData* v, b3Vec3i cell );

b3CastOutput b3RayCastVoxel( const b3VoxelData* v, const b3RayCastInput* input );

b3CastOutput b3ShapeCastVoxel( const b3VoxelData* v, const b3ShapeCastInput* input );

bool b3OverlapVoxel( const b3VoxelData* v, b3Transform xf, const b3ShapeProxy* proxy );
