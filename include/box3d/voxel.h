#pragma once

#include "base.h"
#include "id.h"
#include "math_functions.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3VoxelData b3VoxelData;

typedef struct b3VoxelSubBox
{
	b3Vec3 center;
	b3Vec3 halfExtents;
} b3VoxelSubBox;

B3_API b3VoxelData* b3CreateVoxelData( const b3Vec3i* cells, int count, float voxelSize );

B3_API b3VoxelData* b3CreateVoxelDataEx( const b3Vec3i* cells, const uint16_t* geomIndices, int count,
										 float voxelSize );

B3_API int b3VoxelData_AddGeometry( b3VoxelData* voxels, const b3VoxelSubBox* boxes, int count );

B3_API uint16_t b3VoxelData_GetCellGeometry( const b3VoxelData* voxels, b3Vec3i cell );

B3_API void b3DestroyVoxelData( b3VoxelData* voxels );

B3_API int b3VoxelData_GetCellCount( const b3VoxelData* voxels );

B3_API float b3VoxelData_GetVoxelSize( const b3VoxelData* voxels );

B3_API b3AABB b3VoxelData_GetBounds( const b3VoxelData* voxels );

B3_API bool b3VoxelData_IsSolid( const b3VoxelData* voxels, b3Vec3i cell );

B3_API int b3VoxelData_GetCells( const b3VoxelData* voxels, b3Vec3i* cells, int capacity );

B3_API void b3VoxelShape_AddCells( b3ShapeId shapeId, const b3Vec3i* cells, const uint16_t* geomIndices,
								   int count );

B3_API void b3VoxelShape_RemoveCells( b3ShapeId shapeId, const b3Vec3i* cells, int count );

/**@}*/
