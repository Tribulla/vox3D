#include "box3d/box3d.h"

#include "voxel_shape.h" // internal query layer

#include "test_macros.h"

#include <math.h>

static int VoxelBuildAndPointQuery( void )
{
	b3Vec3i cells[] = {
		{ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 5, 5, 5 }, { -1, -1, -1 }, { 16, 0, 0 },
	};
	b3VoxelData* v = b3CreateVoxelData( cells, ARRAY_COUNT( cells ), 1.0f );
	ENSURE( v != NULL );
	ENSURE( b3VoxelData_GetCellCount( v ) == 6 );
	ENSURE( b3VoxelData_GetVoxelSize( v ) == 1.0f );

	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ 0, 0, 0 } ) );
	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ 1, 0, 0 } ) );
	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ -1, -1, -1 } ) ); // negative chunk
	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ 16, 0, 0 } ) );   // chunk 1
	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ 2, 0, 0 } ) == false );
	ENSURE( b3VoxelData_IsSolid( v, ( b3Vec3i ){ 15, 0, 0 } ) == false ); // in chunk 0 but empty

	b3AABB b = b3VoxelData_GetBounds( v );
	ENSURE_SMALL( b.lowerBound.x - ( -1.5f ), 1e-6f );
	ENSURE_SMALL( b.lowerBound.y - ( -1.5f ), 1e-6f );
	ENSURE_SMALL( b.lowerBound.z - ( -1.5f ), 1e-6f );
	ENSURE_SMALL( b.upperBound.x - ( 16.5f ), 1e-6f );
	ENSURE_SMALL( b.upperBound.y - ( 5.5f ), 1e-6f );
	ENSURE_SMALL( b.upperBound.z - ( 5.5f ), 1e-6f );

	b3DestroyVoxelData( v );
	return 0;
}

static int VoxelDedup( void )
{
	b3Vec3i cells[] = { { 0, 0, 0 }, { 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0 } };
	b3VoxelData* v = b3CreateVoxelData( cells, ARRAY_COUNT( cells ), 1.0f );
	ENSURE( v != NULL );
	ENSURE( b3VoxelData_GetCellCount( v ) == 2 );
	b3DestroyVoxelData( v );
	return 0;
}

static bool QueryHas( const b3VoxelData* v, b3AABB q, b3Vec3i want )
{
	b3Vec3i out[64];
	int n = b3Voxel_QueryCells( v, q, out, 64 );
	for ( int i = 0; i < n; ++i )
		if ( out[i].x == want.x && out[i].y == want.y && out[i].z == want.z )
			return true;
	return false;
}

static int VoxelRangeQuery( void )
{
	b3Vec3i cells[] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
	b3VoxelData* v = b3CreateVoxelData( cells, ARRAY_COUNT( cells ), 1.0f );
	b3Vec3i out[64];

	b3AABB q0 = { { -0.4f, -0.4f, -0.4f }, { 0.4f, 0.4f, 0.4f } };
	ENSURE( b3Voxel_QueryCells( v, q0, out, 64 ) == 1 );
	ENSURE( QueryHas( v, q0, ( b3Vec3i ){ 0, 0, 0 } ) );

	b3AABB q1 = { { -0.4f, -0.4f, -0.4f }, { 1.4f, 0.4f, 0.4f } };
	ENSURE( b3Voxel_QueryCells( v, q1, out, 64 ) == 2 );
	ENSURE( QueryHas( v, q1, ( b3Vec3i ){ 0, 0, 0 } ) );
	ENSURE( QueryHas( v, q1, ( b3Vec3i ){ 1, 0, 0 } ) );

	b3AABB qall = { { -10, -10, -10 }, { 10, 10, 10 } };
	ENSURE( b3Voxel_QueryCells( v, qall, out, 64 ) == 3 );

	b3AABB qfar = { { 100, 100, 100 }, { 101, 101, 101 } };
	ENSURE( b3Voxel_QueryCells( v, qfar, out, 64 ) == 0 );

	b3DestroyVoxelData( v );
	return 0;
}

static int VoxelFaceBoundary( void )
{
	b3Vec3i cells[] = { { 0, 0, 0 } };
	b3VoxelData* v = b3CreateVoxelData( cells, 1, 1.0f );
	b3Vec3i out[8];

	b3AABB inside = { { 0.49f, -0.1f, -0.1f }, { 0.6f, 0.1f, 0.1f } };
	ENSURE( b3Voxel_QueryCells( v, inside, out, 8 ) == 1 );

	b3AABB touch = { { 0.5f, -0.1f, -0.1f }, { 0.6f, 0.1f, 0.1f } };
	ENSURE( b3Voxel_QueryCells( v, touch, out, 8 ) == 1 );

	b3AABB past = { { 0.51f, -0.1f, -0.1f }, { 0.6f, 0.1f, 0.1f } };
	ENSURE( b3Voxel_QueryCells( v, past, out, 8 ) == 0 );

	b3DestroyVoxelData( v );
	return 0;
}

static int VoxelScaledSize( void )
{
	b3Vec3i cells[] = { { 0, 0, 0 }, { 1, 0, 0 } };
	b3VoxelData* v = b3CreateVoxelData( cells, ARRAY_COUNT( cells ), 2.0f );
	b3Vec3i out[8];

	b3AABB b = b3VoxelData_GetBounds( v );
	ENSURE_SMALL( b.lowerBound.x - ( -1.0f ), 1e-6f ); // cell 0 min = -1
	ENSURE_SMALL( b.upperBound.x - ( 3.0f ), 1e-6f );  // cell 1 max = 3

	b3AABB q = { { 1.5f, -0.5f, -0.5f }, { 2.5f, 0.5f, 0.5f } };
	ENSURE( b3Voxel_QueryCells( v, q, out, 8 ) == 1 );
	ENSURE( out[0].x == 1 );

	b3DestroyVoxelData( v );
	return 0;
}

int VoxelTest( void )
{
	RUN_SUBTEST( VoxelBuildAndPointQuery );
	RUN_SUBTEST( VoxelDedup );
	RUN_SUBTEST( VoxelRangeQuery );
	RUN_SUBTEST( VoxelFaceBoundary );
	RUN_SUBTEST( VoxelScaledSize );
	return 0;
}
