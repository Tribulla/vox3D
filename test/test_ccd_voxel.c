#include "test_macros.h"

#include "voxel_shape.h" // b3ShapeCastVoxel

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/voxel.h"

#include <math.h>
#include <stdlib.h>

#define WALL_HALF 8
#define BALL_R 2

static b3VoxelData* MakeWallData( float voxelSize )
{
	b3Vec3i cells[( 2 * WALL_HALF + 1 ) * ( 2 * WALL_HALF + 1 )];
	int n = 0;
	for ( int y = -WALL_HALF; y <= WALL_HALF; ++y )
	{
		for ( int z = -WALL_HALF; z <= WALL_HALF; ++z )
		{
			cells[n++] = (b3Vec3i){ 0, y, z };
		}
	}
	return b3CreateVoxelData( cells, n, voxelSize );
}

static b3VoxelData* MakeBallData( float voxelSize )
{
	b3Vec3i cells[512];
	int n = 0;
	float rr = ( BALL_R + 0.25f ) * ( BALL_R + 0.25f );
	for ( int x = -BALL_R; x <= BALL_R; ++x )
	{
		for ( int y = -BALL_R; y <= BALL_R; ++y )
		{
			for ( int z = -BALL_R; z <= BALL_R; ++z )
			{
				if ( (float)( x * x + y * y + z * z ) <= rr )
				{
					cells[n++] = (b3Vec3i){ x, y, z };
				}
			}
		}
	}
	return b3CreateVoxelData( cells, n, voxelSize );
}

static int SphereVsVoxelWall( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* wallData = MakeWallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( wallId, &shapeDef, wallData );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -40.0f, 0.0f, 0.0f };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos finalPos = b3Body_GetPosition( ballId );
	ENSURE( 0.2f < finalPos.x && finalPos.x < 1.5f );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( wallData );
	return 0;
}

static int VoxelBulletVsVoxelWall( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* wallData = MakeWallData( voxelSize );
	b3VoxelData* ballData = MakeBallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( wallId, &shapeDef, wallData );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -120.0f, 0.0f, 0.0f };
	bodyDef.isBullet = true;
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( ballId, &shapeDef, ballData );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos finalPos = b3Body_GetPosition( ballId );
	ENSURE( finalPos.x > 0.0f );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( wallData );
	b3DestroyVoxelData( ballData );
	return 0;
}

static int VoxelBulletVsHullWall( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* ballData = MakeBallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull wallBox = b3MakeBoxHull( 0.1f, 5.0f, 5.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wallId, &shapeDef, &wallBox.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -120.0f, 0.0f, 0.0f };
	bodyDef.isBullet = true;
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( ballId, &shapeDef, ballData );

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos finalPos = b3Body_GetPosition( ballId );
	ENSURE( finalPos.x > 0.0f );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( ballData );
	return 0;
}

static int ShapeCastVoxelDirect( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* wallData = MakeWallData( voxelSize );

	b3Vec3 point = { 2.0f, 0.0f, 0.0f };
	b3ShapeCastInput input = { 0 };
	input.proxy = (b3ShapeProxy){ &point, 1, 0.5f };
	input.translation = (b3Vec3){ -4.0f, 0.0f, 0.0f };
	input.maxFraction = 1.0f;

	b3CastOutput out = b3ShapeCastVoxel( wallData, &input );
	ENSURE( out.hit );
	ENSURE( fabsf( out.fraction - 0.34375f ) < 0.01f );
	ENSURE( out.normal.x > 0.9f );

	b3Vec3 highPoint = { 2.0f, 5.0f, 0.0f };
	input.proxy = (b3ShapeProxy){ &highPoint, 1, 0.5f };
	out = b3ShapeCastVoxel( wallData, &input );
	ENSURE( out.hit == false );

	b3Vec3 farPoint = { 40.0f, 0.0f, 0.0f };
	input.proxy = (b3ShapeProxy){ &farPoint, 1, 0.5f };
	input.translation = (b3Vec3){ -80.0f, 0.0f, 0.0f };
	out = b3ShapeCastVoxel( wallData, &input );
	ENSURE( out.hit );
	ENSURE( fabsf( out.fraction - 0.4921875f ) < 0.002f );

	b3Vec3 insidePoint = { 0.0f, 0.0f, 0.0f };
	input.proxy = (b3ShapeProxy){ &insidePoint, 1, 0.5f };
	input.translation = (b3Vec3){ -2.0f, 0.0f, 0.0f };
	out = b3ShapeCastVoxel( wallData, &input );
	ENSURE( out.hit == true );
	ENSURE( out.fraction == 0.0f );

	b3DestroyVoxelData( wallData );
	return 0;
}

static b3VoxelData* MakeGroundData( float voxelSize )
{
	b3Vec3i cells[( 2 * WALL_HALF + 1 ) * ( 2 * WALL_HALF + 1 ) * 3];
	int n = 0;
	for ( int x = -WALL_HALF; x <= WALL_HALF; ++x )
	{
		for ( int z = -WALL_HALF; z <= WALL_HALF; ++z )
		{
			for ( int y = -3; y <= -1; ++y )
			{
				cells[n++] = (b3Vec3i){ x, y, z };
			}
		}
	}
	return b3CreateVoxelData( cells, n, voxelSize );
}

static int MediumSpeedImpactPenetration( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* groundData = MakeGroundData( voxelSize );
	b3VoxelData* ballData = MakeBallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.5f * voxelSize, 0.0f }; // cell y=-1 top at y=0
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( groundId, &shapeDef, groundData );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 1.5f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( ballId, &shapeDef, ballData );

	float restingY = 0.625f;
	float minY = FLT_MAX;
	for ( int step = 0; step < 60; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		float y = (float)b3Body_GetPosition( ballId ).y;
		if ( y < minY )
		{
			minY = y;
		}
	}

	ENSURE( minY > restingY - voxelSize );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( groundData );
	b3DestroyVoxelData( ballData );
	return 0;
}

static int EmbeddedBodyRecovers( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* groundData = MakeGroundData( voxelSize );
	b3VoxelData* ballData = MakeBallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.5f * voxelSize, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( groundId, &shapeDef, groundData );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.2f, 0.0f };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( ballId, &shapeDef, ballData );

	for ( int step = 0; step < 120; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float finalY = (float)b3Body_GetPosition( ballId ).y;
	ENSURE( finalY > 0.5f );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( groundData );
	b3DestroyVoxelData( ballData );
	return 0;
}

static bool CcdAnchorBase( b3Vec3i cell, void* context )
{
	(void)context;
	return cell.y <= 0;
}

static int VoxelBulletVsFractureWall( void )
{
	float voxelSize = 0.25f;
	b3VoxelData* ballData = MakeBallData( voxelSize );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableFracture( worldId, voxelSize, 0.0f );

	b3FractureTuning tuning = b3World_GetFractureTuning( worldId );
	tuning.strengthScale = 0.5f; // all four match the Teardown demo
	tuning.impactRadius = 4;
	tuning.minFragment = 2;
	tuning.warmupFrames = 0;
	b3World_SetFractureTuning( worldId, tuning );

	int n = 0;
	static b3Vec3i cells[2 * 40 * 80];
	for ( int x = 0; x < 2; ++x )
		for ( int y = 0; y < 40; ++y )
			for ( int z = -40; z < 40; ++z )
				cells[n++] = (b3Vec3i){ x, y, z };
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = CcdAnchorBase;
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -4.0f, 2.5f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 60.0f, 0.0f, 0.0f };
	bodyDef.isBullet = true;
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 8.0f;
	shapeDef.baseMaterial.friction = 0.5f;
	shapeDef.enableContactEvents = true;
	b3CreateVoxelShape( ballId, &shapeDef, ballData );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 4.5f, 2.5f, 3.0f };
	bodyDef.linearVelocity = (b3Vec3){ -60.0f, 0.0f, 0.0f };
	bodyDef.isBullet = true;
	b3BodyId mirrorId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 8.0f;
	b3VoxelData* mirrorData = MakeBallData( voxelSize );
	b3CreateVoxelShape( mirrorId, &shapeDef, mirrorData );

	int voxelsBefore = b3World_GetFractureVoxelCount( worldId );

	for ( int step = 0; step < 60; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos finalPos = b3Body_GetPosition( ballId );
	b3Pos mirrorPos = b3Body_GetPosition( mirrorId );
	ENSURE( finalPos.x < 0.0f );
	ENSURE( mirrorPos.x > 0.5f );

	ENSURE( b3World_GetFractureBodyCount( worldId ) >= 3 );
	(void)voxelsBefore;

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( ballData );
	b3DestroyVoxelData( mirrorData );
	return 0;
}

static int DebrisImpactFractures( void )
{
	float voxelSize = 0.25f;

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableFracture( worldId, voxelSize, 0.0f );

	b3FractureTuning tuning = b3World_GetFractureTuning( worldId );
	tuning.strengthScale = 0.5f; // matches the Teardown demo
	tuning.warmupFrames = 0;
	tuning.stressEnabled = false; // impact-only: isolate the impact path
	b3World_SetFractureTuning( worldId, tuning );

	static b3Vec3i slab[40 * 8 * 40];
	int n = 0;
	for ( int x = -20; x < 20; ++x )
		for ( int y = 0; y < 8; ++y )
			for ( int z = -20; z < 20; ++z )
				slab[n++] = (b3Vec3i){ x, y, z };
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = CcdAnchorBase;
	b3World_CreateFractureVoxels( worldId, slab, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );

	int before = b3World_GetFractureBodyCount( worldId );

	static b3Vec3i block[12 * 12 * 12];
	n = 0;
	for ( int x = 0; x < 12; ++x )
		for ( int y = 0; y < 12; ++y )
			for ( int z = 0; z < 12; ++z )
				block[n++] = (b3Vec3i){ x, 56 + y, z };
	b3World_CreateFractureVoxels( worldId, block, n, b3GetFractureMaterial( b3_fractureStone ), NULL );

	int afterSpawn = b3World_GetFractureBodyCount( worldId );

	for ( int step = 0; step < 180; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	int final = b3World_GetFractureBodyCount( worldId );
	ENSURE( final > afterSpawn );
	(void)before;

	b3DestroyWorld( worldId );
	return 0;
}

int CcdVoxelTest( void )
{
	RUN_SUBTEST( ShapeCastVoxelDirect );
	RUN_SUBTEST( SphereVsVoxelWall );
	RUN_SUBTEST( VoxelBulletVsVoxelWall );
	RUN_SUBTEST( VoxelBulletVsHullWall );
	RUN_SUBTEST( MediumSpeedImpactPenetration );
	RUN_SUBTEST( EmbeddedBodyRecovers );
	RUN_SUBTEST( VoxelBulletVsFractureWall );
	RUN_SUBTEST( DebrisImpactFractures );
	return 0;
}
