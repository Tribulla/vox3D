#include "box3d/box3d.h"

#include "test_macros.h"

#include <math.h>

static b3WorldId FractureMakeWorld( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World_EnableFracture( worldId, 1.0f, 0.0f );
	return worldId;
}

static void FractureMakeGround( b3WorldId worldId )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
	b3BodyId g = b3CreateBody( worldId, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3BoxHull hull = b3MakeBoxHull( 200.0f, 1.0f, 200.0f );
	b3CreateHullShape( g, &sd, &hull.base );
}

static void FractureFireBall( b3WorldId worldId, b3Vec3 center, b3Vec3 velocity, float radius )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = b3ToPos( center );
	bd.linearVelocity = velocity;
	bd.isBullet = true;
	b3BodyId body = b3CreateBody( worldId, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density *= 4.0f; // matches Sample::MouseDown's shot
	sd.enableContactEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
	b3CreateSphereShape( body, &sd, &sphere );
}

static int FractureBoxCells( b3Vec3i* out, int nx, int ny, int nz, int ox, int oy, int oz )
{
	int n = 0;
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
				out[n++] = ( b3Vec3i ){ ox + x, oy + y, oz + z };
	return n;
}

static void FractureStepN( b3WorldId worldId, int n )
{
	for ( int i = 0; i < n; ++i )
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
}

static bool FractureBounded( b3WorldId worldId )
{
	b3AABB b = b3World_GetBounds( worldId );
	float v[6] = { b.lowerBound.x, b.lowerBound.y, b.lowerBound.z, b.upperBound.x, b.upperBound.y, b.upperBound.z };
	for ( int i = 0; i < 6; ++i )
		if ( !isfinite( v[i] ) || fabsf( v[i] ) > 1.0e4f )
			return false;
	return true;
}

static bool AnchorLowX( b3Vec3i cell, void* context )
{
	return cell.x < *(int*)context;
}

static int FractureRestAndShatter( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3Vec3i cells[22 * 10 * 2];
	int n = FractureBoxCells( cells, 22, 10, 2, -11, 0, -1 );
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureBrick ), NULL );

	int p0 = b3World_GetFractureBodyCount( worldId );
	FractureStepN( worldId, 120 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) == p0 );
	ENSURE( FractureBounded( worldId ) );

	FractureFireBall( worldId, ( b3Vec3 ){ 0.0f, 5.0f, 18.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -100.0f }, 0.25f );
	FractureStepN( worldId, 150 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) > p0 + 1 );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureCantileverSnaps( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3Vec3i cells[26 * 4 * 4];
	int n = FractureBoxCells( cells, 26, 4, 4, -2, 14, -2 );
	int anchorThresh = 0; // anchor cells with x < 0 (the two columns above origin x=-2)
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );

	int p0 = b3World_GetFractureBodyCount( worldId );
	bool snapped = false;
	for ( int s = 0; s < 150 && !snapped; ++s )
	{
		FractureStepN( worldId, 1 );
		if ( b3World_GetFractureBodyCount( worldId ) > p0 )
			snapped = true;
		ENSURE( FractureBounded( worldId ) );
	}
	ENSURE( snapped );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureBeamSnapNormalBody( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3Vec3i cells[24 * 3 * 3];
	int n = FractureBoxCells( cells, 24, 3, 3, -3, 12, -1 );
	int anchorThresh = -1; // anchor cells with x < -1
	b3FractureDef def = b3DefaultFractureDef();
	def.merge = true; // a solid ("normal") rigid body
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );

	int p0 = b3World_GetFractureBodyCount( worldId );
	bool snapped = false;
	for ( int s = 0; s < 200 && !snapped; ++s )
	{
		FractureStepN( worldId, 1 );
		if ( b3World_GetFractureBodyCount( worldId ) > p0 )
			snapped = true;
	}
	ENSURE( snapped );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureStressField( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3Vec3i cells[26 * 4 * 4];
	int n = FractureBoxCells( cells, 26, 4, 4, -2, 14, -2 );
	int anchorThresh = 0;
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.fractureEnabled = false; // observe the field without it snapping
	b3World_SetFractureTuning( worldId, t );
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );
	FractureStepN( worldId, 25 );
	ENSURE( b3World_GetFractureMaxStress( worldId ) > 0.0f );
	ENSURE( isfinite( b3World_GetFractureMaxStress( worldId ) ) );
	b3DestroyWorld( worldId );

	b3WorldId w2 = FractureMakeWorld();
	FractureMakeGround( w2 );
	b3Vec3i c2[3 * 3 * 3];
	int n2 = FractureBoxCells( c2, 3, 3, 3, 0, 20, 0 );
	b3World_CreateFractureVoxels( w2, c2, n2, b3GetFractureMaterial( b3_fractureMetal ), NULL );
	FractureStepN( w2, 15 );
	ENSURE( b3World_GetFractureMaxStress( w2 ) == 0.0f );
	b3DestroyWorld( w2 );
	return 0;
}

static int FractureRestingBeamHolds( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.position = ( b3Pos ){ 0.0f, 3.0f, 0.0f };
	b3BodyId plat = b3CreateBody( worldId, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3BoxHull hull = b3MakeBoxHull( 12.0f, 3.0f, 6.0f );
	b3CreateHullShape( plat, &sd, &hull.base );

	b3Vec3i cells[20 * 3 * 3];
	int n = FractureBoxCells( cells, 20, 3, 3, -10, 6, -1 );
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), NULL );

	int p0 = b3World_GetFractureBodyCount( worldId );
	FractureStepN( worldId, 120 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) == p0 );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureBridgeHoldsThenBreaks( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3FractureMaterial stone = b3GetFractureMaterial( b3_fractureStone );
	b3FractureMaterial concrete = b3GetFractureMaterial( b3_fractureConcrete );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ -8.0f, 4.0f, 0.0f }, ( b3Vec3 ){ 2.0f, 4.0f, 4.0f }, stone, NULL );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ 8.0f, 4.0f, 0.0f }, ( b3Vec3 ){ 2.0f, 4.0f, 4.0f }, stone, NULL );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ 0.0f, 10.5f, 0.0f }, ( b3Vec3 ){ 10.0f, 2.5f, 4.0f }, concrete, NULL );

	int p0 = b3World_GetFractureBodyCount( worldId );
	FractureStepN( worldId, 60 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) == p0 );
	ENSURE( FractureBounded( worldId ) );

	FractureFireBall( worldId, ( b3Vec3 ){ 0.0f, 34.0f, 0.0f }, ( b3Vec3 ){ 0.0f, -34.0f, 0.0f }, 0.6f );
	FractureStepN( worldId, 150 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) > p0 );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureConvexRests( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3HullData* hull = b3CreateCylinder( 6.0f, 2.0f, 0.0f, 12 ); // height, radius, yOffset, sides
	b3AABB aabb = b3ComputeHullAABB( hull, ( b3Transform ){ b3Vec3_zero, b3Quat_identity } );
	b3WorldTransform xf = { { 0.0f, -aabb.lowerBound.y, 0.0f }, b3Quat_identity };
	b3World_CreateFractureConvex( worldId, hull, xf, 16, b3GetFractureMaterial( b3_fractureStone ), NULL );
	b3DestroyHull( hull );

	int p0 = b3World_GetFractureBodyCount( worldId );
	ENSURE( p0 == 1 );
	FractureStepN( worldId, 120 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) == 1 );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureConvexShatters( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3HullData* hull = b3CreateCylinder( 8.0f, 2.0f, 0.0f, 12 );
	b3AABB aabb = b3ComputeHullAABB( hull, ( b3Transform ){ b3Vec3_zero, b3Quat_identity } );
	b3WorldTransform xf = { { 0.0f, -aabb.lowerBound.y, 0.0f }, b3Quat_identity };
	b3World_CreateFractureConvex( worldId, hull, xf, 24, b3GetFractureMaterial( b3_fractureStone ), NULL );
	b3DestroyHull( hull );

	int p0 = b3World_GetFractureBodyCount( worldId );
	FractureStepN( worldId, 40 ); // settle past warmup
	ENSURE( b3World_GetFractureBodyCount( worldId ) == p0 );

	FractureFireBall( worldId, ( b3Vec3 ){ 0.0f, 4.0f, 18.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -110.0f }, 0.5f );
	FractureStepN( worldId, 150 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) > p0 );
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureConvertBody( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = ( b3Pos ){ 0.0f, 2.0f, 0.0f };
	b3BodyId body = b3CreateBody( worldId, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3BoxHull h = b3MakeBoxHull( 2.0f, 2.0f, 2.0f );
	b3CreateHullShape( body, &sd, &h.base );

	ENSURE( b3World_GetFractureBodyCount( worldId ) == 0 );
	int piece = b3World_MakeBodyFracture( worldId, body, b3GetFractureMaterial( b3_fractureConcrete ), NULL );
	ENSURE( piece >= 0 );
	ENSURE( b3Body_IsValid( body ) == false );			 // original replaced
	ENSURE( b3World_GetFractureBodyCount( worldId ) == 1 ); // now destructible

	FractureStepN( worldId, 60 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) == 1 ); // rests intact
	FractureFireBall( worldId, ( b3Vec3 ){ 0.0f, 2.0f, 18.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -110.0f }, 0.5f );
	FractureStepN( worldId, 120 );
	ENSURE( b3World_GetFractureBodyCount( worldId ) > 1 ); // shatters on impact
	ENSURE( FractureBounded( worldId ) );
	b3DestroyWorld( worldId );
	return 0;
}

int FractureTest( void )
{
	RUN_SUBTEST( FractureConvexRests );
	RUN_SUBTEST( FractureConvexShatters );
	RUN_SUBTEST( FractureConvertBody );
	RUN_SUBTEST( FractureRestAndShatter );
	RUN_SUBTEST( FractureCantileverSnaps );
	RUN_SUBTEST( FractureBeamSnapNormalBody );
	RUN_SUBTEST( FractureStressField );
	RUN_SUBTEST( FractureRestingBeamHolds );
	RUN_SUBTEST( FractureBridgeHoldsThenBreaks );
	return 0;
}
