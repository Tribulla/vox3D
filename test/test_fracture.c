#include "box3d/box3d.h"

#include "test_macros.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

static int FractureConvexCantileverSnaps( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3BoxHull beam = b3MakeBoxHull( 8.0f, 0.5f, 0.5f );
	int anchorThresh = -4; // anchor chunks whose local centroid x < -4 (one end of the beam)
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3WorldTransform xf = { { 0.0f, 8.0f, 0.0f }, b3Quat_identity };
	b3World_CreateFractureConvex( worldId, &beam.base, xf, 24, b3GetFractureMaterial( b3_fractureStone ), &def );

	int p0 = b3World_GetFractureBodyCount( worldId );
	ENSURE( p0 == 1 );
	bool snapped = false;
	for ( int s = 0; s < 200 && !snapped; ++s )
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

static int FractureStrideCantileverSnaps( void )
{
	b3WorldId worldId = FractureMakeWorld();
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.analysisStride = 3;
	b3World_SetFractureTuning( worldId, t );
	FractureMakeGround( worldId );
	b3Vec3i cells[26 * 4 * 4];
	int n = FractureBoxCells( cells, 26, 4, 4, -2, 14, -2 );
	int anchorThresh = 0;
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureConcrete ), &def );

	int p0 = b3World_GetFractureBodyCount( worldId );
	bool snapped = false;
	for ( int s = 0; s < 250 && !snapped; ++s )
	{
		FractureStepN( worldId, 1 );
		if ( b3World_GetFractureBodyCount( worldId ) > p0 )
			snapped = true;
		ENSURE( FractureBounded( worldId ) );
	}
	ENSURE( snapped );
	ENSURE( b3World_GetFractureMaxStress( worldId ) >= 0.0f );
	ENSURE( isfinite( b3World_GetFractureMaxStress( worldId ) ) );
	b3DestroyWorld( worldId );
	return 0;
}

static int FractureParallelCantileverSnaps( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	wd.workerCount = 4;
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World_EnableFracture( worldId, 1.0f, 0.0f );
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.parallelAnalysis = true;
	b3World_SetFractureTuning( worldId, t );
	FractureMakeGround( worldId );

	b3BoxHull beam = b3MakeBoxHull( 8.0f, 0.5f, 0.5f );
	int anchorThresh = -4;
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	b3WorldTransform xf = { { 0.0f, 8.0f, 0.0f }, b3Quat_identity };
	b3World_CreateFractureConvex( worldId, &beam.base, xf, 24, b3GetFractureMaterial( b3_fractureStone ), &def );
	// and a voxel tower so the parallel path covers both kinds
	b3Vec3i cells[3 * 8 * 3];
	int n = FractureBoxCells( cells, 3, 8, 3, 8, 0, 8 );
	b3World_CreateFractureVoxels( worldId, cells, n, b3GetFractureMaterial( b3_fractureBrick ), NULL );

	int p0 = b3World_GetFractureBodyCount( worldId );
	ENSURE( p0 == 2 );
	bool snapped = false;
	for ( int s = 0; s < 200 && !snapped; ++s )
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

static int FractureDebrisBudget( void )
{
	int counts[2] = { 0, 0 };
	for ( int pass = 0; pass < 2; ++pass )
	{
		b3WorldId w = FractureMakeWorld();
		if ( pass == 1 )
		{
			b3FractureTuning t = b3World_GetFractureTuning( w );
			t.maxDebris = 3;
			b3World_SetFractureTuning( w, t );
		}
		FractureMakeGround( w );
		b3HullData* hull = b3CreateCylinder( 8.0f, 2.0f, 0.0f, 12 );
		b3AABB aabb = b3ComputeHullAABB( hull, ( b3Transform ){ b3Vec3_zero, b3Quat_identity } );
		b3WorldTransform xf = { { 0.0f, -aabb.lowerBound.y, 0.0f }, b3Quat_identity };
		b3World_CreateFractureConvex( w, hull, xf, 24, b3GetFractureMaterial( b3_fractureStone ), NULL );
		b3DestroyHull( hull );

		FractureStepN( w, 40 );
		FractureFireBall( w, ( b3Vec3 ){ 0.0f, 4.0f, 18.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -110.0f }, 0.5f );
		FractureStepN( w, 150 );
		counts[pass] = b3World_GetFractureBodyCount( w );
		ENSURE( FractureBounded( w ) );
		b3DestroyWorld( w );
	}
	ENSURE( counts[0] > 1 );		   // it shattered
	ENSURE( counts[1] < counts[0] ); // the cap removed rubble
	ENSURE( counts[1] >= 1 );
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

typedef struct DomCollect
{
	b3BodyId ids[1024];
	int count;
} DomCollect;

static bool CollectBodiesCb( b3ShapeId shapeId, void* ctx )
{
	DomCollect* c = (DomCollect*)ctx;
	b3BodyId b = b3Shape_GetBody( shapeId );
	for ( int i = 0; i < c->count; ++i )
		if ( c->ids[i].index1 == b.index1 && c->ids[i].world0 == b.world0 && c->ids[i].generation == b.generation )
			return true;
	if ( c->count < 1024 )
		c->ids[c->count++] = b;
	return true;
}

static void DominoBuild( b3WorldId worldId )
{
	b3BoxHull box = b3MakeBoxHull( 0.125f, 0.5f, 0.25f );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.baseMaterial.friction = 0.6f;
	sd.density = 4.0f;
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	int count = 15;
	float x = -0.5f * count;
	for ( int i = 0; i < count; ++i )
	{
		bd.position = ( b3Pos ){ x, 0.5f, 0.0f };
		b3BodyId body = b3CreateBody( worldId, &bd );
		b3CreateHullShape( body, &sd, &box.base );
		if ( i == 0 )
			b3Body_ApplyLinearImpulse( body, ( b3Vec3 ){ 0.2f, 0.0f, 0.0f }, ( b3Pos ){ x, 1.0f, 0.0f }, true );
		x += 1.01f;
	}
}

static int DominoToppledCount( b3WorldId worldId )
{
	DomCollect c = { 0 };
	b3AABB huge = { { -1.0e4f, -1.0e4f, -1.0e4f }, { 1.0e4f, 1.0e4f, 1.0e4f } };
	b3World_OverlapAABB( worldId, huge, b3DefaultQueryFilter(), CollectBodiesCb, &c );
	int toppled = 0;
	for ( int i = 0; i < c.count; ++i )
	{
		if ( b3Body_GetType( c.ids[i] ) != b3_dynamicBody )
			continue;
		b3Quat q = b3Body_GetRotation( c.ids[i] );
		float upY = 1.0f - 2.0f * ( q.v.x * q.v.x + q.v.z * q.v.z ); // (R*{0,1,0}).y
		if ( upY < 0.5f )
			toppled++;
	}
	return toppled;
}

static int FractureDestructibleDominoesTopple( void )
{
	b3WorldId solid = FractureMakeWorld();
	FractureMakeGround( solid );
	DominoBuild( solid );
	FractureStepN( solid, 400 );
	int solidToppled = DominoToppledCount( solid );
	b3DestroyWorld( solid );
	ENSURE( solidToppled >= 5 );

	b3WorldId frac = FractureMakeWorld();
	FractureMakeGround( frac );
	DominoBuild( frac );
	{
		DomCollect c = { 0 };
		b3AABB huge = { { -1.0e4f, -1.0e4f, -1.0e4f }, { 1.0e4f, 1.0e4f, 1.0e4f } };
		b3World_OverlapAABB( frac, huge, b3DefaultQueryFilter(), CollectBodiesCb, &c );
		for ( int i = 0; i < c.count; ++i )
			if ( b3Body_GetType( c.ids[i] ) == b3_dynamicBody )
				b3World_MakeBodyFracture( frac, c.ids[i], b3GetFractureMaterial( b3_fractureStone ), NULL );
	}
	FractureStepN( frac, 400 );
	int fracToppled = DominoToppledCount( frac );
	ENSURE( FractureBounded( frac ) );
	b3DestroyWorld( frac );

	ENSURE( fracToppled >= 5 );
	return 0;
}

static int FractureConvertPreservesMass( void )
{
	b3WorldId w = FractureMakeWorld();
	FractureMakeGround( w );

	b3BoxHull block = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = ( b3Pos ){ 0.0f, 1.0f, 0.0f };
	b3BodyId body = b3CreateBody( w, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 200.0f; // heavy, like an Arch block
	b3CreateHullShape( body, &sd, &block.base );
	float massBefore = b3Body_GetMass( body );

	b3World_MakeBodyFracture( w, body, b3GetFractureMaterial( b3_fractureStone ), NULL );

	DomCollect c = { 0 };
	b3AABB huge = { { -1.0e4f, -1.0e4f, -1.0e4f }, { 1.0e4f, 1.0e4f, 1.0e4f } };
	b3World_OverlapAABB( w, huge, b3DefaultQueryFilter(), CollectBodiesCb, &c );
	float massAfter = 0.0f;
	for ( int i = 0; i < c.count; ++i )
		if ( b3Body_GetType( c.ids[i] ) == b3_dynamicBody )
			massAfter = b3Body_GetMass( c.ids[i] );
	ENSURE( massAfter > 0.9f * massBefore && massAfter < 1.1f * massBefore ); // mass preserved

	int p0 = b3World_GetFractureBodyCount( w );
	FractureStepN( w, 90 );
	ENSURE( b3World_GetFractureBodyCount( w ) == p0 ); // rests intact, does not self-crumble
	ENSURE( FractureBounded( w ) );
	b3DestroyWorld( w );
	return 0;
}

static int FractureConvertPreservesGravityScale( void )
{
	b3WorldId w = FractureMakeWorld();
	FractureMakeGround( w );

	b3BoxHull box = b3MakeBoxHull( 0.35f, 0.08f, 0.5f ); // a "book"
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = ( b3Pos ){ 0.0f, 5.0f, 0.0f };
	bd.gravityScale = 0.0f;						  // floats
	bd.angularVelocity = ( b3Vec3 ){ 0.01f, 5.0f, 0.01f }; // spinning
	b3BodyId body = b3CreateBody( w, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3CreateHullShape( body, &sd, &box.base );

	b3World_MakeBodyFracture( w, body, b3GetFractureMaterial( b3_fractureStone ), NULL );
	FractureStepN( w, 120 ); // 2 seconds; would fall ~20 m under gravity if gravityScale were lost

	DomCollect c = { 0 };
	b3AABB huge = { { -1.0e4f, -1.0e4f, -1.0e4f }, { 1.0e4f, 1.0e4f, 1.0e4f } };
	b3World_OverlapAABB( w, huge, b3DefaultQueryFilter(), CollectBodiesCb, &c );
	int dyn = 0;
	float minY = 1.0e9f, maxAng = 0.0f;
	for ( int i = 0; i < c.count; ++i )
		if ( b3Body_GetType( c.ids[i] ) == b3_dynamicBody )
		{
			dyn++;
			float y = b3Body_GetWorldCenter( c.ids[i] ).y;
			if ( y < minY )
				minY = y;
			float ang = b3Length( b3Body_GetAngularVelocity( c.ids[i] ) );
			if ( ang > maxAng )
				maxAng = ang;
		}
	ENSURE( dyn >= 1 );
	ENSURE( minY > 4.0f );	// still floating near y = 5, not fallen to the ground
	ENSURE( maxAng > 1.0f ); // still spinning (angular velocity preserved too)
	b3DestroyWorld( w );
	return 0;
}

static int FractureConvertCompound( void )
{
	b3WorldId w = FractureMakeWorld();
	FractureMakeGround( w );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = ( b3Pos ){ 0.0f, 4.0f, 0.0f };
	bd.gravityScale = 0.0f; // floats, like the Gyroscopic Torque demo
	b3BodyId body = b3CreateBody( w, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3HullData* cyl = b3CreateCylinder( 1.2f, 0.4f, 0.0f, 16 );
	b3BoxHull box = b3MakeBoxHull( 2.0f, 0.1f, 0.2f );
	b3CreateHullShape( body, &sd, cyl );
	b3CreateHullShape( body, &sd, &box.base );
	b3DestroyHull( cyl );
	ENSURE( b3Body_GetShapeCount( body ) == 2 );

	int piece = b3World_MakeBodyFracture( w, body, b3GetFractureMaterial( b3_fractureStone ), NULL );
	ENSURE( piece >= 0 );							   // compound body IS converted now
	ENSURE( b3World_GetFractureBodyCount( w ) == 1 ); // as a single destructible body

	DomCollect c = { 0 };
	b3AABB huge = { { -1.0e4f, -1.0e4f, -1.0e4f }, { 1.0e4f, 1.0e4f, 1.0e4f } };
	b3World_OverlapAABB( w, huge, b3DefaultQueryFilter(), CollectBodiesCb, &c );
	b3BodyId conv = c.ids[0];
	for ( int i = 0; i < c.count; ++i )
		if ( b3Body_GetType( c.ids[i] ) == b3_dynamicBody )
			conv = c.ids[i];
	ENSURE( b3Body_GetShapeCount( conv ) == 2 ); // BOTH shapes preserved, nothing dropped

	FractureStepN( w, 60 );
	ENSURE( b3World_GetFractureBodyCount( w ) == 1 );		   // stays whole while intact
	ENSURE( b3Body_GetWorldCenter( conv ).y > 3.0f );	   // still floating (gravityScale kept)

	FractureFireBall( w, ( b3Vec3 ){ 0.0f, 4.0f, 12.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -140.0f }, 0.5f );
	FractureStepN( w, 120 );
	ENSURE( b3World_GetFractureBodyCount( w ) > 1 ); // a hard hit breaks it into its pieces
	ENSURE( FractureBounded( w ) );
	b3DestroyWorld( w );
	return 0;
}

static int FractureDynamicSpanSnapsUnderSelfWeight( void )
{
	b3WorldId worldId = FractureMakeWorld();
	FractureMakeGround( worldId );
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.strengthScale = 0.2f; // weak enough that self-weight bending exceeds the span's strength
	b3World_SetFractureTuning( worldId, t );
	b3FractureMaterial stone = b3GetFractureMaterial( b3_fractureStone );
	b3FractureMaterial concrete = b3GetFractureMaterial( b3_fractureConcrete );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ -9.0f, 3.0f, 0.0f }, ( b3Vec3 ){ 2.0f, 3.0f, 4.0f }, stone, NULL );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ 9.0f, 3.0f, 0.0f }, ( b3Vec3 ){ 2.0f, 3.0f, 4.0f }, stone, NULL );
	b3World_CreateFractureBox( worldId, ( b3Vec3 ){ 0.0f, 7.0f, 0.0f }, ( b3Vec3 ){ 12.0f, 1.0f, 4.0f }, concrete, NULL );

	int p0 = b3World_GetFractureBodyCount( worldId );
	bool snapped = false;
	for ( int s = 0; s < 200 && !snapped; ++s )
	{
		FractureStepN( worldId, 1 );
		if ( b3World_GetFractureBodyCount( worldId ) > p0 )
			snapped = true;
		ENSURE( FractureBounded( worldId ) );
	}
	ENSURE( snapped ); // static self-weight stress fractured a settled dynamic body
	b3DestroyWorld( worldId );
	return 0;
}

static b3BodyId MakeVoxelShapeBody( b3WorldId worldId, bool dynamic, b3Vec3 pos, const b3Vec3i* cells, int n,
									b3VoxelData** outVd )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = dynamic ? b3_dynamicBody : b3_staticBody;
	bd.position = b3ToPos( pos );
	b3BodyId body = b3CreateBody( worldId, &bd );
	b3VoxelData* vd = b3CreateVoxelData( cells, n, 1.0f );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;
	sd.enableContactEvents = true; // the fracture engine gathers contact forces from these
	b3CreateVoxelShape( body, &sd, vd );
	*outVd = vd;
	return body;
}

static int VoxelShapeImpactFractureEmitsEvents( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, 0.0f, 0.0f }; // in space: only the impact fractures the hull
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World_EnableFracture( worldId, 1.0f, 0.0f );
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.warmupFrames = 2; // no settling scene to wait on
	b3World_SetFractureTuning( worldId, t );

	b3Vec3i cells[15 * 10 * 2];
	int n = FractureBoxCells( cells, 15, 10, 2, -7, -5, -1 );
	b3VoxelData* vd = NULL;
	b3BodyId ship = MakeVoxelShapeBody( worldId, true, ( b3Vec3 ){ 0.0f, 0.0f, 0.0f }, cells, n, &vd );

	int piece = b3World_MakeVoxelBodyFracture( worldId, ship, b3GetFractureMaterial( b3_fractureGlass ), NULL );
	ENSURE( piece >= 0 );
	int cellsBefore = b3VoxelData_GetCellCount( vd );
	ENSURE( cellsBefore == n );

	FractureFireBall( worldId, ( b3Vec3 ){ 0.0f, 0.0f, 9.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -140.0f }, 0.6f );

	bool got = false;
	for ( int s = 0; s < 90 && !got; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3FractureEvents ev = b3World_GetFractureEvents( worldId );
		for ( int e = 0; e < ev.count; ++e )
		{
			const b3FractureEvent* fe = ev.events + e;
			ENSURE( fe->cellCount > 0 );
			ENSURE( fe->cells != NULL );
			ENSURE( fe->mass > 0.0f );
			ENSURE( B3_IS_NULL( fe->fragmentBody ) );				 // host-materialised, no spawned body
			ENSURE( B3_ID_EQUALS( fe->parentBody, ship ) );
			ENSURE( fe->reason == b3_fractureImpact );
			ENSURE( isfinite( fe->centerOfMassWorld.x ) && isfinite( fe->centerOfMassWorld.y ) );
			ENSURE( isfinite( fe->linearVelocity.z ) && isfinite( fe->angularVelocity.x ) );
			for ( int i = 0; i < fe->cellCount; ++i )
			{
				b3Vec3i c = fe->cells[i];
				ENSURE( c.x >= -7 && c.x <= 7 && c.y >= -5 && c.y <= 4 && c.z >= -1 && c.z <= 0 );
				ENSURE( b3VoxelData_IsSolid( vd, c ) == false );
			}
			got = true;
		}
	}
	ENSURE( got );
	ENSURE( b3VoxelData_GetCellCount( vd ) < cellsBefore ); // the collider lost cells
	ENSURE( FractureBounded( worldId ) );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( vd );
	return 0;
}

static int VoxelShapeStressFractureEmitsEvents( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World_EnableFracture( worldId, 1.0f, 0.0f );

	b3Vec3i cells[26 * 4 * 4];
	int n = FractureBoxCells( cells, 26, 4, 4, -2, 14, -2 );
	b3VoxelData* vd = NULL;
	b3BodyId beam = MakeVoxelShapeBody( worldId, false, ( b3Vec3 ){ 0.0f, 0.0f, 0.0f }, cells, n, &vd );

	int anchorThresh = 0; // anchor the two cell columns with x < 0
	b3FractureDef def = b3DefaultFractureDef();
	def.anchor = AnchorLowX;
	def.anchorContext = &anchorThresh;
	int piece = b3World_MakeVoxelBodyFracture( worldId, beam, b3GetFractureMaterial( b3_fractureConcrete ), &def );
	ENSURE( piece >= 0 );
	int cellsBefore = b3VoxelData_GetCellCount( vd );

	bool got = false;
	b3FractureReason reason = b3_fractureImpact;
	for ( int s = 0; s < 200 && !got; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3FractureEvents ev = b3World_GetFractureEvents( worldId );
		if ( ev.count > 0 )
		{
			const b3FractureEvent* fe = ev.events + 0;
			reason = fe->reason;
			ENSURE( fe->cellCount > 0 );
			ENSURE( fe->mass > 0.0f );
			ENSURE( B3_IS_NULL( fe->fragmentBody ) );
			ENSURE( isfinite( fe->centerOfMassWorld.y ) );
			for ( int i = 0; i < fe->cellCount; ++i )
				ENSURE( b3VoxelData_IsSolid( vd, fe->cells[i] ) == false );
			got = true;
		}
		ENSURE( FractureBounded( worldId ) );
	}
	ENSURE( got );
	ENSURE( reason == b3_fractureStress ); // failed under self-load, not a hit
	ENSURE( b3VoxelData_GetCellCount( vd ) < cellsBefore );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( vd );
	return 0;
}

static int VoxelShapeTwoBodiesIndependentFracture( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &wd );
	b3World_EnableFracture( worldId, 1.0f, 0.0f );
	b3FractureTuning t = b3World_GetFractureTuning( worldId );
	t.warmupFrames = 2;
	b3World_SetFractureTuning( worldId, t );

	b3Vec3i cells[7 * 7 * 2];
	int n = FractureBoxCells( cells, 7, 7, 2, -3, -3, -1 ); // identical local grid for both
	b3VoxelData* vdA = NULL;
	b3VoxelData* vdB = NULL;
	b3BodyId shipA = MakeVoxelShapeBody( worldId, true, ( b3Vec3 ){ -12.0f, 0.0f, 0.0f }, cells, n, &vdA );
	b3BodyId shipB = MakeVoxelShapeBody( worldId, true, ( b3Vec3 ){ 12.0f, 0.0f, 0.0f }, cells, n, &vdB );
	ENSURE( b3World_MakeVoxelBodyFracture( worldId, shipA, b3GetFractureMaterial( b3_fractureGlass ), NULL ) >= 0 );
	ENSURE( b3World_MakeVoxelBodyFracture( worldId, shipB, b3GetFractureMaterial( b3_fractureGlass ), NULL ) >= 0 );
	int aBefore = b3VoxelData_GetCellCount( vdA );

	FractureFireBall( worldId, ( b3Vec3 ){ 12.0f, 0.0f, 9.0f }, ( b3Vec3 ){ 0.0f, 0.0f, -140.0f }, 0.6f ); // smash B only

	bool hitB = false;
	for ( int s = 0; s < 90 && !hitB; ++s )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3FractureEvents ev = b3World_GetFractureEvents( worldId );
		for ( int e = 0; e < ev.count; ++e )
		{
			ENSURE( B3_ID_EQUALS( ev.events[e].parentBody, shipB ) ); // never ship A
			ENSURE( ev.events[e].cellCount > 0 );
			for ( int i = 0; i < ev.events[e].cellCount; ++i )
			{
				b3Vec3i c = ev.events[e].cells[i]; // reported in B's own ORIGINAL coords (bias undone)
				ENSURE( c.x >= -3 && c.x <= 3 && c.y >= -3 && c.y <= 3 && c.z >= -1 && c.z <= 0 );
			}
			hitB = true;
		}
	}
	ENSURE( hitB );
	ENSURE( b3VoxelData_GetCellCount( vdB ) < n );		 // B (biased) lost cells
	ENSURE( b3VoxelData_GetCellCount( vdA ) == aBefore ); // A untouched despite identical coords

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( vdA );
	b3DestroyVoxelData( vdB );
	return 0;
}

// Opt-in perf harness (VOX_PERF=1): a big settled pile of small voxel fracture cubes, stepped
// single-threaded, with the profile sections averaged over a measurement window. Deterministic
// and low-noise for A/B optimisation. Run: VOX_PERF=1 test.exe VoxelPerfTest  (VOX_PERF_N=side).
int VoxelPerfTest( void )
{
	if ( getenv( "VOX_PERF" ) == NULL )
		return 0; // skipped in the normal suite

	int side = getenv( "VOX_PERF_N" ) ? atoi( getenv( "VOX_PERF_N" ) ) : 20;
	int layers = getenv( "VOX_PERF_L" ) ? atoi( getenv( "VOX_PERF_L" ) ) : 15;
	int warm = getenv( "VOX_PERF_WARM" ) ? atoi( getenv( "VOX_PERF_WARM" ) ) : 150;
	int meas = getenv( "VOX_PERF_MEAS" ) ? atoi( getenv( "VOX_PERF_MEAS" ) ) : 150;

	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId world = b3CreateWorld( &wd );
	b3World_EnableFracture( world, 1.0f, 0.0f );

	b3BodyDef gd = b3DefaultBodyDef();
	gd.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
	b3BodyId g = b3CreateBody( world, &gd );
	b3ShapeDef gsd = b3DefaultShapeDef();
	b3BoxHull gh = b3MakeBoxHull( 400.0f, 1.0f, 400.0f );
	b3CreateHullShape( g, &gsd, &gh.base );

	b3FractureMaterial mat = b3GetFractureMaterial( b3_fractureMetal );
	b3FractureDef def = b3DefaultFractureDef();
	int cubes = 0;
	for ( int gx = 0; gx < side; ++gx )
		for ( int gz = 0; gz < side; ++gz )
			for ( int gy = 0; gy < layers; ++gy )
			{
				int ox = gx * 3, oy = 1 + gy * 3, oz = gz * 3;
				b3Vec3i cells[8];
				int k = 0;
				for ( int x = 0; x < 2; ++x )
					for ( int y = 0; y < 2; ++y )
						for ( int z = 0; z < 2; ++z )
							cells[k++] = ( b3Vec3i ){ ox + x, oy + y, oz + z };
				b3World_CreateFractureVoxels( world, cells, 8, mat, &def );
				cubes++;
			}

	float dt = 1.0f / 60.0f;
	for ( int i = 0; i < warm; ++i )
		b3World_Step( world, dt, 4 );

	double sStep = 0, sColl = 0, sSolve = 0, sConstr = 0, sFrac = 0, sGather = 0, sAnalyze = 0, sPairs = 0;
	for ( int i = 0; i < meas; ++i )
	{
		b3World_Step( world, dt, 4 );
		b3Profile p = b3World_GetProfile( world );
		sStep += p.step;
		sColl += p.collide;
		sSolve += p.solve;
		sConstr += p.constraints;
		sFrac += p.fracture;
		sGather += p.fractureGather;
		sAnalyze += p.fractureAnalyze;
		sPairs += p.pairs;
	}
	b3Counters c = b3World_GetCounters( world );
	double inv = 1.0 / (double)meas;
	printf( "[VOX_PERF] cubes=%d bodies=%d contacts=%d | avg/%d steps: step=%.3f pairs=%.3f collide=%.3f solve=%.3f "
			"constr=%.3f frac=%.3f (gather=%.3f analyze=%.3f)\n",
			cubes, c.bodyCount, c.contactCount, meas, sStep * inv, sPairs * inv, sColl * inv, sSolve * inv, sConstr * inv,
			sFrac * inv, sGather * inv, sAnalyze * inv );
	b3DestroyWorld( world );
	return 0;
}

int FractureTest( void )
{
	RUN_SUBTEST( FractureConvexRests );
	RUN_SUBTEST( FractureConvexShatters );
	RUN_SUBTEST( FractureConvexCantileverSnaps );
	RUN_SUBTEST( FractureDynamicSpanSnapsUnderSelfWeight );
	RUN_SUBTEST( FractureStrideCantileverSnaps );
	RUN_SUBTEST( FractureParallelCantileverSnaps );
	RUN_SUBTEST( FractureDebrisBudget );
	RUN_SUBTEST( FractureConvertBody );
	RUN_SUBTEST( FractureRestAndShatter );
	RUN_SUBTEST( FractureCantileverSnaps );
	RUN_SUBTEST( FractureBeamSnapNormalBody );
	RUN_SUBTEST( FractureStressField );
	RUN_SUBTEST( FractureRestingBeamHolds );
	RUN_SUBTEST( FractureBridgeHoldsThenBreaks );
	RUN_SUBTEST( FractureDestructibleDominoesTopple );
	RUN_SUBTEST( FractureConvertPreservesMass );
	RUN_SUBTEST( FractureConvertPreservesGravityScale );
	RUN_SUBTEST( FractureConvertCompound );
	RUN_SUBTEST( VoxelShapeImpactFractureEmitsEvents );
	RUN_SUBTEST( VoxelShapeStressFractureEmitsEvents );
	RUN_SUBTEST( VoxelShapeTwoBodiesIndependentFracture );
	return 0;
}
