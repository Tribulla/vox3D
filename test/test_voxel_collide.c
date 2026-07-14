#include "box3d/box3d.h"

#include "voxel_collide.h" // internal box primitives

#include "test_macros.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

static b3VoxelOBB MakeOBB( b3Vec3 center, b3Quat q, b3Vec3 half )
{
	b3VoxelOBB o;
	o.center = center;
	o.half = half;
	o.axes[0] = b3RotateVector( q, ( b3Vec3 ){ 1.0f, 0.0f, 0.0f } );
	o.axes[1] = b3RotateVector( q, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } );
	o.axes[2] = b3RotateVector( q, ( b3Vec3 ){ 0.0f, 0.0f, 1.0f } );
	return o;
}

static float ProjR( const b3VoxelOBB* b, b3Vec3 a )
{
	return fabsf( b3Dot( a, b->axes[0] ) ) * b->half.x + fabsf( b3Dot( a, b->axes[1] ) ) * b->half.y +
		   fabsf( b3Dot( a, b->axes[2] ) ) * b->half.z;
}

static float OverlapAlong( const b3VoxelOBB* o0, const b3VoxelOBB* o1, b3Vec3 a )
{
	b3Vec3 d = b3Sub( o0->center, o1->center );
	return ProjR( o0, a ) + ProjR( o1, a ) - fabsf( b3Dot( a, d ) );
}

static float RefMinOverlap( const b3VoxelOBB* o0, const b3VoxelOBB* o1, b3Vec3* axisOut )
{
	b3Vec3 axes[15];
	int na = 0;
	for ( int i = 0; i < 3; ++i )
		axes[na++] = o0->axes[i];
	for ( int i = 0; i < 3; ++i )
		axes[na++] = o1->axes[i];
	for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j )
			axes[na++] = b3Cross( o0->axes[i], o1->axes[j] );

	b3Vec3 d = b3Sub( o0->center, o1->center );
	float minOv = FLT_MAX;
	b3Vec3 minAxis = { 0.0f, 0.0f, 1.0f };
	for ( int k = 0; k < na; ++k )
	{
		b3Vec3 a = axes[k];
		float l = b3Dot( a, a );
		if ( l <= 1e-9f )
			continue;
		a = b3MulSV( 1.0f / sqrtf( l ), a );
		float proj = b3Dot( a, d );
		float ov = ProjR( o0, a ) + ProjR( o1, a ) - fabsf( proj );
		if ( ov < minOv )
		{
			minOv = ov;
			minAxis = proj >= 0.0f ? a : b3Neg( a );
		}
	}
	*axisOut = minAxis;
	return minOv;
}

static uint32_t s_rng = 0x9e3779b9u;
static float Frand( void )
{
	s_rng ^= s_rng << 13;
	s_rng ^= s_rng >> 17;
	s_rng ^= s_rng << 5;
	return (float)( ( s_rng >> 8 ) & 0xFFFFFF ) / (float)0x1000000; // [0,1)
}
static float Frand2( void )
{
	return Frand() * 2.0f - 1.0f;
}
static b3Quat RandQuat( void )
{
	b3Vec3 axis = { Frand2(), Frand2(), Frand2() };
	float len = b3Length( axis );
	if ( len < 0.1f )
		axis = ( b3Vec3 ){ 0.0f, 0.0f, 1.0f };
	else
		axis = b3MulSV( 1.0f / len, axis );
	float ang = Frand() * B3_PI;
	float s = sinf( ang * 0.5f );
	return ( b3Quat ){ { axis.x * s, axis.y * s, axis.z * s }, cosf( ang * 0.5f ) };
}

static int VoxelObbKnownCases( void )
{
	b3Quat qi = b3Quat_identity;

	b3VoxelOBB a = MakeOBB( ( b3Vec3 ){ 0.8f, 0.0f, 0.0f }, qi, ( b3Vec3 ){ 0.5f, 0.5f, 0.5f } );
	b3VoxelOBB b = MakeOBB( ( b3Vec3 ){ 0.0f, 0.0f, 0.0f }, qi, ( b3Vec3 ){ 0.5f, 0.5f, 0.5f } );
	b3VoxelContact out[4];
	int n = b3VoxelCollideOBB( &a, &b, 0.0f, 4, out );
	ENSURE( n >= 1 );
	ENSURE_SMALL( out[0].normal.x - 1.0f, 1e-4f ); // B->A points +x (A at +x)
	ENSURE_SMALL( out[0].normal.y, 1e-4f );
	ENSURE_SMALL( out[0].normal.z, 1e-4f );
	ENSURE_SMALL( out[0].initialPenetration - 0.2f, 1e-4f );
	ENSURE( n == 4 );

	b3VoxelOBB c = MakeOBB( ( b3Vec3 ){ 3.0f, 0.0f, 0.0f }, qi, ( b3Vec3 ){ 0.5f, 0.5f, 0.5f } );
	ENSURE( b3VoxelCollideOBB( &c, &b, 0.0f, 4, out ) == 0 );

	b3VoxelOBB e = MakeOBB( ( b3Vec3 ){ 1.1f, 0.0f, 0.0f }, qi, ( b3Vec3 ){ 0.5f, 0.5f, 0.5f } );
	n = b3VoxelCollideOBB( &e, &b, 0.2f, 4, out );
	ENSURE( n >= 1 );
	ENSURE( out[0].initialPenetration < 0.0f ); // gap -> negative penetration
	ENSURE_SMALL( out[0].initialPenetration - ( -0.1f ), 1e-4f );
	return 0;
}

static int VoxelObbFuzz( void )
{
	s_rng = 0x1234567u;
	int checked = 0;
	for ( int iter = 0; iter < 40000; ++iter )
	{
		b3VoxelOBB o0 = MakeOBB( ( b3Vec3 ){ 0.0f, 0.0f, 0.0f }, RandQuat(),
								 ( b3Vec3 ){ 0.3f + Frand(), 0.3f + Frand(), 0.3f + Frand() } );
		b3VoxelOBB o1 = MakeOBB( ( b3Vec3 ){ Frand2() * 2.0f, Frand2() * 2.0f, Frand2() * 2.0f }, RandQuat(),
								 ( b3Vec3 ){ 0.3f + Frand(), 0.3f + Frand(), 0.3f + Frand() } );

		b3Vec3 refAxis;
		float refOv = RefMinOverlap( &o0, &o1, &refAxis );

		b3VoxelContact out[4];
		int n = b3VoxelCollideOBB( &o0, &o1, 0.0f, 4, out );

		if ( refOv < -1e-3f )
		{
			ENSURE( n == 0 );
			checked++;
		}
		else if ( refOv > 1e-3f )
		{
			ENSURE( n >= 1 );
			float ovN = OverlapAlong( &o0, &o1, out[0].normal );
			ENSURE_SMALL( ovN - refOv, 3e-3f );
			b3Vec3 diff = b3Sub( out[0].body1Point, out[0].body0Point );
			ENSURE( b3Dot( diff, out[0].normal ) > -1e-3f );
			float maxPen = out[0].initialPenetration;
			for ( int i = 1; i < n; ++i )
				maxPen = b3MaxFloat( maxPen, out[i].initialPenetration );
			ENSURE( maxPen > 0.0f );
			checked++;
		}
	}
	ENSURE( checked > 20000 ); // the fuzz actually exercised both classes
	return 0;
}

static int VoxelAabbFuzz( void )
{
	s_rng = 0x89abcdefu;
	for ( int iter = 0; iter < 40000; ++iter )
	{
		b3Vec3 c0 = { 0.0f, 0.0f, 0.0f }, c1 = { Frand2() * 2.0f, Frand2() * 2.0f, Frand2() * 2.0f };
		b3Vec3 h0 = { 0.3f + Frand(), 0.3f + Frand(), 0.3f + Frand() };
		b3Vec3 h1 = { 0.3f + Frand(), 0.3f + Frand(), 0.3f + Frand() };
		b3AABB a0 = { b3Sub( c0, h0 ), b3Add( c0, h0 ) };
		b3AABB a1 = { b3Sub( c1, h1 ), b3Add( c1, h1 ) };

		float ovx = b3MinFloat( a0.upperBound.x, a1.upperBound.x ) - b3MaxFloat( a0.lowerBound.x, a1.lowerBound.x );
		float ovy = b3MinFloat( a0.upperBound.y, a1.upperBound.y ) - b3MaxFloat( a0.lowerBound.y, a1.lowerBound.y );
		float ovz = b3MinFloat( a0.upperBound.z, a1.upperBound.z ) - b3MaxFloat( a0.lowerBound.z, a1.lowerBound.z );
		float minOv = b3MinFloat( ovx, b3MinFloat( ovy, ovz ) );

		b3VoxelContact out[4];
		int n = b3VoxelCollideAABB( &a0, &a1, 0.0f, 4, out );

		if ( minOv < -1e-3f )
			ENSURE( n == 0 );
		else if ( minOv > 1e-3f )
		{
			ENSURE( n >= 1 );
			ENSURE_SMALL( out[0].initialPenetration - minOv, 1e-4f );
			float nlen = b3Length( out[0].normal );
			ENSURE_SMALL( nlen - 1.0f, 1e-4f );
			b3Vec3 diff = b3Sub( out[0].body1Point, out[0].body0Point );
			ENSURE( b3Dot( diff, out[0].normal ) > -1e-3f );
		}
	}
	return 0;
}

static b3VoxelOBB CellOBB( b3Vec3i cell, float vs, b3Transform xf )
{
	b3Vec3 lc = { cell.x * vs, cell.y * vs, cell.z * vs };
	b3VoxelOBB o;
	o.center = b3Add( xf.p, b3RotateVector( xf.q, lc ) );
	o.axes[0] = b3RotateVector( xf.q, ( b3Vec3 ){ 1.0f, 0.0f, 0.0f } );
	o.axes[1] = b3RotateVector( xf.q, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } );
	o.axes[2] = b3RotateVector( xf.q, ( b3Vec3 ){ 0.0f, 0.0f, 1.0f } );
	float h = 0.5f * vs;
	o.half = ( b3Vec3 ){ h, h, h };
	return o;
}

static int OraclePairs( const b3Vec3i* c0, int n0, b3Transform xf0, const b3Vec3i* c1, int n1, b3Transform xf1, float* maxPen )
{
	int count = 0;
	float mp = -FLT_MAX;
	for ( int a = 0; a < n0; ++a )
	{
		b3VoxelOBB o0 = CellOBB( c0[a], 1.0f, xf0 );
		for ( int b = 0; b < n1; ++b )
		{
			b3VoxelOBB o1 = CellOBB( c1[b], 1.0f, xf1 );
			b3VoxelContact c[4];
			if ( b3VoxelCollideOBB( &o0, &o1, 0.0f, 1, c ) > 0 )
			{
				count++;
				if ( c[0].initialPenetration > mp )
					mp = c[0].initialPenetration;
			}
		}
	}
	*maxPen = mp;
	return count;
}

static b3AABB WorldBounds( const b3Vec3i* cells, int n, b3Transform xf )
{
	b3AABB r = { { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };
	for ( int i = 0; i < n; ++i )
	{
		for ( int k = 0; k < 8; ++k )
		{
			b3Vec3 corner = { cells[i].x + ( ( k & 1 ) ? 0.5f : -0.5f ), cells[i].y + ( ( k & 2 ) ? 0.5f : -0.5f ),
							  cells[i].z + ( ( k & 4 ) ? 0.5f : -0.5f ) };
			b3Vec3 w = b3Add( xf.p, b3RotateVector( xf.q, corner ) );
			r.lowerBound.x = b3MinFloat( r.lowerBound.x, w.x );
			r.lowerBound.y = b3MinFloat( r.lowerBound.y, w.y );
			r.lowerBound.z = b3MinFloat( r.lowerBound.z, w.z );
			r.upperBound.x = b3MaxFloat( r.upperBound.x, w.x );
			r.upperBound.y = b3MaxFloat( r.upperBound.y, w.y );
			r.upperBound.z = b3MaxFloat( r.upperBound.z, w.z );
		}
	}
	return r;
}

static bool InAABB( b3Vec3 p, b3AABB b, float m )
{
	return p.x >= b.lowerBound.x - m && p.x <= b.upperBound.x + m && p.y >= b.lowerBound.y - m &&
		   p.y <= b.upperBound.y + m && p.z >= b.lowerBound.z - m && p.z <= b.upperBound.z + m;
}

static int VoxelDriverBasic( void )
{
	b3Vec3i c[] = { { 0, 0, 0 } };
	b3VoxelData* v0 = b3CreateVoxelData( c, 1, 1.0f );
	b3VoxelData* v1 = b3CreateVoxelData( c, 1, 1.0f );
	b3Transform x1 = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3VoxelContact out[64];

	b3Transform x0 = { { 0.8f, 0.0f, 0.0f }, b3Quat_identity };
	int n = b3VoxelCollide( v0, x0, v1, x1, 0.0f, 64, out );
	ENSURE( n >= 1 );
	ENSURE_SMALL( out[0].normal.x - 1.0f, 1e-3f );
	ENSURE_SMALL( out[0].initialPenetration - 0.2f, 1e-3f );

	b3Transform xf = { { 3.0f, 0.0f, 0.0f }, b3Quat_identity };
	ENSURE( b3VoxelCollide( v0, xf, v1, x1, 0.0f, 64, out ) == 0 );

	b3DestroyVoxelData( v0 );
	b3DestroyVoxelData( v1 );
	return 0;
}

static int VoxelDriverSlab( void )
{
	b3Vec3i c[9];
	int n = 0;
	for ( int x = 0; x < 3; ++x )
		for ( int z = 0; z < 3; ++z )
			c[n++] = ( b3Vec3i ){ x, 0, z };
	b3VoxelData* v0 = b3CreateVoxelData( c, 9, 1.0f );
	b3VoxelData* v1 = b3CreateVoxelData( c, 9, 1.0f );
	b3Transform x0 = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3Transform x1 = { { 0.0f, 0.9f, 0.0f }, b3Quat_identity };
	b3VoxelContact out[64];
	int cnt = b3VoxelCollide( v0, x0, v1, x1, 0.0f, 64, out );
	ENSURE( cnt >= 4 ); // a spread resting manifold, not a single point
	ENSURE_SMALL( out[0].normal.y - ( -1.0f ), 1e-3f );	 // B (above) -> A (below) = -y
	ENSURE_SMALL( out[0].initialPenetration - 0.1f, 1e-3f );
	b3AABB wb0 = WorldBounds( c, 9, x0 ), wb1 = WorldBounds( c, 9, x1 );
	for ( int i = 0; i < cnt; ++i )
	{
		ENSURE( InAABB( out[i].body0Point, wb0, 0.02f ) );
		ENSURE( InAABB( out[i].body1Point, wb1, 0.02f ) );
	}
	b3DestroyVoxelData( v0 );
	b3DestroyVoxelData( v1 );
	return 0;
}

static int VoxelDriverOracle( void )
{
	s_rng = 0xC0FFEEu;
	b3Vec3i c0[27];
	int n0 = 0;
	for ( int x = 0; x < 3; ++x )
		for ( int y = 0; y < 3; ++y )
			for ( int z = 0; z < 3; ++z )
				c0[n0++] = ( b3Vec3i ){ x, y, z };
	b3Vec3i c1[8];
	int n1 = 0;
	for ( int x = 0; x < 2; ++x )
		for ( int y = 0; y < 2; ++y )
			for ( int z = 0; z < 2; ++z )
				c1[n1++] = ( b3Vec3i ){ x, y, z };
	b3VoxelData* v0 = b3CreateVoxelData( c0, n0, 1.0f );
	b3VoxelData* v1 = b3CreateVoxelData( c1, n1, 1.0f );

	b3Transform x0 = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
	int shallowChecked = 0;
	for ( int iter = 0; iter < 4000; ++iter )
	{
		bool rotate = ( iter & 1 ) != 0;
		b3Transform x1 = { { 0.5f + Frand2() * 2.5f, Frand2() * 3.0f, Frand2() * 3.0f }, rotate ? RandQuat() : b3Quat_identity };

		float mp;
		int op = OraclePairs( c0, n0, x0, c1, n1, x1, &mp );

		b3VoxelContact out[64];
		int n = b3VoxelCollide( v0, x0, v1, x1, 0.0f, 64, out );

		if ( op == 0 )
		{
			ENSURE( n == 0 ); // no cell pair overlaps -> no contacts
		}
		else if ( mp > 0.02f && mp < 0.4f )
		{
			ENSURE( n >= 1 ); // shallow surface overlap -> at least one contact survives culling
			b3AABB wb0 = WorldBounds( c0, n0, x0 ), wb1 = WorldBounds( c1, n1, x1 );
			b3AABB wbU = { { b3MinFloat( wb0.lowerBound.x, wb1.lowerBound.x ), b3MinFloat( wb0.lowerBound.y, wb1.lowerBound.y ),
							 b3MinFloat( wb0.lowerBound.z, wb1.lowerBound.z ) },
						   { b3MaxFloat( wb0.upperBound.x, wb1.upperBound.x ), b3MaxFloat( wb0.upperBound.y, wb1.upperBound.y ),
							 b3MaxFloat( wb0.upperBound.z, wb1.upperBound.z ) } };
			for ( int i = 0; i < n; ++i )
			{
				ENSURE_SMALL( b3Length( out[i].normal ) - 1.0f, 1e-3f );
				ENSURE( InAABB( out[i].body0Point, wbU, 0.1f ) );
				ENSURE( InAABB( out[i].body1Point, wbU, 0.1f ) );
				b3Vec3 diff = b3Sub( out[i].body1Point, out[i].body0Point );
				ENSURE_SMALL( b3Dot( diff, out[i].normal ) - out[i].initialPenetration, 1e-4f );
			}
			shallowChecked++;
		}
	}
	ENSURE( shallowChecked > 200 ); // the fuzz actually hit the shallow-overlap regime
	b3DestroyVoxelData( v0 );
	b3DestroyVoxelData( v1 );
	return 0;
}

static b3VoxelData* MakeVoxelBox( int x0, int x1, int y0, int y1, int z0, int z1, float s )
{
	int nx = x1 - x0 + 1, ny = y1 - y0 + 1, nz = z1 - z0 + 1;
	int count = nx * ny * nz;
	b3Vec3i* cells = (b3Vec3i*)malloc( (size_t)count * sizeof( b3Vec3i ) );
	int k = 0;
	for ( int x = x0; x <= x1; ++x )
		for ( int y = y0; y <= y1; ++y )
			for ( int z = z0; z <= z1; ++z )
				cells[k++] = (b3Vec3i){ x, y, z };
	b3VoxelData* v = b3CreateVoxelData( cells, count, s );
	free( cells );
	return v;
}

static int VoxelBodyRest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3VoxelData* ground = MakeVoxelBox( -5, 5, 0, 0, -5, 5, 1.0f );
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateVoxelShape( groundId, &groundShapeDef, ground );

	b3VoxelData* cube = MakeVoxelBox( -1, 1, -1, 1, -1, 1, 1.0f );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateVoxelShape( bodyId, &shapeDef, cube );

	ENSURE( b3Body_GetMass( bodyId ) > 0.0f );

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3Pos position = b3Body_GetPosition( bodyId );
	b3Quat rotation = b3Body_GetRotation( bodyId );

	b3DestroyWorld( worldId );
	b3DestroyVoxelData( ground );
	b3DestroyVoxelData( cube );

	ENSURE( position.y > 1.5f );			 // did not tunnel through the slab
	ENSURE_SMALL( position.y - 2.0f, 0.2f ); // came to rest on top
	ENSURE_SMALL( rotation.v.x, 0.05f );	 // did not tumble or explode
	ENSURE_SMALL( rotation.v.z, 0.05f );
	return 0;
}

static int VoxelContactSpread( void )
{
	b3Vec3i c[25];
	int n = 0;
	for ( int x = 0; x < 5; ++x )
		for ( int z = 0; z < 5; ++z )
			c[n++] = ( b3Vec3i ){ x, 0, z };
	b3VoxelData* v0 = b3CreateVoxelData( c, 25, 1.0f );
	b3VoxelData* v1 = b3CreateVoxelData( c, 25, 1.0f );
	b3Transform x0 = { { 0.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3Transform x1 = { { 0.0f, 0.95f, 0.0f }, b3Quat_identity }; // 0.05 overlap in y

	b3VoxelContact out[64];
	int cnt = b3VoxelCollide( v0, x0, v1, x1, 0.0f, 64, out );
	ENSURE( cnt >= 4 );

	float minX = FLT_MAX, maxX = -FLT_MAX, minZ = FLT_MAX, maxZ = -FLT_MAX;
	for ( int i = 0; i < cnt; ++i )
	{
		minX = b3MinFloat( minX, out[i].body0Point.x );
		maxX = b3MaxFloat( maxX, out[i].body0Point.x );
		minZ = b3MinFloat( minZ, out[i].body0Point.z );
		maxZ = b3MaxFloat( maxZ, out[i].body0Point.z );
	}
	ENSURE( maxX - minX > 4.5f );
	ENSURE( maxZ - minZ > 4.5f );

	b3DestroyVoxelData( v0 );
	b3DestroyVoxelData( v1 );
	return 0;
}

static uint32_t HashBytes( uint32_t h, const void* data, size_t bytes )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( size_t i = 0; i < bytes; ++i )
	{
		h ^= p[i];
		h *= 16777619u; // FNV-1a
	}
	return h;
}

static uint32_t HashVoxelScene( int workerCount, int steps )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.workerCount = workerCount;
	wd.gravity = ( b3Vec3 ){ 0.0f, -10.0f, 0.0f };
	b3WorldId world = b3CreateWorld( &wd );

	b3VoxelData* ground = MakeVoxelBox( -10, 10, 0, 0, -10, 10, 1.0f );
	b3BodyDef gd = b3DefaultBodyDef();
	b3BodyId gid = b3CreateBody( world, &gd );
	b3ShapeDef gsd = b3DefaultShapeDef();
	b3CreateVoxelShape( gid, &gsd, ground );

	enum
	{
		NB = 4 * 4 * 2
	};
	b3VoxelData* datas[NB];
	b3BodyId bodies[NB];
	int nb = 0;
	for ( int gx = 0; gx < 4; ++gx )
		for ( int gz = 0; gz < 4; ++gz )
			for ( int level = 0; level < 2; ++level )
			{
				b3VoxelData* cube = MakeVoxelBox( -1, 1, -1, 1, -1, 1, 1.0f );
				datas[nb] = cube;
				b3BodyDef bd = b3DefaultBodyDef();
				bd.type = b3_dynamicBody;
				bd.position = ( b3Pos ){ (float)( gx * 4 - 6 ), 2.0f + (float)level * 3.2f, (float)( gz * 4 - 6 ) };
				b3BodyId body = b3CreateBody( world, &bd );
				b3ShapeDef sd = b3DefaultShapeDef();
				sd.density = 1.0f;
				b3CreateVoxelShape( body, &sd, cube );
				bodies[nb] = body;
				nb++;
			}

	for ( int i = 0; i < steps; ++i )
		b3World_Step( world, 1.0f / 60.0f, 4 );

	uint32_t h = 2166136261u;
	for ( int i = 0; i < nb; ++i )
	{
		b3Pos p = b3Body_GetPosition( bodies[i] );
		b3Quat q = b3Body_GetRotation( bodies[i] );
		h = HashBytes( h, &p, sizeof( p ) );
		h = HashBytes( h, &q, sizeof( q ) );
	}

	b3DestroyWorld( world );
	for ( int i = 0; i < nb; ++i )
		b3DestroyVoxelData( datas[i] );
	b3DestroyVoxelData( ground );
	return h;
}

static int VoxelDeterminism( void )
{
	uint32_t h1 = HashVoxelScene( 1, 200 );
	uint32_t h1b = HashVoxelScene( 1, 200 );
	uint32_t h2 = HashVoxelScene( 2, 200 );
	uint32_t h4 = HashVoxelScene( 4, 200 );

	ENSURE( h1 == h1b ); // reproducible run-to-run
	ENSURE( h1 == h2 );	 // independent of worker count
	ENSURE( h1 == h4 );
	return 0;
}

static float DropConvexOnVoxel( int shapeKind )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -10.0f, 0.0f };
	b3WorldId world = b3CreateWorld( &wd );

	b3VoxelData* ground = MakeVoxelBox( -5, 5, 0, 0, -5, 5, 1.0f ); // top face at y = 0.5
	b3BodyDef gd = b3DefaultBodyDef();
	b3BodyId gid = b3CreateBody( world, &gd );
	b3ShapeDef gsd = b3DefaultShapeDef();
	b3CreateVoxelShape( gid, &gsd, ground );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = ( b3Pos ){ 0.0f, 4.0f, 0.0f };
	b3BodyId body = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;

	if ( shapeKind == 0 )
	{
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( body, &sd, &sphere );
	}
	else if ( shapeKind == 1 )
	{
		b3BoxHull hull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3CreateHullShape( body, &sd, &hull.base );
	}
	else
	{
		b3Capsule cap = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.5f }; // lying flat
		b3CreateCapsuleShape( body, &sd, &cap );
	}

	for ( int i = 0; i < 240; ++i )
		b3World_Step( world, 1.0f / 60.0f, 4 );

	float y = (float)b3Body_GetPosition( body ).y;
	b3DestroyWorld( world );
	b3DestroyVoxelData( ground );
	return y;
}

static int VoxelConvexRest( void )
{
	for ( int kind = 0; kind < 3; ++kind )
	{
		float y = DropConvexOnVoxel( kind );
		ENSURE( y > 0.6f );			   // did not tunnel through the voxel ground
		ENSURE_SMALL( y - 1.0f, 0.2f ); // came to rest on top
	}
	return 0;
}

static int VoxelConvexFastHitDiag( void )
{
	float speeds[] = { 5.0f, 20.0f, 50.0f, 100.0f, 200.0f };
	for ( int s = 0; s < 5; ++s )
	{
		b3WorldDef wd = b3DefaultWorldDef();
		wd.gravity = ( b3Vec3 ){ 0.0f, 0.0f, 0.0f };
		b3WorldId world = b3CreateWorld( &wd );

		b3VoxelData* wall = MakeVoxelBox( 0, 2, -3, 3, -3, 3, 1.0f ); // 3 thick in x
		b3BodyDef wallDef = b3DefaultBodyDef();
		b3BodyId wallId = b3CreateBody( world, &wallDef );
		b3ShapeDef wsd = b3DefaultShapeDef();
		b3CreateVoxelShape( wallId, &wsd, wall );

		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = ( b3Pos ){ -8.0f, 0.0f, 0.0f };
		bd.linearVelocity = ( b3Vec3 ){ speeds[s], 0.0f, 0.0f };
		b3BodyId body = b3CreateBody( world, &bd );
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = 1.0f;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( body, &sd, &sphere );

		for ( int i = 0; i < 90; ++i )
			b3World_Step( world, 1.0f / 60.0f, 4 );

		b3Pos p = b3Body_GetPosition( body );
		b3Vec3 v = b3Body_GetLinearVelocity( body );
		printf( "  speed=%6.1f -> finalX=%10.3f  y=%8.3f z=%8.3f  |v|=%10.3f\n", speeds[s], (float)p.x, (float)p.y, (float)p.z,
				b3Length( v ) );
		float finalX = (float)p.x;

		b3DestroyWorld( world );
		b3DestroyVoxelData( wall );

		ENSURE( finalX < 3.0f );
	}
	return 0;
}

int VoxelCollideTest( void )
{
	RUN_SUBTEST( VoxelConvexFastHitDiag );
	RUN_SUBTEST( VoxelConvexRest );
	RUN_SUBTEST( VoxelObbKnownCases );
	RUN_SUBTEST( VoxelObbFuzz );
	RUN_SUBTEST( VoxelAabbFuzz );
	RUN_SUBTEST( VoxelDriverBasic );
	RUN_SUBTEST( VoxelDriverSlab );
	RUN_SUBTEST( VoxelDriverOracle );
	RUN_SUBTEST( VoxelContactSpread );
	RUN_SUBTEST( VoxelDeterminism );
	RUN_SUBTEST( VoxelBodyRest );
	return 0;
}
