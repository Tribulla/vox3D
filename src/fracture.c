#include "fracture.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include "container.h"
#include "core.h"
#include "parallel_for.h"
#include "physics_world.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static const int FACE[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
static const float B3F_EPS = 1e-9f;

static int b3F_iaxis( b3Vec3i c, int a )
{
	return ( &c.x )[a];
}
static float b3F_faxis( b3Vec3 v, int a )
{
	return ( &v.x )[a];
}
static float b3F_min( float a, float b )
{
	return a < b ? a : b;
}
static float b3F_max( float a, float b )
{
	return a > b ? a : b;
}

static float b3F_hash01( int i, int j )
{
	unsigned h = (unsigned)( i * 374761393 ) ^ (unsigned)( j * 668265263 );
	h = ( h ^ ( h >> 13 ) ) * 1274126177u;
	return ( ( h ^ ( h >> 16 ) ) & 0xffffff ) / (float)( 0x1000000 );
}
static float b3F_valnoise( float x, float y )
{
	int xi = (int)floorf( x ), yi = (int)floorf( y );
	float xf = x - xi, yf = y - yi;
	float sx = xf * xf * ( 3 - 2 * xf ), sy = yf * yf * ( 3 - 2 * yf );
	float a = b3F_hash01( xi, yi ), b = b3F_hash01( xi + 1, yi );
	float c = b3F_hash01( xi, yi + 1 ), d = b3F_hash01( xi + 1, yi + 1 );
	float top = a + ( b - a ) * sx, bot = c + ( d - c ) * sx;
	return ( top + ( bot - top ) * sy ) * 2.0f - 1.0f;
}

static uint32_t b3F_heatColor( float t )
{
	float r, g, b;
	t = t < 0 ? 0 : ( t > 1 ? 1 : t );
	if ( t < 0.5f )
	{
		float u = t / 0.5f;
		r = 0.1f;
		g = 0.2f + 0.8f * u;
		b = 1.0f - 0.9f * u;
	}
	else
	{
		float u = ( t - 0.5f ) / 0.5f;
		r = 0.1f + 0.9f * u;
		g = 1.0f - 0.9f * u;
		b = 0.1f;
	}
	uint32_t ri = (uint32_t)( 255.0f * r ), gi = (uint32_t)( 255.0f * g ), bi = (uint32_t)( 255.0f * b );
	uint32_t col = ( ri << 16 ) | ( gi << 8 ) | bi;
	return col ? col : 1u;
}

static uint32_t b3F_fragmentColor( int piece )
{
	unsigned h = (unsigned)( piece * 2654435761u );
	uint32_t c = ( h & 0x7f7f7f ) | 0x404040;
	return c ? c : 1u;
}

typedef struct b3F_Cell
{
	uint64_t key; // 0 = empty
	int value;
} b3F_Cell;

typedef struct b3F_Map
{
	b3F_Cell* slots;
	int cap;
	int count;
} b3F_Map;

static uint64_t b3F_packCell( b3Vec3i c )
{
	const int64_t OFF = 1 << 20;
	uint64_t x = (uint64_t)( (int64_t)c.x + OFF );
	uint64_t y = (uint64_t)( (int64_t)c.y + OFF );
	uint64_t z = (uint64_t)( (int64_t)c.z + OFF );
	return ( x << 42 ) | ( y << 21 ) | z; // never 0 for real cells
}
static uint32_t b3F_mixU64( uint64_t k )
{
	k ^= k >> 33;
	k *= 0xff51afd7ed558ccdULL;
	k ^= k >> 33;
	k *= 0xc4ceb9fe1a85ec53ULL;
	k ^= k >> 33;
	return (uint32_t)k;
}
static void b3F_mapPut( b3F_Map* m, uint64_t key, int value );

static void b3F_mapGrow( b3F_Map* m )
{
	int newCap = m->cap ? 2 * m->cap : 64;
	b3F_Cell* old = m->slots;
	int oldCap = m->cap;
	m->slots = (b3F_Cell*)b3AllocZeroed( (size_t)newCap * sizeof( b3F_Cell ) );
	m->cap = newCap;
	m->count = 0;
	for ( int i = 0; i < oldCap; ++i )
	{
		if ( old[i].key != 0 )
			b3F_mapPut( m, old[i].key, old[i].value );
	}
	if ( old )
		b3Free( old, (size_t)oldCap * sizeof( b3F_Cell ) );
}
static void b3F_mapPut( b3F_Map* m, uint64_t key, int value )
{
	if ( m->cap == 0 || 10 * ( m->count + 1 ) >= 7 * m->cap )
		b3F_mapGrow( m );
	uint32_t mask = (uint32_t)( m->cap - 1 );
	uint32_t i = b3F_mixU64( key ) & mask;
	while ( m->slots[i].key != 0 && m->slots[i].key != key )
		i = ( i + 1 ) & mask;
	if ( m->slots[i].key == 0 )
		m->count++;
	m->slots[i].key = key;
	m->slots[i].value = value;
}
static int b3F_mapGet( const b3F_Map* m, uint64_t key )
{
	if ( m->cap == 0 )
		return -1;
	uint32_t mask = (uint32_t)( m->cap - 1 );
	uint32_t i = b3F_mixU64( key ) & mask;
	while ( m->slots[i].key != 0 )
	{
		if ( m->slots[i].key == key )
			return m->slots[i].value;
		i = ( i + 1 ) & mask;
	}
	return -1;
}
static void b3F_mapFree( b3F_Map* m )
{
	if ( m->slots )
		b3Free( m->slots, (size_t)m->cap * sizeof( b3F_Cell ) );
	m->slots = NULL;
	m->cap = 0;
	m->count = 0;
}

typedef struct b3FractureVoxel
{
	b3Vec3i cell;
	int mat;
	int piece; // -1 = destroyed
	b3Vec3 local;
	b3ShapeId shape;
	float stress;		// this frame's raw computed failure ratio (drives the display EMA)
	float stressShown;	// temporally smoothed value used for colouring; frozen when the body sleeps
	b3Vec3 force;	 // EMA-smoothed contact force (section analysis)
	b3Vec3 forceRaw; // this frame's raw contact force (local impact)
	int restOn;		 // -1 none, -2 static/fixed support, else a dynamic body key
	uint8_t anchor;
	uint8_t brokenFaces; // bitmask of severed face-bonds
} b3FractureVoxel;

b3DeclareArrayNative( b3FractureVoxel );
b3DeclareArrayNative( b3FractureMaterial );

typedef struct b3F_Chunk
{
	b3HullData* hull; // owned convex hull, local frame
	b3Vec3 centroid;  // local centre of mass
	float mass;
	int mat;
	int piece; // -1 = destroyed
	b3ShapeId shape;
	float stress;
	float stressShown; // temporally smoothed display value; frozen when the body sleeps
	b3Vec3 force;	 // EMA-smoothed contact force
	b3Vec3 forceRaw; // this frame's raw contact force
	int restOn;		 // -1 none, -2 static/fixed, else a dynamic body key
	uint8_t anchor;
	int scratchLocal; // transient: this chunk's local index during its piece's analysis
} b3F_Chunk;

b3DeclareArrayNative( b3F_Chunk );

typedef struct b3F_Interface
{
	int a, b;		 // chunk ids (global)
	b3Vec3 centroid; // local
	b3Vec3 normal;	 // local, points from a toward b
	float area;
	uint8_t broken;
} b3F_Interface;

b3DeclareArrayNative( b3F_Interface );

enum
{
	b3F_kindVoxel = 0,
	b3F_kindChunk = 1,
};

typedef struct b3FracturePiece
{
	b3BodyId body;
	b3Array( int ) voxels; // member node ids (voxels or chunks by kind)
	b3Vec3 localOffset;	   // voxel body-frame centre = cell*voxel + localOffset (voxel kind)
	uint8_t kind;
	uint8_t isStatic;
	uint8_t debris;
	uint8_t merge;
	uint8_t active;
	float peakApproach;
	int overloadStreak;
} b3FracturePiece;

b3DeclareArrayNative( b3FracturePiece );

typedef struct b3F_Buf
{
	void* ptr;
	size_t cap; // bytes
} b3F_Buf;

enum b3F_ScratchId
{
	B3F_SC_CONTACTS, // b3F_gatherContactForces: contact query buffer

	B3F_SC_C_WC,
	B3F_SC_C_MASS,
	B3F_SC_C_FEXT,
	B3F_SC_C_SUPPORT,
	B3F_SC_C_COORD,
	B3F_SC_C_SORTED,
	B3F_SC_C_SIDX,
	B3F_SC_C_SUPPRE,
	B3F_SC_C_PIFS,
	B3F_SC_C_IA,
	B3F_SC_C_IB,
	B3F_SC_C_ICEN,
	B3F_SC_C_IAREA,
	B3F_SC_C_ISTR,
	B3F_SC_C_STRADDLE,
	B3F_SC_C_PREF,
	B3F_SC_C_PRECROSS,
	B3F_SC_C_PREMP,
	B3F_SC_C_PREM,
	B3F_SC_C_SUFF,
	B3F_SC_C_SUFCROSS,
	B3F_SC_C_SUFMP,
	B3F_SC_C_SUFM,

	B3F_SC_V_VOX,
	B3F_SC_V_WP,
	B3F_SC_V_MASS,
	B3F_SC_V_FEXT,
	B3F_SC_V_SUPPORT,
	B3F_SC_V_BONDS,
	B3F_SC_V_Q,
	B3F_SC_V_ORDER,
	B3F_SC_V_BINOFF,
	B3F_SC_V_CURSOR,
	B3F_SC_V_SUPPRE,
	B3F_SC_V_PREF,
	B3F_SC_V_PRECROSS,
	B3F_SC_V_PREMP,
	B3F_SC_V_PREM,
	B3F_SC_V_SUFF,
	B3F_SC_V_SUFCROSS,
	B3F_SC_V_SUFMP,
	B3F_SC_V_SUFM,

	B3F_SC_PLIST, // parallel analysis: piece worklist (worker 0 only)
	B3F_SC_PDEC,  // parallel analysis: per-piece decisions (worker 0 only)

	B3F_SC_COUNT
};

typedef struct b3FractureWorld
{
	b3WorldId worldId;
	float voxel, radius, groundY;
	b3Vec3 gravity;
	long frame;
	b3FractureTuning tuning;

	b3Array( b3FractureVoxel ) voxels;
	b3Array( b3FracturePiece ) pieces;
	b3Array( b3FractureMaterial ) materials;
	b3F_Map cellToVox;

	b3Array( b3F_Chunk ) chunks;
	b3Array( b3F_Interface ) interfaces;

	b3Array( int ) chunkIfStart;
	b3Array( int ) chunkIfCount;
	b3Array( int ) chunkIfList;

	b3Array( int ) debrisQueue; // split-born single-chunk/single-voxel pieces, oldest first
	int debrisHead;				// consumed prefix of debrisQueue (tuning.maxDebris enforcement)

	float profSeverMs; // this step's sever/split time (b3Profile.fractureSever), reset each step

	b3F_Buf scratch[B3_MAX_WORKERS][B3F_SC_COUNT];
} b3FractureWorld;

static void* b3F_scratchW( b3FractureWorld* fw, int worker, int slot, size_t bytes )
{
	b3F_Buf* b = &fw->scratch[worker][slot];
	if ( b->cap < bytes )
	{
		size_t nb = bytes + ( bytes >> 1 ) + 64;
		if ( b->ptr )
			b3Free( b->ptr, b->cap );
		b->ptr = b3Alloc( nb );
		b->cap = nb;
	}
	return b->ptr;
}

static void* b3F_scratch( b3FractureWorld* fw, int slot, size_t bytes )
{
	return b3F_scratchW( fw, 0, slot, bytes );
}

b3FractureMaterial b3GetFractureMaterial( b3FractureMaterialType type )
{
	static const b3FractureMaterial table[] = {
		{ 2.4f, 2400.0f, 4.0f, 0.03f, 0.70f, 0x9E9E96 }, // concrete
		{ 1.9f, 2700.0f, 4.0f, 0.05f, 0.65f, 0xB35442 }, // brick
		{ 2.6f, 2250.0f, 5.0f, 0.04f, 0.75f, 0x80808C }, // stone
		{ 0.7f, 6000.0f, 2.0f, 0.10f, 0.60f, 0x8C6133 }, // wood
		{ 2.5f, 1200.0f, 3.0f, 0.08f, 0.40f, 0x8CC7D9 }, // glass
		{ 7.8f, 45000.0f, 1.5f, 0.20f, 0.45f, 0xB8BCCC }, // metal
	};
	int i = (int)type;
	if ( i < 0 || i >= (int)( sizeof( table ) / sizeof( table[0] ) ) )
		i = 0;
	return table[i];
}

b3FractureDef b3DefaultFractureDef( void )
{
	b3FractureDef def = { 0 };
	def.merge = false;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3FractureTuning b3DefaultFractureTuning( void )
{
	b3FractureTuning t = { 0 };
	t.strengthScale = 1.0f;
	t.contactSmoothing = 0.12f;
	t.fractureRoughness = 1.5f;
	t.settleSpeed = 3.0f;
	t.impactSpeed = 4.0f;
	t.impactBearing = 1.0f;
	t.impactRadius = 2;
	t.minFragment = 1; // keep every fragment (down to a single voxel); no dust culling
	t.fractureHoldFrames = 4;
	t.warmupFrames = 25;
	t.analysisStride = 1;
	t.maxDebris = 0;
	t.fractureEnabled = true;
	t.contactStress = true;
	t.parallelAnalysis = false;
	return t;
}

static b3FractureWorld* b3F_getOrCreate( b3World* world, b3WorldId worldId, float voxel, float groundY )
{
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	if ( fw == NULL )
	{
		fw = (b3FractureWorld*)b3AllocZeroed( sizeof( b3FractureWorld ) );
		fw->worldId = worldId;
		fw->voxel = voxel;
		fw->radius = 0.5f * voxel;
		fw->groundY = groundY;
		fw->tuning = b3DefaultFractureTuning();
		world->fractureWorld = fw;
	}
	return fw;
}

static b3FractureVoxel* b3F_vox( b3FractureWorld* fw, int v )
{
	return fw->voxels.data + v;
}

static float b3F_voxelMass( b3FractureWorld* fw, int v )
{
	b3FractureVoxel* vx = b3F_vox( fw, v );
	return fw->materials.data[vx->mat].density * fw->voxel * fw->voxel * fw->voxel;
}

static b3Vec3 b3F_worldPosLocal( b3WorldTransform xf, b3Vec3 local )
{
	return b3ToVec3( b3TransformWorldPoint( xf, local ) );
}

static int b3F_voxelAt( b3FractureWorld* fw, int p, b3Pos worldPt )
{
	b3FracturePiece* P = fw->pieces.data + p;
	b3WorldTransform xf = b3Body_GetTransform( P->body );
	b3Vec3 local = b3InvTransformWorldPoint( xf, worldPt );
	b3Vec3i cell;
	for ( int a = 0; a < 3; ++a )
	{
		float f = ( b3F_faxis( local, a ) - b3F_faxis( P->localOffset, a ) ) / fw->voxel;
		( &cell.x )[a] = (int)lroundf( f );
	}
	int v = b3F_mapGet( &fw->cellToVox, b3F_packCell( cell ) );
	if ( v >= 0 && b3F_vox( fw, v )->piece == p )
		return v;
	for ( int f = 0; f < 6; ++f )
	{
		b3Vec3i nc = { cell.x + FACE[f][0], cell.y + FACE[f][1], cell.z + FACE[f][2] };
		int nv = b3F_mapGet( &fw->cellToVox, b3F_packCell( nc ) );
		if ( nv >= 0 && b3F_vox( fw, nv )->piece == p )
			return nv;
	}
	return -1;
}

static int b3F_neighbor( b3FractureWorld* fw, int v, int f, int piece )
{
	b3FractureVoxel* vx = b3F_vox( fw, v );
	if ( ( vx->brokenFaces >> f ) & 1u )
		return -1;
	b3Vec3i nc = { vx->cell.x + FACE[f][0], vx->cell.y + FACE[f][1], vx->cell.z + FACE[f][2] };
	int nv = b3F_mapGet( &fw->cellToVox, b3F_packCell( nc ) );
	if ( nv >= 0 && b3F_vox( fw, nv )->piece == piece )
		return nv;
	return -1;
}

static void b3F_sever( b3FractureWorld* fw, int v, int f )
{
	b3FractureVoxel* vx = b3F_vox( fw, v );
	b3Vec3i nc = { vx->cell.x + FACE[f][0], vx->cell.y + FACE[f][1], vx->cell.z + FACE[f][2] };
	int nv = b3F_mapGet( &fw->cellToVox, b3F_packCell( nc ) );
	vx->brokenFaces |= (uint8_t)( 1u << f );
	if ( nv >= 0 )
		b3F_vox( fw, nv )->brokenFaces |= (uint8_t)( 1u << ( f ^ 1 ) );
}

static int b3F_allocPiece( b3FractureWorld* fw )
{
	b3FracturePiece piece = { 0 };
	b3Array_Push( fw->pieces, piece );
	return fw->pieces.count - 1;
}

static void b3F_formBody( b3FractureWorld* fw, int p, const int* voxels, int count, const b3Vec3* wpos, b3Quat rot,
						  b3Vec3 vel, b3Vec3 omega, bool isStatic, bool debris, bool merge )
{
	double M = 0.0;
	b3Vec3 com = b3Vec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		float m = b3F_voxelMass( fw, voxels[i] );
		M += m;
		com = b3MulAdd( com, m, wpos[i] );
	}
	com = b3MulSV( 1.0f / (float)M, com );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = isStatic ? b3_staticBody : b3_dynamicBody;
	bd.position = b3ToPos( com );
	bd.rotation = rot;
	bd.linearVelocity = vel;
	bd.angularVelocity = omega;
	b3BodyId body = b3CreateBody( fw->worldId, &bd );

	for ( int i = 0; i < count; ++i )
	{
		b3FractureVoxel* vx = b3F_vox( fw, voxels[i] );
		vx->local = b3InvRotateVector( rot, b3Sub( wpos[i], com ) );
		vx->piece = p;
	}
	b3Vec3 localOffset;
	{
		b3FractureVoxel* v0 = b3F_vox( fw, voxels[0] );
		b3Vec3 base = { v0->cell.x * fw->voxel, v0->cell.y * fw->voxel, v0->cell.z * fw->voxel };
		localOffset = b3Sub( v0->local, base );
	}

	b3FracturePiece* P = fw->pieces.data + p;
	b3Array_Clear( P->voxels );
	b3Array_Append( P->voxels, voxels, count );
	P->body = body;
	P->localOffset = localOffset;
	P->isStatic = (uint8_t)isStatic;
	P->debris = (uint8_t)debris;
	P->merge = (uint8_t)merge;
	P->active = 1;
	P->peakApproach = 0.0f;
	P->overloadStreak = 0;

	if ( !merge )
	{
		for ( int i = 0; i < count; ++i )
		{
			b3FractureVoxel* vx = b3F_vox( fw, voxels[i] );
			b3FractureMaterial* mat = fw->materials.data + vx->mat;
			b3ShapeDef sd = b3DefaultShapeDef();
			sd.density = mat->density;
			sd.baseMaterial.friction = mat->friction;
			sd.baseMaterial.restitution = mat->restitution;
			sd.baseMaterial.customColor = mat->color ? mat->color : 1u;
			sd.enableContactEvents = true;
			b3BoxHull hull = b3MakeOffsetBoxHull( fw->radius, fw->radius, fw->radius, vx->local );
			vx->shape = b3CreateHullShape( body, &sd, &hull.base );
		}
		return;
	}

	b3F_Map local;
	memset( &local, 0, sizeof( local ) );
	for ( int i = 0; i < count; ++i )
		b3F_mapPut( &local, b3F_packCell( b3F_vox( fw, voxels[i] )->cell ), i );

	uint8_t* covered = (uint8_t*)b3AllocZeroed( (size_t)count );
	int* order = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	for ( int i = 0; i < count; ++i )
		order[i] = i;
	for ( int a = 1; a < count; ++a ) // insertion sort (count is modest per body)
	{
		int key = order[a], j = a - 1;
		b3Vec3i ck = b3F_vox( fw, voxels[key] )->cell;
		while ( j >= 0 )
		{
			b3Vec3i cj = b3F_vox( fw, voxels[order[j]] )->cell;
			bool greater = ( cj.z > ck.z ) || ( cj.z == ck.z && cj.y > ck.y ) ||
						   ( cj.z == ck.z && cj.y == ck.y && cj.x > ck.x );
			if ( !greater )
				break;
			order[j + 1] = order[j];
			j--;
		}
		order[j + 1] = key;
	}

	for ( int oi = 0; oi < count; ++oi )
	{
		int seed = order[oi];
		if ( covered[seed] )
			continue;
		int matI = b3F_vox( fw, voxels[seed] )->mat;
		b3Vec3i c0 = b3F_vox( fw, voxels[seed] )->cell;

#define B3F_OCC( CX, CY, CZ, OUT )                                                                                      \
	do                                                                                                                 \
	{                                                                                                                  \
		b3Vec3i _c = { ( CX ), ( CY ), ( CZ ) };                                                                        \
		int _j = b3F_mapGet( &local, b3F_packCell( _c ) );                                                              \
		OUT = ( _j >= 0 && !covered[_j] && b3F_vox( fw, voxels[_j] )->mat == matI ) ? _j : -1;                          \
	} while ( 0 )

		int ex = c0.x, tmp;
		for ( ;; )
		{
			B3F_OCC( ex + 1, c0.y, c0.z, tmp );
			if ( tmp < 0 )
				break;
			ex++;
		}
		int ey = c0.y;
		for ( bool grow = true; grow; )
		{
			for ( int x = c0.x; x <= ex; ++x )
			{
				B3F_OCC( x, ey + 1, c0.z, tmp );
				if ( tmp < 0 )
				{
					grow = false;
					break;
				}
			}
			if ( grow )
				ey++;
		}
		int ez = c0.z;
		for ( bool grow = true; grow; )
		{
			for ( int x = c0.x; x <= ex && grow; ++x )
				for ( int y = c0.y; y <= ey; ++y )
				{
					B3F_OCC( x, y, ez + 1, tmp );
					if ( tmp < 0 )
					{
						grow = false;
						break;
					}
				}
			if ( grow )
				ez++;
		}

		b3Vec3 he = { 0.5f * ( ex - c0.x + 1 ) * fw->voxel, 0.5f * ( ey - c0.y + 1 ) * fw->voxel,
					  0.5f * ( ez - c0.z + 1 ) * fw->voxel };
		b3Vec3 off = { 0.5f * ( c0.x + ex ) * fw->voxel + localOffset.x, 0.5f * ( c0.y + ey ) * fw->voxel + localOffset.y,
					   0.5f * ( c0.z + ez ) * fw->voxel + localOffset.z };
		b3FractureMaterial* mat = fw->materials.data + matI;
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = mat->density;
		sd.baseMaterial.friction = mat->friction;
		sd.baseMaterial.restitution = mat->restitution;
		sd.baseMaterial.customColor = mat->color ? mat->color : 1u;
		sd.enableContactEvents = true;
		b3BoxHull hull = b3MakeOffsetBoxHull( he.x, he.y, he.z, off );
		b3ShapeId shape = b3CreateHullShape( body, &sd, &hull.base );

		for ( int x = c0.x; x <= ex; ++x )
			for ( int y = c0.y; y <= ey; ++y )
				for ( int z = c0.z; z <= ez; ++z )
				{
					b3Vec3i cc = { x, y, z };
					int j = b3F_mapGet( &local, b3F_packCell( cc ) );
					covered[j] = 1;
					b3F_vox( fw, voxels[j] )->shape = shape;
				}
#undef B3F_OCC
	}

	b3Free( order, (size_t)count * sizeof( int ) );
	b3Free( covered, (size_t)count );
	b3F_mapFree( &local );
}

static int b3F_internMaterial( b3FractureWorld* fw, b3FractureMaterial m )
{
	b3Array_Push( fw->materials, m );
	return fw->materials.count - 1;
}

static int b3F_addBody( b3FractureWorld* fw, const b3Vec3i* cells, int count, b3FractureMaterial material,
						const b3FractureDef* def )
{
	int matIdx = b3F_internMaterial( fw, material );

	int* idx = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	int kept = 0;
	bool anyAnchor = false;
	int startVox = fw->voxels.count; // voxels added below start here (piece is still -1 during this call)
	for ( int i = 0; i < count; ++i )
	{
		uint64_t key = b3F_packCell( cells[i] );
		int existing = b3F_mapGet( &fw->cellToVox, key );
		if ( existing >= 0 && ( b3F_vox( fw, existing )->piece >= 0 || existing >= startVox ) )
			continue;

		bool a = def->anchor ? def->anchor( cells[i], def->anchorContext ) : def->isStatic;
		anyAnchor = anyAnchor || a;

		b3FractureVoxel vx = { 0 };
		vx.cell = cells[i];
		vx.mat = matIdx;
		vx.piece = -1;
		vx.shape = (b3ShapeId){ 0 };
		vx.restOn = -1;
		vx.anchor = a ? 1 : 0;
		int id = fw->voxels.count;
		b3Array_Push( fw->voxels, vx );
		b3F_mapPut( &fw->cellToVox, key, id );
		idx[kept++] = id;
	}
	if ( kept == 0 )
	{
		b3Free( idx, (size_t)count * sizeof( int ) );
		return -1;
	}

	int p = b3F_allocPiece( fw );
	b3Vec3* wpos = (b3Vec3*)b3Alloc( (size_t)kept * sizeof( b3Vec3 ) );
	for ( int i = 0; i < kept; ++i )
	{
		b3Vec3i c = b3F_vox( fw, idx[i] )->cell;
		wpos[i] = (b3Vec3){ ( c.x + 0.5f ) * fw->voxel, ( c.y + 0.5f ) * fw->voxel, ( c.z + 0.5f ) * fw->voxel };
	}
	bool bodyStatic = def->isStatic || anyAnchor;
	b3F_formBody( fw, p, idx, kept, wpos, b3Quat_identity, def->velocity, b3Vec3_zero, bodyStatic, false, def->merge );

	b3Free( wpos, (size_t)kept * sizeof( b3Vec3 ) );
	b3Free( idx, (size_t)count * sizeof( int ) );
	return p;
}

static void b3F_destroyVoxels( b3FractureWorld* fw, const int* vox, int count )
{
	for ( int i = 0; i < count; ++i )
		b3F_vox( fw, vox[i] )->piece = -1;
}

static void b3F_destroyPieceBody( b3FractureWorld* fw, int p )
{
	b3FracturePiece* P = fw->pieces.data + p;
	if ( P->active && b3Body_IsValid( P->body ) )
		b3DestroyBody( P->body );
	P->active = 0;
	b3Array_Clear( P->voxels );
}

static int b3F_components( b3FractureWorld* fw, const int* vox, int count, int piece, int* label, int* stack, b3F_Map* id2local )
{
	for ( int i = 0; i < count; ++i )
	{
		label[i] = -1;
		b3F_mapPut( id2local, b3F_packCell( b3F_vox( fw, vox[i] )->cell ), i );
	}
	int comp = 0;
	for ( int s = 0; s < count; ++s )
	{
		if ( label[s] >= 0 )
			continue;
		int sp = 0;
		stack[sp++] = s;
		label[s] = comp;
		while ( sp > 0 )
		{
			int li = stack[--sp];
			int v = vox[li];
			for ( int f = 0; f < 6; ++f )
			{
				int nv = b3F_neighbor( fw, v, f, piece );
				if ( nv < 0 )
					continue;
				int nl = b3F_mapGet( id2local, b3F_packCell( b3F_vox( fw, nv )->cell ) );
				if ( nl >= 0 && label[nl] < 0 )
				{
					label[nl] = comp;
					stack[sp++] = nl;
				}
			}
		}
		comp++;
	}
	return comp;
}

static void b3F_splitPiece( b3FractureWorld* fw, int piece )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	int count = P->voxels.count;
	if ( count == 0 )
	{
		b3F_destroyPieceBody( fw, piece );
		return;
	}
	int* vox = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	memcpy( vox, P->voxels.data, (size_t)count * sizeof( int ) );

	int* label = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	int* stack = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	b3F_Map id2local;
	memset( &id2local, 0, sizeof( id2local ) );
	int nComp = b3F_components( fw, vox, count, piece, label, stack, &id2local );
	b3F_mapFree( &id2local );

	int* sizes = (int*)b3AllocZeroed( (size_t)nComp * sizeof( int ) );
	for ( int i = 0; i < count; ++i )
		sizes[label[i]]++;

	int minFrag = fw->tuning.minFragment;

	int keepComps = 0;
	for ( int c = 0; c < nComp; ++c )
		if ( sizes[c] >= minFrag )
			keepComps++;

	if ( keepComps == 0 )
	{
		b3F_destroyVoxels( fw, vox, count );
		b3F_destroyPieceBody( fw, piece );
		b3Free( sizes, (size_t)nComp * sizeof( int ) );
		b3Free( stack, (size_t)count * sizeof( int ) );
		b3Free( label, (size_t)count * sizeof( int ) );
		b3Free( vox, (size_t)count * sizeof( int ) );
		return;
	}
	if ( keepComps == 1 && nComp == 1 )
	{
		b3Free( sizes, (size_t)nComp * sizeof( int ) );
		b3Free( stack, (size_t)count * sizeof( int ) );
		b3Free( label, (size_t)count * sizeof( int ) );
		b3Free( vox, (size_t)count * sizeof( int ) );
		return;
	}

	b3WorldTransform xf = b3Body_GetTransform( P->body );
	b3Vec3 pvel = b3Body_GetLinearVelocity( P->body );
	b3Vec3 pomega = b3Body_GetAngularVelocity( P->body );
	b3Pos pcom = b3Body_GetWorldCenter( P->body );
	b3Quat prot = xf.q;
	bool pdebris = P->debris != 0;
	bool pmerge = P->merge != 0;

	b3Vec3* wp = (b3Vec3*)b3Alloc( (size_t)count * sizeof( b3Vec3 ) );
	for ( int i = 0; i < count; ++i )
		wp[i] = b3F_worldPosLocal( xf, b3F_vox( fw, vox[i] )->local );

	int* scratch = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	int tinyCount = 0;
	for ( int i = 0; i < count; ++i )
		if ( sizes[label[i]] < minFrag )
			scratch[tinyCount++] = vox[i];
	if ( tinyCount > 0 )
		b3F_destroyVoxels( fw, scratch, tinyCount );

	int* compOrder = (int*)b3Alloc( (size_t)nComp * sizeof( int ) );
	int nOrder = 0;
	for ( int c = 0; c < nComp; ++c )
		if ( sizes[c] >= minFrag )
			compOrder[nOrder++] = c;
	for ( int a = 1; a < nOrder; ++a ) // insertion sort by size desc (stable enough)
	{
		int key = compOrder[a], j = a - 1;
		while ( j >= 0 && sizes[compOrder[j]] < sizes[key] )
		{
			compOrder[j + 1] = compOrder[j];
			j--;
		}
		compOrder[j + 1] = key;
	}

	b3F_destroyPieceBody( fw, piece );

	b3Vec3* cwpos = (b3Vec3*)b3Alloc( (size_t)count * sizeof( b3Vec3 ) );
	for ( int ci = 0; ci < nOrder; ++ci )
	{
		int comp = compOrder[ci];
		int n = 0;
		double M = 0.0;
		b3Vec3 com = b3Vec3_zero;
		bool childStatic = false;
		for ( int i = 0; i < count; ++i )
		{
			if ( label[i] != comp )
				continue;
			scratch[n] = vox[i];
			cwpos[n] = wp[i];
			float m = b3F_voxelMass( fw, vox[i] );
			M += m;
			com = b3MulAdd( com, m, wp[i] );
			if ( b3F_vox( fw, vox[i] )->anchor )
				childStatic = true;
			n++;
		}
		com = b3MulSV( 1.0f / (float)M, com );
		b3Vec3 vel = b3Add( pvel, b3Cross( pomega, b3Sub( com, b3ToVec3( pcom ) ) ) );
		int target = ( ci == 0 ) ? piece : b3F_allocPiece( fw );
		P = NULL; // pieces array may have grown; do not use stale P below
		b3F_formBody( fw, target, scratch, n, cwpos, prot, vel, pomega, childStatic, pdebris, pmerge );
		if ( !childStatic && n == 1 )
			b3Array_Push( fw->debrisQueue, target ); // candidate for the tuning.maxDebris cap
	}

	b3Free( cwpos, (size_t)count * sizeof( b3Vec3 ) );
	b3Free( compOrder, (size_t)nComp * sizeof( int ) );
	b3Free( scratch, (size_t)count * sizeof( int ) );
	b3Free( wp, (size_t)count * sizeof( b3Vec3 ) );
	b3Free( sizes, (size_t)nComp * sizeof( int ) );
	b3Free( stack, (size_t)count * sizeof( int ) );
	b3Free( label, (size_t)count * sizeof( int ) );
	b3Free( vox, (size_t)count * sizeof( int ) );
}

static int b3F_chunkAt( b3FractureWorld* fw, int p, b3Pos worldPt ); // defined in the chunk backend below

static void b3F_gatherPiece( b3FractureWorld* fw, int p, int worker, float invDt, float a, b3Vec3 gUp )
{
	b3FracturePiece* P = fw->pieces.data + p;
	if ( !P->active )
		return;
	if ( P->kind == b3F_kindChunk && P->voxels.count < 2 )
		return;
	if ( P->isStatic == 0 && b3Body_IsAwake( P->body ) == false )
		return;
	int cap = b3Body_GetContactCapacity( P->body );
	if ( cap <= 0 )
		return;
	b3ContactData* buf = (b3ContactData*)b3F_scratchW( fw, worker, B3F_SC_CONTACTS, (size_t)cap * sizeof( b3ContactData ) );
	int n = b3Body_GetContactData( P->body, buf, cap );
	b3Pos ourCom = b3Body_GetWorldCenter( P->body );
	for ( int k = 0; k < n; ++k )
	{
		b3ContactData* cd = buf + k;
		b3BodyId bodyA = b3Shape_GetBody( cd->shapeIdA );
		bool weAreA = B3_ID_EQUALS( bodyA, P->body );
		b3BodyId other = weAreA ? b3Shape_GetBody( cd->shapeIdB ) : bodyA;
		bool otherStatic = b3Body_GetType( other ) == b3_staticBody;
		int otherKey = otherStatic ? -2 : (int)other.index1; // distinct dynamic body id
		for ( int m = 0; m < cd->manifoldCount; ++m )
		{
			const b3Manifold* man = cd->manifolds + m;
			b3Vec3 normal = man->normal; // A -> B
			float sign = weAreA ? -1.0f : 1.0f;
			for ( int pt = 0; pt < man->pointCount; ++pt )
			{
				const b3ManifoldPoint* mp = man->points + pt;
				if ( mp->normalVelocity < -P->peakApproach )
					P->peakApproach = -mp->normalVelocity;
				if ( mp->totalNormalImpulse <= 0.0f )
					continue;
				float mag = mp->totalNormalImpulse * invDt;
				b3Vec3 Ffull = b3MulSV( sign * mag, normal );
				b3Vec3 anchor = weAreA ? mp->anchorA : mp->anchorB;
				b3Pos worldPt = b3OffsetPos( ourCom, anchor );
				if ( P->kind == b3F_kindChunk )
				{
					int c = b3F_chunkAt( fw, p, worldPt );
					if ( c < 0 )
						continue;
					b3F_Chunk* ch = fw->chunks.data + c;
					ch->force = b3Add( ch->force, b3MulSV( a, Ffull ) );
					ch->forceRaw = b3Add( ch->forceRaw, Ffull );
					if ( b3Dot( Ffull, gUp ) > 0.0f )
						ch->restOn = otherKey;
					continue;
				}
				int v = b3F_voxelAt( fw, p, worldPt );
				if ( v < 0 )
					continue;
				b3FractureVoxel* vx = b3F_vox( fw, v );
				vx->force = b3Add( vx->force, b3MulSV( a, Ffull ) );
				vx->forceRaw = b3Add( vx->forceRaw, Ffull );
				if ( b3Dot( Ffull, gUp ) > 0.0f )
					vx->restOn = otherKey;
			}
		}
	}
}

typedef struct b3F_GatherCtx
{
	b3FractureWorld* fw;
	float invDt;
	float a;
	b3Vec3 gUp;
} b3F_GatherCtx;

static void b3F_gatherTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b3F_GatherCtx* ctx = (b3F_GatherCtx*)context;
	for ( int p = startIndex; p < endIndex; ++p )
		b3F_gatherPiece( ctx->fw, p, workerIndex, ctx->invDt, ctx->a, ctx->gUp );
}

static void b3F_gatherContactForces( b3World* world, b3FractureWorld* fw, float invDt )
{
	for ( int p = 0; p < fw->pieces.count; ++p )
		fw->pieces.data[p].peakApproach = 0.0f;
	for ( int v = 0; v < fw->voxels.count; ++v )
		if ( fw->voxels.data[v].piece >= 0 )
		{
			fw->voxels.data[v].restOn = -1;
			fw->voxels.data[v].forceRaw = b3Vec3_zero;
		}
	for ( int c = 0; c < fw->chunks.count; ++c )
		if ( fw->chunks.data[c].piece >= 0 )
		{
			fw->chunks.data[c].restOn = -1;
			fw->chunks.data[c].forceRaw = b3Vec3_zero;
		}
	if ( !fw->tuning.contactStress )
	{
		for ( int v = 0; v < fw->voxels.count; ++v )
			if ( fw->voxels.data[v].piece >= 0 )
				fw->voxels.data[v].force = b3Vec3_zero;
		for ( int c = 0; c < fw->chunks.count; ++c )
			if ( fw->chunks.data[c].piece >= 0 )
				fw->chunks.data[c].force = b3Vec3_zero;
		return;
	}

	float gmag = b3Length( fw->gravity );
	b3Vec3 gUp = ( gmag > B3F_EPS ) ? b3MulSV( -1.0f / gmag, fw->gravity ) : (b3Vec3){ 0, 1, 0 };
	float a = fw->tuning.contactSmoothing;
	if ( a <= 0.0f || a > 1.0f )
		a = 1.0f;

	for ( int v = 0; v < fw->voxels.count; ++v )
		if ( fw->voxels.data[v].piece >= 0 )
			fw->voxels.data[v].force = b3MulSV( 1.0f - a, fw->voxels.data[v].force );
	for ( int c = 0; c < fw->chunks.count; ++c )
		if ( fw->chunks.data[c].piece >= 0 )
			fw->chunks.data[c].force = b3MulSV( 1.0f - a, fw->chunks.data[c].force );

	if ( fw->tuning.parallelAnalysis && world->workerCount > 1 )
	{
		b3F_GatherCtx ctx = { fw, invDt, a, gUp };
		b3ParallelFor( world, b3F_gatherTask, fw->pieces.count, 16, &ctx, "FractureGather" );
	}
	else
	{
		for ( int p = 0; p < fw->pieces.count; ++p )
			b3F_gatherPiece( fw, p, 0, invDt, a, gUp );
	}
}

static bool b3F_impactFracture( b3FractureWorld* fw, int piece )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	if ( P->peakApproach <= fw->tuning.impactSpeed )
		return false;
	float invA = 1.0f / ( fw->voxel * fw->voxel );
	float overload = 1.0f;
	int count = P->voxels.count;
	int* voxbuf = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	memcpy( voxbuf, P->voxels.data, (size_t)count * sizeof( int ) );

	uint8_t* isSite = (uint8_t*)b3AllocZeroed( (size_t)fw->voxels.count );
	int nSites = 0;
	for ( int i = 0; i < count; ++i )
	{
		int v = voxbuf[i];
		b3FractureVoxel* vx = b3F_vox( fw, v );
		float strength = fw->materials.data[vx->mat].strength * fw->tuning.strengthScale * fw->tuning.impactBearing;
		if ( strength <= B3F_EPS )
			continue;
		float bearing = b3Length( vx->forceRaw ) * invA;
		if ( bearing > strength )
		{
			isSite[v] = 1;
			nSites++;
			overload = b3F_max( overload, bearing / strength );
		}
	}
	if ( nSites == 0 )
	{
		b3Free( isSite, (size_t)fw->voxels.count );
		b3Free( voxbuf, (size_t)count * sizeof( int ) );
		return false;
	}
	uint64_t profTicks = b3GetTicks(); // the impact response (BFS + sever + split) is sever time

	int radius = fw->tuning.impactRadius + (int)b3F_min( 2.0f, overload - 1.0f );
	int* frontier = (int*)b3Alloc( (size_t)fw->voxels.count * sizeof( int ) );
	int* nextf = (int*)b3Alloc( (size_t)fw->voxels.count * sizeof( int ) );
	int fn = 0;
	for ( int i = 0; i < count; ++i )
		if ( isSite[voxbuf[i]] )
			frontier[fn++] = voxbuf[i];
	for ( int ring = 0; ring < radius && fn > 0; ++ring )
	{
		int nn = 0;
		for ( int i = 0; i < fn; ++i )
		{
			int v = frontier[i];
			for ( int f = 0; f < 6; ++f )
			{
				int nv = b3F_neighbor( fw, v, f, piece );
				if ( nv >= 0 && !isSite[nv] )
				{
					isSite[nv] = 1;
					nextf[nn++] = nv;
				}
			}
		}
		int* t = frontier;
		frontier = nextf;
		nextf = t;
		fn = nn;
	}

	for ( int i = 0; i < count; ++i )
	{
		int v = voxbuf[i];
		if ( !isSite[v] )
			continue;
		for ( int f = 0; f < 6; ++f )
		{
			b3FractureVoxel* vx = b3F_vox( fw, v );
			b3Vec3i nc = { vx->cell.x + FACE[f][0], vx->cell.y + FACE[f][1], vx->cell.z + FACE[f][2] };
			int nv = b3F_mapGet( &fw->cellToVox, b3F_packCell( nc ) );
			if ( nv >= 0 && b3F_vox( fw, nv )->piece == piece && !isSite[nv] )
				b3F_sever( fw, v, f );
		}
	}

	b3Free( nextf, (size_t)fw->voxels.count * sizeof( int ) );
	b3Free( frontier, (size_t)fw->voxels.count * sizeof( int ) );
	b3Free( isSite, (size_t)fw->voxels.count );
	b3Free( voxbuf, (size_t)count * sizeof( int ) );

	b3F_splitPiece( fw, piece );
	fw->profSeverMs += b3GetMilliseconds( profTicks );
	return true;
}

typedef struct b3F_Bond
{
	int lo, hi;
	b3Vec3 mid;
} b3F_Bond;

static int b3F_sectionBonds( b3FractureWorld* fw, const int* vox, int count, int axis, int b, b3WorldTransform xf,
							 b3F_Bond* out )
{
	int off[3] = { 0, 0, 0 };
	off[axis] = 1;
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		int v = vox[i];
		b3FractureVoxel* vx = b3F_vox( fw, v );
		if ( b3F_iaxis( vx->cell, axis ) != b )
			continue;
		int face = ( axis << 1 ); // +axis face index (0,2,4)
		if ( ( vx->brokenFaces >> face ) & 1u )
			continue;
		b3Vec3i nc = { vx->cell.x + off[0], vx->cell.y + off[1], vx->cell.z + off[2] };
		int nv = b3F_mapGet( &fw->cellToVox, b3F_packCell( nc ) );
		if ( nv < 0 || b3F_vox( fw, nv )->piece != vx->piece )
			continue;
		b3Vec3 pv = b3F_worldPosLocal( xf, vx->local );
		b3Vec3 pn = b3F_worldPosLocal( xf, b3F_vox( fw, nv )->local );
		out[n].lo = v;
		out[n].hi = nv;
		out[n].mid = b3MulSV( 0.5f, b3Add( pv, pn ) );
		n++;
	}
	return n;
}

static int b3F_sectionBondsCut( b3FractureWorld* fw, const int* vox, int nv, const int* binVox, const int* binOff,
								int cmin, int axis, int b, b3WorldTransform xf, b3F_Bond* out )
{
	if ( binVox != NULL )
	{
		int base = binOff[b - cmin];
		return b3F_sectionBonds( fw, binVox + base, binOff[b - cmin + 1] - base, axis, b, xf, out );
	}
	return b3F_sectionBonds( fw, vox, nv, axis, b, xf, out );
}

static float b3F_sectionStrength( b3FractureWorld* fw, const b3F_Bond* bonds, int nb, float* cfacOut )
{
	float strength = 1e30f, cfac = 1.0f;
	for ( int k = 0; k < nb; ++k )
	{
		int ids[2] = { bonds[k].lo, bonds[k].hi };
		for ( int e = 0; e < 2; ++e )
		{
			b3FractureMaterial* mat = fw->materials.data + b3F_vox( fw, ids[e] )->mat;
			if ( mat->strength < strength )
			{
				strength = mat->strength;
				cfac = mat->compressiveFactor;
			}
		}
	}
	*cfacOut = cfac;
	return strength * fw->tuning.strengthScale;
}

typedef struct b3F_Decision
{
	int axis;	   // failing cut axis (chunk and voxel kinds)
	float cut;	   // chunk kind: world coordinate of the cut plane along axis
	int cellB;	   // voxel kind: cell layer of the cut
	int freeDir;   // voxel kind: which side is unsupported (+1 / -1 / 0)
	bool fracture; // sever now
} b3F_Decision;

static int b3F_stride( const b3FractureWorld* fw )
{
	int s = fw->tuning.analysisStride;
	return s > 1 ? s : 1;
}

static bool b3F_analysisEligible( const b3FractureWorld* fw, const b3FracturePiece* P )
{
	if ( P->isStatic )
		return true;
	if ( P->peakApproach > fw->tuning.impactSpeed )
		return true;
	float ls = b3Length( b3Body_GetLinearVelocity( P->body ) );
	float as = b3Length( b3Body_GetAngularVelocity( P->body ) );
	return ls < fw->tuning.settleSpeed && as < fw->tuning.settleSpeed;
}

static void b3F_analyzePiece( b3FractureWorld* fw, int worker, int piece, bool doFracture, b3F_Decision* dec )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	dec->fracture = false;
	int nv = P->voxels.count;
	for ( int i = 0; i < nv; ++i )
	{
		b3FractureVoxel* vx = b3F_vox( fw, P->voxels.data[i] );
		vx->stressShown += 0.15f * ( vx->stress - vx->stressShown );
		vx->stress = 0.0f;
	}
	if ( nv < 2 )
		return;
	if ( !b3F_analysisEligible( fw, P ) )
		return; // overloadStreak freezes until the body can fracture again

	float gmag = b3Length( fw->gravity );
	b3Vec3 up = ( gmag > B3F_EPS ) ? b3MulSV( -1.0f / gmag, fw->gravity ) : (b3Vec3){ 0, 1, 0 };
	b3WorldTransform xf = b3Body_GetTransform( P->body );

	int* vox = (int*)b3F_scratchW( fw, worker, B3F_SC_V_VOX, (size_t)nv * sizeof( int ) );
	memcpy( vox, P->voxels.data, (size_t)nv * sizeof( int ) );
	b3Vec3* wp = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_WP, (size_t)nv * sizeof( b3Vec3 ) );
	float* mass = (float*)b3F_scratchW( fw, worker, B3F_SC_V_MASS, (size_t)nv * sizeof( float ) );
	b3Vec3* fext = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_FEXT, (size_t)nv * sizeof( b3Vec3 ) );
	char* support = (char*)b3F_scratchW( fw, worker, B3F_SC_V_SUPPORT, (size_t)nv );

	for ( int i = 0; i < nv; ++i )
	{
		b3FractureVoxel* vx = b3F_vox( fw, vox[i] );
		wp[i] = b3F_worldPosLocal( xf, vx->local );
		float m = b3F_voxelMass( fw, vox[i] );
		mass[i] = m;
		b3Vec3 load = vx->force;
		float upc = b3Dot( load, up );
		if ( upc > 0.0f )
			load = b3MulSub( load, upc, up );
		fext[i] = b3Add( b3MulSV( m, fw->gravity ), load );
	}

	if ( !P->isStatic )
	{
		float M = 0.0f;
		b3Vec3 Fnet = b3Vec3_zero;
		for ( int i = 0; i < nv; ++i )
		{
			M += mass[i];
			Fnet = b3Add( Fnet, b3Add( b3MulSV( mass[i], fw->gravity ), b3F_vox( fw, vox[i] )->force ) );
		}
		if ( M > B3F_EPS )
		{
			b3Vec3 com = b3ToVec3( b3Body_GetWorldCenter( P->body ) );
			b3Vec3 aLin = b3MulSV( 1.0f / M, Fnet );
			b3Vec3 tau = b3Vec3_zero;
			for ( int i = 0; i < nv; ++i )
			{
				b3Vec3 load = b3Add( b3MulSV( mass[i], fw->gravity ), b3F_vox( fw, vox[i] )->force );
				tau = b3Add( tau, b3Cross( b3Sub( wp[i], com ), load ) );
			}
			b3MassData md = b3Body_GetMassData( P->body );
			b3Vec3 alpha = b3RotateVector( xf.q, b3Solve3( md.inertia, b3InvRotateVector( xf.q, tau ) ) );
			for ( int i = 0; i < nv; ++i )
			{
				b3Vec3 acc = b3Add( aLin, b3Cross( alpha, b3Sub( wp[i], com ) ) );
				fext[i] = b3MulSub( fext[i], mass[i], acc );
			}
		}
	}

	int nsup = 0;
	bool fixedSupport = false;
	int spanKeys[16];
	int spanCount = 0;
	float tol = 0.25f * fw->voxel;
	for ( int i = 0; i < nv; ++i )
	{
		b3FractureVoxel* vx = b3F_vox( fw, vox[i] );
		bool s = false;
		if ( vx->anchor )
		{
			s = true;
			fixedSupport = true;
		}
		else if ( wp[i].y - fw->radius <= fw->groundY + tol )
		{
			s = true;
			fixedSupport = true;
		}
		else if ( vx->restOn == -2 )
		{
			s = true;
			fixedSupport = true;
		}
		else if ( vx->restOn >= 0 )
		{
			s = true;
			bool found = false;
			for ( int k = 0; k < spanCount; ++k )
				if ( spanKeys[k] == vx->restOn )
				{
					found = true;
					break;
				}
			if ( !found && spanCount < 16 )
				spanKeys[spanCount++] = vx->restOn;
		}
		support[i] = s ? 1 : 0;
		nsup += s ? 1 : 0;
	}

	if ( nsup == 0 || nsup == nv )
		return;
	{
		bool span = !fixedSupport && spanCount >= 2;

		const float A_cell = fw->voxel * fw->voxel;
		float bestFail = 1.0f;
		int bestAxis = -1, bestB = 0, bestFreeDir = 0;
		b3F_Bond* bonds = (b3F_Bond*)b3F_scratchW( fw, worker, B3F_SC_V_BONDS, (size_t)nv * sizeof( b3F_Bond ) );
		b3Vec3 ref = wp[0]; // a member position: keeps every moment lever arm piece-sized

		for ( int axis = 0; axis < 3; ++axis )
		{
			b3Vec3 e = b3Vec3_zero;
			( &e.x )[axis] = 1.0f;
			b3Vec3 normal = b3RotateVector( xf.q, e );
			int cmin = INT_MAX, cmax = INT_MIN;
			for ( int i = 0; i < nv; ++i )
			{
				int c = b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis );
				if ( c < cmin )
					cmin = c;
				if ( c > cmax )
					cmax = c;
			}

			int range = cmax - cmin + 1;
			const int* binVox = NULL;
			const int* binOff = NULL;
			const int* supPre = NULL; // integer partition prefixes (bit-identical to the direct scan)
			const b3Vec3* preF = NULL; // per-bin running sums for the free-body cut (reordered summation)
			const b3Vec3* preCross = NULL;
			const b3Vec3* preMP = NULL;
			const float* preM = NULL;
			const b3Vec3* sufF = NULL; // and the same summed from the high end, for a high free side
			const b3Vec3* sufCross = NULL;
			const b3Vec3* sufMP = NULL;
			const float* sufM = NULL;
			if ( range >= 1 && range <= 4 * nv + 64 )
			{
				int* off = (int*)b3F_scratchW( fw, worker, B3F_SC_V_BINOFF, (size_t)( range + 1 ) * sizeof( int ) );
				memset( off, 0, (size_t)( range + 1 ) * sizeof( int ) );
				for ( int i = 0; i < nv; ++i )
					off[b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) - cmin + 1]++;
				for ( int c = 0; c < range; ++c )
					off[c + 1] += off[c];
				int* order = (int*)b3F_scratchW( fw, worker, B3F_SC_V_ORDER, (size_t)nv * sizeof( int ) );
				int* cursor = (int*)b3F_scratchW( fw, worker, B3F_SC_V_CURSOR, (size_t)range * sizeof( int ) );
				for ( int c = 0; c < range; ++c )
					cursor[c] = off[c];
				for ( int i = 0; i < nv; ++i )
				{
					int c = b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) - cmin;
					order[cursor[c]++] = vox[i];
				}
				binVox = order;
				binOff = off;

				if ( !span )
				{
					int* sp = (int*)b3F_scratchW( fw, worker, B3F_SC_V_SUPPRE, (size_t)( range + 1 ) * sizeof( int ) );
					b3Vec3* pf = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_PREF, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					b3Vec3* pc = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_PRECROSS, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					b3Vec3* pm = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_PREMP, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					float* pw = (float*)b3F_scratchW( fw, worker, B3F_SC_V_PREM, (size_t)( range + 1 ) * sizeof( float ) );
					b3Vec3* sf = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_SUFF, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					b3Vec3* sc = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_SUFCROSS, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					b3Vec3* sm = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_V_SUFMP, (size_t)( range + 1 ) * sizeof( b3Vec3 ) );
					float* sw = (float*)b3F_scratchW( fw, worker, B3F_SC_V_SUFM, (size_t)( range + 1 ) * sizeof( float ) );
					for ( int c = 0; c <= range; ++c )
					{
						sp[c] = 0;
						sf[c] = b3Vec3_zero; // raw per-bin sums first; suffixed in place below
						sc[c] = b3Vec3_zero;
						sm[c] = b3Vec3_zero;
						sw[c] = 0.0f;
					}
					for ( int i = 0; i < nv; ++i )
					{
						int c = b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) - cmin;
						sp[c + 1] += support[i];
						sf[c] = b3Add( sf[c], fext[i] );
						sc[c] = b3Add( sc[c], b3Cross( b3Sub( wp[i], ref ), fext[i] ) );
						sm[c] = b3MulAdd( sm[c], mass[i], b3Sub( wp[i], ref ) );
						sw[c] += mass[i];
					}
					pf[0] = b3Vec3_zero;
					pc[0] = b3Vec3_zero;
					pm[0] = b3Vec3_zero;
					pw[0] = 0.0f;
					for ( int c = 0; c < range; ++c )
					{
						sp[c + 1] += sp[c];
						pf[c + 1] = b3Add( pf[c], sf[c] );
						pc[c + 1] = b3Add( pc[c], sc[c] );
						pm[c + 1] = b3Add( pm[c], sm[c] );
						pw[c + 1] = pw[c] + sw[c];
					}
					for ( int c = range - 1; c >= 0; --c )
					{
						sf[c] = b3Add( sf[c], sf[c + 1] );
						sc[c] = b3Add( sc[c], sc[c + 1] );
						sm[c] = b3Add( sm[c], sm[c + 1] );
						sw[c] += sw[c + 1];
					}
					supPre = sp;
					preF = pf;
					preCross = pc;
					preMP = pm;
					preM = pw;
					sufF = sf;
					sufCross = sc;
					sufMP = sm;
					sufM = sw;
				}
			}

			if ( span )
			{
				float aL = 1e30f, aR = -1e30f;
				for ( int i = 0; i < nv; ++i )
					if ( support[i] && b3F_vox( fw, vox[i] )->restOn >= 0 )
					{
						float ax = b3F_faxis( wp[i], axis );
						aL = b3F_min( aL, ax );
						aR = b3F_max( aR, ax );
					}
				if ( aR - aL < 0.75f * fw->voxel )
					continue;
				b3Vec3 gperp = b3MulSub( fw->gravity, b3Dot( fw->gravity, normal ), normal );
				float gpn = b3Length( gperp );
				if ( gpn < B3F_EPS )
					continue;
				b3Vec3 dLoad = b3MulSV( 1.0f / gpn, gperp );

				float* q = (float*)b3F_scratchW( fw, worker, B3F_SC_V_Q, (size_t)nv * sizeof( float ) );
				float W = 0.0f, ac = 0.0f;
				for ( int i = 0; i < nv; ++i )
				{
					float qi = b3Dot( fext[i], dLoad );
					q[i] = qi > 0.0f ? qi : 0.0f;
					W += q[i];
					ac += q[i] * b3F_faxis( wp[i], axis );
				}
				if ( W <= B3F_EPS )
					continue;
				ac /= W;
				float RL = W * ( aR - ac ) / ( aR - aL );

				for ( int b = cmin; b < cmax; ++b )
				{
					int nb = b3F_sectionBondsCut( fw, vox, nv, binVox, binOff, cmin, axis, b, xf, bonds );
					if ( nb == 0 )
						continue;
					b3Vec3 c0 = b3Vec3_zero;
					for ( int k = 0; k < nb; ++k )
						c0 = b3Add( c0, bonds[k].mid );
					c0 = b3MulSV( 1.0f / (float)nb, c0 );
					float bw = b3F_faxis( c0, axis );
					if ( bw <= aL + 0.5f * fw->voxel || bw >= aR - 0.5f * fw->voxel )
						continue;
					float M = RL * ( bw - aL );
					for ( int i = 0; i < nv; ++i )
					{
						float ax = b3F_faxis( wp[i], axis );
						if ( ax < bw )
							M -= q[i] * ( bw - ax );
					}
					float sum_s2 = 0.0f, cext = 0.0f;
					for ( int k = 0; k < nb; ++k )
					{
						float s = b3Dot( b3Sub( bonds[k].mid, c0 ), dLoad );
						sum_s2 += s * s;
						cext = b3F_max( cext, fabsf( s ) );
					}
					float I = A_cell * ( sum_s2 + nb * fw->voxel * fw->voxel / 12.0f );
					float cfac, strength = b3F_sectionStrength( fw, bonds, nb, &cfac );
					if ( strength <= B3F_EPS )
						continue;
					float ratio = fabsf( M ) * ( cext + 0.5f * fw->voxel ) / I / strength;
					for ( int k = 0; k < nb; ++k )
					{
						b3F_vox( fw, bonds[k].lo )->stress = b3F_max( b3F_vox( fw, bonds[k].lo )->stress, ratio );
						b3F_vox( fw, bonds[k].hi )->stress = b3F_max( b3F_vox( fw, bonds[k].hi )->stress, ratio );
					}
					if ( ratio > bestFail )
					{
						bestFail = ratio;
						bestAxis = axis;
						bestB = b;
						bestFreeDir = 0;
					}
				}
				continue;
			}

			for ( int b = cmin; b < cmax; ++b )
			{
				int nlow, nhigh, slow, shigh;
				if ( binOff != NULL && supPre != NULL )
				{
					nlow = binOff[b - cmin + 1]; // voxels with cell coord <= b (exact integer partition)
					nhigh = nv - nlow;
					slow = supPre[b - cmin + 1];
					shigh = nsup - slow;
				}
				else
				{
					nlow = 0;
					nhigh = 0;
					slow = 0;
					shigh = 0;
					for ( int i = 0; i < nv; ++i )
					{
						bool low = b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) <= b;
						if ( low )
						{
							nlow++;
							slow += support[i];
						}
						else
						{
							nhigh++;
							shigh += support[i];
						}
					}
				}
				if ( nlow == 0 || nhigh == 0 )
					continue;
				bool sLow = slow > 0, sHigh = shigh > 0;
				if ( sLow == sHigh )
					continue; // needs exactly one supported side; impacts handled elsewhere
				bool freeIsHigh = !sHigh;
				int freeDir = freeIsHigh ? 1 : -1;

				int nb = b3F_sectionBondsCut( fw, vox, nv, binVox, binOff, cmin, axis, b, xf, bonds );
				if ( nb == 0 )
					continue;
				b3Vec3 c0 = b3Vec3_zero;
				for ( int k = 0; k < nb; ++k )
					c0 = b3Add( c0, bonds[k].mid );
				c0 = b3MulSV( 1.0f / (float)nb, c0 );
				float A = nb * A_cell;

				b3Vec3 F, Mv, comFree;
				float mFree;
				if ( preF != NULL )
				{
					int bi = b - cmin + 1;
					b3Vec3 crossSum, MP;
					if ( freeIsHigh )
					{
						F = sufF[bi];
						crossSum = sufCross[bi];
						MP = sufMP[bi];
						mFree = sufM[bi];
					}
					else
					{
						F = preF[bi];
						crossSum = preCross[bi];
						MP = preMP[bi];
						mFree = preM[bi];
					}
					if ( mFree <= B3F_EPS )
						continue;
					Mv = b3Sub( crossSum, b3Cross( b3Sub( c0, ref ), F ) );
					comFree = b3Add( ref, b3MulSV( 1.0f / mFree, MP ) );
				}
				else
				{
					F = b3Vec3_zero;
					Mv = b3Vec3_zero;
					comFree = b3Vec3_zero;
					mFree = 0.0f;
					for ( int i = 0; i < nv; ++i )
					{
						bool onFree = freeIsHigh ? ( b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) > b )
												 : ( b3F_iaxis( b3F_vox( fw, vox[i] )->cell, axis ) <= b );
						if ( !onFree )
							continue;
						F = b3Add( F, fext[i] );
						Mv = b3Add( Mv, b3Cross( b3Sub( wp[i], c0 ), fext[i] ) );
						comFree = b3MulAdd( comFree, mass[i], wp[i] );
						mFree += mass[i];
					}
					if ( mFree <= B3F_EPS )
						continue;
					comFree = b3MulSV( 1.0f / mFree, comFree );
				}

				float dn = b3Dot( normal, b3Sub( comFree, c0 ) );
				b3Vec3 nOut = b3MulSV( dn < 0.0f ? -1.0f : 1.0f, normal );
				float nAx = b3Dot( F, nOut ) / A;

				b3Vec3 Mplane = b3Sub( Mv, b3MulSV( b3Dot( Mv, normal ), normal ) );
				float Mmag = b3Length( Mplane );
				float bend = 0.0f;
				if ( Mmag > B3F_EPS )
				{
					b3Vec3 d = b3Cross( normal, b3MulSV( 1.0f / Mmag, Mplane ) );
					float sum_s2 = 0.0f, cext = 0.0f;
					for ( int k = 0; k < nb; ++k )
					{
						float s = b3Dot( b3Sub( bonds[k].mid, c0 ), d );
						sum_s2 += s * s;
						cext = b3F_max( cext, fabsf( s ) );
					}
					float I = A_cell * ( sum_s2 + nb * fw->voxel * fw->voxel / 12.0f );
					bend = Mmag * ( cext + 0.5f * fw->voxel ) / I;
				}

				float cfac, strength = b3F_sectionStrength( fw, bonds, nb, &cfac );
				if ( strength <= B3F_EPS )
					continue;
				float tFib = b3F_max( 0.0f, nAx + bend );
				float cFib = b3F_max( 0.0f, bend - nAx );
				float fail = b3F_max( tFib, cFib / b3F_max( 1.0f, cfac ) ) / strength;

				for ( int k = 0; k < nb; ++k )
				{
					b3F_vox( fw, bonds[k].lo )->stress = b3F_max( b3F_vox( fw, bonds[k].lo )->stress, fail );
					b3F_vox( fw, bonds[k].hi )->stress = b3F_max( b3F_vox( fw, bonds[k].hi )->stress, fail );
				}
				if ( fail > bestFail )
				{
					bestFail = fail;
					bestAxis = axis;
					bestB = b;
					bestFreeDir = freeDir;
				}
			}
		}

		if ( bestAxis >= 0 )
			P->overloadStreak += b3F_stride( fw ); // one analysis stands in for K frames of overload
		else
			P->overloadStreak = 0;
		bool impactNow = P->peakApproach > fw->tuning.impactSpeed;
		bool sustained = P->overloadStreak >= fw->tuning.fractureHoldFrames;

		// eligibility was checked on entry, so an overload here may sever
		if ( doFracture && bestAxis >= 0 && ( impactNow || sustained ) )
		{
			dec->axis = bestAxis;
			dec->cellB = bestB;
			dec->freeDir = bestFreeDir;
			dec->fracture = true;
		}
	}
}

static void b3F_severVoxelPiece( b3FractureWorld* fw, int piece, const b3F_Decision* dec )
{
	uint64_t profTicks = b3GetTicks();
	b3FracturePiece* P = fw->pieces.data + piece;
	int nv = P->voxels.count;
	const int* vox = P->voxels.data;
	int axis = dec->axis, b = dec->cellB, freeDir = dec->freeDir;
	int p1 = ( axis + 1 ) % 3, p2 = ( axis + 2 ) % 3;
	const float freq = 0.45f;
	for ( int i = 0; i < nv; ++i )
	{
		int v = vox[i];
		b3FractureVoxel* vx = b3F_vox( fw, v );
		b3Vec3i c = vx->cell;
		int ov = 0;
		if ( fw->tuning.fractureRoughness > 0.0f )
		{
			float nz = b3F_valnoise( b3F_iaxis( c, p1 ) * freq, b3F_iaxis( c, p2 ) * freq );
			if ( freeDir > 0 )
				ov = (int)lroundf( fw->tuning.fractureRoughness * 0.5f * ( nz + 1.0f ) );
			else if ( freeDir < 0 )
				ov = -(int)lroundf( fw->tuning.fractureRoughness * 0.5f * ( nz + 1.0f ) );
			else
				ov = (int)lroundf( fw->tuning.fractureRoughness * nz );
		}
		bool lv = b3F_iaxis( c, axis ) <= b + ov;
		for ( int f = 0; f < 6; ++f )
		{
			int nvid = b3F_mapGet( &fw->cellToVox, b3F_packCell( (b3Vec3i){ c.x + FACE[f][0], c.y + FACE[f][1],
																			   c.z + FACE[f][2] } ) );
			if ( nvid < 0 || b3F_vox( fw, nvid )->piece != piece )
				continue;
			b3Vec3i nc = b3F_vox( fw, nvid )->cell;
			int ov2 = 0;
			if ( fw->tuning.fractureRoughness > 0.0f )
			{
				float nz2 = b3F_valnoise( b3F_iaxis( nc, p1 ) * freq, b3F_iaxis( nc, p2 ) * freq );
				if ( freeDir > 0 )
					ov2 = (int)lroundf( fw->tuning.fractureRoughness * 0.5f * ( nz2 + 1.0f ) );
				else if ( freeDir < 0 )
					ov2 = -(int)lroundf( fw->tuning.fractureRoughness * 0.5f * ( nz2 + 1.0f ) );
				else
					ov2 = (int)lroundf( fw->tuning.fractureRoughness * nz2 );
			}
			bool ln = b3F_iaxis( nc, axis ) <= b + ov2;
			if ( lv != ln )
				b3F_sever( fw, v, f );
		}
	}
	b3F_splitPiece( fw, piece );
	fw->profSeverMs += b3GetMilliseconds( profTicks );
}

#define B3F_MAX_SEEDS 64
#define B3F_MAX_CELL_PLANES 96
#define B3F_MAX_CELL_VERTS 64
#define B3F_NEIGHBOR_SEEDS 24 // bisector planes kept per cell (nearest seeds)

static float b3F_rand01( unsigned* state )
{
	unsigned x = *state ? *state : 0x9e3779b9u;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return ( x & 0xffffff ) / (float)0x1000000;
}

static b3Plane b3F_bisector( b3Vec3 si, b3Vec3 sj )
{
	b3Vec3 n = b3Sub( sj, si );
	float len = b3Length( n );
	b3Plane p;
	if ( len < B3F_EPS )
	{
		p.normal = ( b3Vec3 ){ 1, 0, 0 };
		p.offset = 1e30f;
		return p;
	}
	n = b3MulSV( 1.0f / len, n );
	p.normal = n;
	p.offset = b3Dot( n, b3MulSV( 0.5f, b3Add( si, sj ) ) );
	return p;
}

static bool b3F_triSolve( b3Plane a, b3Plane b, b3Plane c, b3Vec3* out )
{
	b3Vec3 nbc = b3Cross( b.normal, c.normal );
	float det = b3Dot( a.normal, nbc );
	if ( fabsf( det ) < 1e-8f )
		return false;
	b3Vec3 nca = b3Cross( c.normal, a.normal );
	b3Vec3 nab = b3Cross( a.normal, b.normal );
	b3Vec3 num = b3Add( b3Add( b3MulSV( a.offset, nbc ), b3MulSV( b.offset, nca ) ), b3MulSV( c.offset, nab ) );
	*out = b3MulSV( 1.0f / det, num );
	return true;
}

static float b3F_polyAreaCentroid( const b3Vec3* pts, int n, b3Vec3 normal, b3Vec3* centroidOut )
{
	if ( n < 3 )
	{
		*centroidOut = n > 0 ? pts[0] : b3Vec3_zero;
		return 0.0f;
	}
	b3Vec3 ref = fabsf( normal.x ) < 0.9f ? ( b3Vec3 ){ 1, 0, 0 } : ( b3Vec3 ){ 0, 1, 0 };
	b3Vec3 u = b3Normalize( b3MulSub( ref, b3Dot( ref, normal ), normal ) );
	b3Vec3 v = b3Cross( normal, u );
	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
		c = b3Add( c, pts[i] );
	c = b3MulSV( 1.0f / n, c );
	float px[B3F_MAX_CELL_VERTS], py[B3F_MAX_CELL_VERTS], ang[B3F_MAX_CELL_VERTS];
	int idx[B3F_MAX_CELL_VERTS];
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 d = b3Sub( pts[i], c );
		px[i] = b3Dot( d, u );
		py[i] = b3Dot( d, v );
		ang[i] = atan2f( py[i], px[i] );
		idx[i] = i;
	}
	for ( int a = 1; a < n; ++a ) // insertion sort by angle
	{
		int key = idx[a];
		float ka = ang[key];
		int j = a - 1;
		while ( j >= 0 && ang[idx[j]] > ka )
		{
			idx[j + 1] = idx[j];
			j--;
		}
		idx[j + 1] = key;
	}
	float area2 = 0.0f, cx = 0.0f, cy = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		int i0 = idx[i], i1 = idx[( i + 1 ) % n];
		float cross = px[i0] * py[i1] - px[i1] * py[i0];
		area2 += cross;
		cx += ( px[i0] + px[i1] ) * cross;
		cy += ( py[i0] + py[i1] ) * cross;
	}
	float area = 0.5f * fabsf( area2 );
	if ( fabsf( area2 ) > B3F_EPS )
	{
		cx /= ( 3.0f * area2 );
		cy /= ( 3.0f * area2 );
	}
	*centroidOut = b3Add( c, b3Add( b3MulSV( cx, u ), b3MulSV( cy, v ) ) );
	return area;
}

static int b3F_voronoiSlice( b3FractureWorld* fw, const b3HullData* srcHull, int seedCount, int matIdx, unsigned rngSeed,
							 int* outChunkIds )
{
	if ( seedCount > B3F_MAX_SEEDS )
		seedCount = B3F_MAX_SEEDS;
	const b3Vec3* hpoints = b3GetHullPoints( srcHull );
	int hpc = srcHull->vertexCount;
	const b3Plane* hplanes = b3GetHullPlanes( srcHull );
	int hfc = srcHull->faceCount;
	float density = fw->materials.data[matIdx].density;
	if ( hpc == 0 || hfc == 0 )
		return 0;

	b3Vec3 lo = hpoints[0], hi = hpoints[0];
	for ( int i = 1; i < hpc; ++i )
	{
		for ( int a = 0; a < 3; ++a )
		{
			if ( ( &hpoints[i].x )[a] < ( &lo.x )[a] )
				( &lo.x )[a] = ( &hpoints[i].x )[a];
			if ( ( &hpoints[i].x )[a] > ( &hi.x )[a] )
				( &hi.x )[a] = ( &hpoints[i].x )[a];
		}
	}
	float diag = b3Length( b3Sub( hi, lo ) );
	float eps = 1e-4f * ( diag > 1.0f ? diag : 1.0f );

	b3Vec3 seeds[B3F_MAX_SEEDS];
	int ns = 0;
	unsigned st = rngSeed;
	for ( int attempt = 0; attempt < seedCount * 60 && ns < seedCount; ++attempt )
	{
		b3Vec3 p = { lo.x + ( hi.x - lo.x ) * b3F_rand01( &st ), lo.y + ( hi.y - lo.y ) * b3F_rand01( &st ),
					 lo.z + ( hi.z - lo.z ) * b3F_rand01( &st ) };
		bool inside = true;
		for ( int f = 0; f < hfc; ++f )
			if ( b3Dot( hplanes[f].normal, p ) > hplanes[f].offset - 0.01f * diag )
			{
				inside = false;
				break;
			}
		if ( inside )
			seeds[ns++] = p;
	}
	if ( ns < 2 )
	{
		b3F_Chunk chunk = { 0 };
		chunk.hull = b3CloneHull( srcHull );
		b3MassData md = b3ComputeHullMass( chunk.hull, density );
		chunk.centroid = md.center;
		chunk.mass = md.mass;
		chunk.mat = matIdx;
		chunk.piece = -1;
		chunk.restOn = -1;
		outChunkIds[0] = fw->chunks.count;
		b3Array_Push( fw->chunks, chunk );
		return 1;
	}

	b3Vec3* allVerts = (b3Vec3*)b3Alloc( (size_t)ns * B3F_MAX_CELL_VERTS * sizeof( b3Vec3 ) );
	int* vcount = (int*)b3Alloc( (size_t)ns * sizeof( int ) );
	int* cellChunk = (int*)b3Alloc( (size_t)ns * sizeof( int ) );
	int outCount = 0;

	for ( int i = 0; i < ns; ++i )
	{
		b3Plane cp[B3F_MAX_CELL_PLANES];
		int np = 0;
		for ( int f = 0; f < hfc && np < B3F_MAX_CELL_PLANES; ++f )
			cp[np++] = hplanes[f];
		int near[B3F_NEIGHBOR_SEEDS];
		float nearD[B3F_NEIGHBOR_SEEDS];
		int nn = 0;
		for ( int j = 0; j < ns; ++j )
		{
			if ( j == i )
				continue;
			float d = b3LengthSquared( b3Sub( seeds[j], seeds[i] ) );
			if ( nn < B3F_NEIGHBOR_SEEDS )
			{
				near[nn] = j;
				nearD[nn] = d;
				nn++;
			}
			else
			{
				int worst = 0;
				for ( int k = 1; k < nn; ++k )
					if ( nearD[k] > nearD[worst] )
						worst = k;
				if ( d < nearD[worst] )
				{
					near[worst] = j;
					nearD[worst] = d;
				}
			}
		}
		for ( int k = 0; k < nn && np < B3F_MAX_CELL_PLANES; ++k )
			cp[np++] = b3F_bisector( seeds[i], seeds[near[k]] );

		b3Vec3 verts[B3F_MAX_CELL_VERTS];
		int nv = 0;
		for ( int a = 0; a < np && nv < B3F_MAX_CELL_VERTS; ++a )
			for ( int b = a + 1; b < np && nv < B3F_MAX_CELL_VERTS; ++b )
				for ( int c = b + 1; c < np && nv < B3F_MAX_CELL_VERTS; ++c )
				{
					b3Vec3 x;
					if ( !b3F_triSolve( cp[a], cp[b], cp[c], &x ) )
						continue;
					bool ok = true;
					for ( int p = 0; p < np; ++p )
						if ( b3Dot( cp[p].normal, x ) > cp[p].offset + eps )
						{
							ok = false;
							break;
						}
					if ( !ok )
						continue;
					bool dup = false;
					for ( int q = 0; q < nv; ++q )
						if ( b3LengthSquared( b3Sub( verts[q], x ) ) < eps * eps )
						{
							dup = true;
							break;
						}
					if ( !dup )
						verts[nv++] = x;
				}

		if ( nv >= 4 )
		{
			b3F_Chunk chunk = { 0 };
			chunk.hull = b3CreateHull( verts, nv, 64 );
			if ( chunk.hull != NULL )
			{
				b3MassData md = b3ComputeHullMass( chunk.hull, density );
				chunk.centroid = md.center;
				chunk.mass = md.mass;
				chunk.mat = matIdx;
				chunk.piece = -1;
				chunk.restOn = -1;
				cellChunk[i] = fw->chunks.count;
				outChunkIds[outCount++] = cellChunk[i];
				b3Array_Push( fw->chunks, chunk );
				for ( int q = 0; q < nv; ++q )
					allVerts[i * B3F_MAX_CELL_VERTS + q] = verts[q];
				vcount[i] = nv;
				continue;
			}
		}
		cellChunk[i] = -1;
		vcount[i] = 0;
	}

	for ( int i = 0; i < ns; ++i )
	{
		if ( cellChunk[i] < 0 )
			continue;
		for ( int j = i + 1; j < ns; ++j )
		{
			if ( cellChunk[j] < 0 )
				continue;
			b3Plane bij = b3F_bisector( seeds[i], seeds[j] );
			b3Vec3 fv[B3F_MAX_CELL_VERTS];
			int fn = 0;
			for ( int q = 0; q < vcount[i]; ++q )
			{
				b3Vec3 vv = allVerts[i * B3F_MAX_CELL_VERTS + q];
				if ( fabsf( b3Dot( bij.normal, vv ) - bij.offset ) < 4.0f * eps )
					fv[fn++] = vv;
			}
			if ( fn < 3 )
				continue;
			b3Vec3 centroid;
			float area = b3F_polyAreaCentroid( fv, fn, bij.normal, &centroid );
			if ( area <= eps * eps )
				continue;
			b3F_Interface iface = { 0 };
			iface.a = cellChunk[i];
			iface.b = cellChunk[j];
			iface.centroid = centroid;
			iface.normal = bij.normal; // points from a (=i) toward b (=j)
			iface.area = area;
			b3Array_Push( fw->interfaces, iface );
		}
	}

	b3Free( cellChunk, (size_t)ns * sizeof( int ) );
	b3Free( vcount, (size_t)ns * sizeof( int ) );
	b3Free( allVerts, (size_t)ns * B3F_MAX_CELL_VERTS * sizeof( b3Vec3 ) );
	return outCount;
}

static void b3F_formChunkBody( b3FractureWorld* fw, int piece, const int* chunkIds, int count, b3WorldTransform xf,
							   b3Vec3 vel, b3Vec3 omega, bool isStatic, const b3HullData* intactHull )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = isStatic ? b3_staticBody : b3_dynamicBody;
	bd.position = xf.p;
	bd.rotation = xf.q;
	bd.linearVelocity = vel;
	bd.angularVelocity = omega;
	b3BodyId body = b3CreateBody( fw->worldId, &bd );

	if ( intactHull != NULL )
	{
		// one smooth collision shape for the whole body; chunks share it (used only for recolour)
		b3FractureMaterial* mat = fw->materials.data + fw->chunks.data[chunkIds[0]].mat;
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = mat->density;
		sd.baseMaterial.friction = mat->friction;
		sd.baseMaterial.restitution = mat->restitution;
		sd.baseMaterial.customColor = mat->color ? mat->color : 1u;
		sd.enableContactEvents = true;
		b3ShapeId shape = b3CreateHullShape( body, &sd, intactHull );
		for ( int i = 0; i < count; ++i )
		{
			fw->chunks.data[chunkIds[i]].piece = piece;
			fw->chunks.data[chunkIds[i]].shape = shape;
		}
	}
	else
	{
		for ( int i = 0; i < count; ++i )
		{
			b3F_Chunk* ch = fw->chunks.data + chunkIds[i];
			ch->piece = piece;
			b3FractureMaterial* mat = fw->materials.data + ch->mat;
			b3ShapeDef sd = b3DefaultShapeDef();
			sd.density = mat->density;
			sd.baseMaterial.friction = mat->friction;
			sd.baseMaterial.restitution = mat->restitution;
			sd.baseMaterial.customColor = mat->color ? mat->color : 1u;
			sd.enableContactEvents = true;
			ch->shape = b3CreateHullShape( body, &sd, ch->hull );
		}
	}

	b3FracturePiece* P = fw->pieces.data + piece;
	b3Array_Clear( P->voxels );
	b3Array_Append( P->voxels, chunkIds, count );
	P->body = body;
	P->kind = b3F_kindChunk;
	P->isStatic = (uint8_t)isStatic;
	P->debris = 0;
	P->merge = 0;
	P->active = 1;
	P->peakApproach = 0.0f;
	P->overloadStreak = 0;
}

static void b3F_buildChunkAdjacency( b3FractureWorld* fw, int chunkBase, int ifaceBase )
{
	int nNew = fw->chunks.count - chunkBase;
	if ( nNew <= 0 )
		return;
	B3_ASSERT( nNew <= B3F_MAX_SEEDS );

	int deg[B3F_MAX_SEEDS] = { 0 };
	for ( int k = ifaceBase; k < fw->interfaces.count; ++k )
	{
		b3F_Interface* I = fw->interfaces.data + k;
		deg[I->a - chunkBase]++;
		deg[I->b - chunkBase]++;
	}

	int listBase = fw->chunkIfList.count;
	int total = 0;
	for ( int i = 0; i < nNew; ++i )
		total += deg[i];
	b3Array_Resize( fw->chunkIfList, listBase + total );

	int cursor[B3F_MAX_SEEDS];
	int run = listBase;
	for ( int i = 0; i < nNew; ++i )
	{
		b3Array_Push( fw->chunkIfStart, run );
		b3Array_Push( fw->chunkIfCount, deg[i] );
		cursor[i] = run;
		run += deg[i];
	}
	for ( int k = ifaceBase; k < fw->interfaces.count; ++k )
	{
		b3F_Interface* I = fw->interfaces.data + k;
		fw->chunkIfList.data[cursor[I->a - chunkBase]++] = k;
		fw->chunkIfList.data[cursor[I->b - chunkBase]++] = k;
	}
}

static int b3F_addChunkBody( b3FractureWorld* fw, const b3HullData* srcHull, b3WorldTransform xf, int seedCount,
							 b3FractureMaterial material, const b3FractureDef* def )
{
	int matIdx = b3F_internMaterial( fw, material );
	int chunkBase = fw->chunks.count;
	int ifaceBase = fw->interfaces.count;
	int chunkIds[B3F_MAX_SEEDS];
	unsigned rngSeed = (unsigned)( fw->chunks.count * 2654435761u ) ^ 0x1234567u;
	int n = b3F_voronoiSlice( fw, srcHull, seedCount, matIdx, rngSeed, chunkIds );
	if ( n == 0 )
		return -1;
	b3F_buildChunkAdjacency( fw, chunkBase, ifaceBase );

	bool anyAnchor = false;
	for ( int i = 0; i < n; ++i )
	{
		b3F_Chunk* ch = fw->chunks.data + chunkIds[i];
		bool a = def->isStatic;
		if ( def->anchor )
		{
			b3Vec3i cell = { (int)lroundf( ch->centroid.x ), (int)lroundf( ch->centroid.y ),
							 (int)lroundf( ch->centroid.z ) };
			a = def->anchor( cell, def->anchorContext );
		}
		ch->anchor = a ? 1 : 0;
		anyAnchor = anyAnchor || a;
	}
	bool bodyStatic = def->isStatic || anyAnchor;

	int piece = b3F_allocPiece( fw );
	b3F_formChunkBody( fw, piece, chunkIds, n, xf, def->velocity, b3Vec3_zero, bodyStatic, srcHull );
	return piece;
}

static int b3F_addCompoundBody( b3FractureWorld* fw, const b3HullData* const* hulls, int nHulls, b3WorldTransform xf,
								b3FractureMaterial material, const b3FractureDef* def )
{
	if ( nHulls < 1 || nHulls > B3F_MAX_SEEDS )
		return -1;
	int matIdx = b3F_internMaterial( fw, material );
	float density = fw->materials.data[matIdx].density;
	int chunkBase = fw->chunks.count;
	int ifaceBase = fw->interfaces.count;
	int chunkIds[B3F_MAX_SEEDS];
	int n = 0;
	for ( int i = 0; i < nHulls; ++i )
	{
		b3HullData* clone = b3CloneHull( hulls[i] );
		if ( clone == NULL )
			continue;
		b3F_Chunk chunk = { 0 };
		chunk.hull = clone;
		b3MassData md = b3ComputeHullMass( clone, density );
		chunk.centroid = md.center;
		chunk.mass = md.mass;
		chunk.mat = matIdx;
		chunk.piece = -1;
		chunk.restOn = -1;
		chunkIds[n++] = fw->chunks.count;
		b3Array_Push( fw->chunks, chunk );
	}
	if ( n == 0 )
		return -1;
	b3F_buildChunkAdjacency( fw, chunkBase, ifaceBase ); // no interfaces added -> each chunk has degree 0

	bool anyAnchor = false;
	for ( int i = 0; i < n; ++i )
	{
		b3F_Chunk* ch = fw->chunks.data + chunkIds[i];
		bool a = def->isStatic;
		if ( def->anchor )
		{
			b3Vec3i cell = { (int)lroundf( ch->centroid.x ), (int)lroundf( ch->centroid.y ),
							 (int)lroundf( ch->centroid.z ) };
			a = def->anchor( cell, def->anchorContext );
		}
		ch->anchor = a ? 1 : 0;
		anyAnchor = anyAnchor || a;
	}
	bool bodyStatic = def->isStatic || anyAnchor;

	int piece = b3F_allocPiece( fw );
	b3F_formChunkBody( fw, piece, chunkIds, n, xf, def->velocity, b3Vec3_zero, bodyStatic, NULL );
	return piece;
}

static b3Vec3 b3F_chunkWorld( b3FractureWorld* fw, int c )
{
	b3F_Chunk* ch = fw->chunks.data + c;
	b3WorldTransform xf = b3Body_GetTransform( fw->pieces.data[ch->piece].body );
	return b3ToVec3( b3TransformWorldPoint( xf, ch->centroid ) );
}

static int b3F_chunkAt( b3FractureWorld* fw, int p, b3Pos worldPt )
{
	b3FracturePiece* P = fw->pieces.data + p;
	b3WorldTransform xf = b3Body_GetTransform( P->body );
	b3Vec3 local = b3InvTransformWorldPoint( xf, worldPt );
	int best = -1;
	float bestD = 1e30f;
	for ( int i = 0; i < P->voxels.count; ++i )
	{
		int c = P->voxels.data[i];
		float d = b3LengthSquared( b3Sub( fw->chunks.data[c].centroid, local ) );
		if ( d < bestD )
		{
			bestD = d;
			best = c;
		}
	}
	return best;
}

static void b3F_analyzeChunkPiece( b3FractureWorld* fw, int worker, int piece, bool doFracture, b3F_Decision* dec );
static void b3F_severChunkPiece( b3FractureWorld* fw, int piece, const b3F_Decision* dec );
static bool b3F_impactChunkPiece( b3FractureWorld* fw, int piece );

typedef struct b3F_AnalyzeCtx
{
	b3FractureWorld* fw;
	const int* plist;
	b3F_Decision* dec;
	bool doFracture;
} b3F_AnalyzeCtx;

static void b3F_analyzeTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b3F_AnalyzeCtx* ctx = (b3F_AnalyzeCtx*)context;
	b3FractureWorld* fw = ctx->fw;
	for ( int i = startIndex; i < endIndex; ++i )
	{
		int p = ctx->plist[i];
		if ( fw->pieces.data[p].kind == b3F_kindChunk )
			b3F_analyzeChunkPiece( fw, workerIndex, p, ctx->doFracture, ctx->dec + i );
		else
			b3F_analyzePiece( fw, workerIndex, p, ctx->doFracture, ctx->dec + i );
	}
}

void b3FractureWorld_Step( b3World* world, float dt )
{
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	if ( fw == NULL )
		return;
	fw->frame++;
	fw->gravity = world->gravity;
	fw->profSeverMs = 0.0f;
	uint64_t profT0 = b3GetTicks();

	int stride = b3F_stride( fw );

	float invDt = dt > 0.0f ? 1.0f / dt : 0.0f;
	b3F_gatherContactForces( world, fw, invDt );

	world->profile.fractureGather = b3GetMilliseconds( profT0 ); // stress zeroing + contact gather
	uint64_t profT1 = b3GetTicks();

	bool doFracture = fw->tuning.fractureEnabled && fw->frame > fw->tuning.warmupFrames;
	int nP = fw->pieces.count; // fixed: fresh children (appended) wait for next step
	int debrisLive = 0;		   // loose single-chunk / single-voxel pieces (for tuning.maxDebris)

	if ( fw->tuning.parallelAnalysis && world->workerCount > 1 )
	{
		int* plist = (int*)b3F_scratch( fw, B3F_SC_PLIST, (size_t)( nP > 0 ? nP : 1 ) * sizeof( int ) );
		int nList = 0;
		for ( int p = 0; p < nP; ++p )
		{
			b3FracturePiece* P = fw->pieces.data + p;
			if ( !P->active )
				continue;
			if ( P->voxels.count < 2 && !P->isStatic )
				debrisLive++;
			if ( P->kind == b3F_kindChunk && P->voxels.count < 2 )
				continue; // inert: no interfaces left, nothing in the pipeline can affect it
			if ( P->isStatic == 0 && b3Body_IsAwake( P->body ) == false )
				continue;
			bool analyse = stride == 1 || ( ( fw->frame + p ) % stride ) == 0;
			bool impacted = doFracture && ( P->kind == b3F_kindChunk ? b3F_impactChunkPiece( fw, p )
																	 : b3F_impactFracture( fw, p ) );
			if ( !impacted && analyse )
				plist[nList++] = p;
		}

		b3F_Decision* dec =
			(b3F_Decision*)b3F_scratch( fw, B3F_SC_PDEC, (size_t)( nList > 0 ? nList : 1 ) * sizeof( b3F_Decision ) );
		b3F_AnalyzeCtx ctx = { fw, plist, dec, doFracture };
		b3ParallelFor( world, b3F_analyzeTask, nList, 4, &ctx, "FractureAnalyze" );

		for ( int i = 0; i < nList; ++i )
		{
			if ( !dec[i].fracture )
				continue;
			if ( fw->pieces.data[plist[i]].kind == b3F_kindChunk )
				b3F_severChunkPiece( fw, plist[i], &dec[i] );
			else
				b3F_severVoxelPiece( fw, plist[i], &dec[i] );
		}
	}
	else
	{
		for ( int p = 0; p < nP; ++p )
		{
			b3FracturePiece* P = fw->pieces.data + p;
			if ( !P->active )
				continue;
			if ( P->voxels.count < 2 && !P->isStatic )
				debrisLive++;
			if ( P->kind == b3F_kindChunk && P->voxels.count < 2 )
				continue; // inert: no interfaces left, nothing in the pipeline can affect it
			if ( P->isStatic == 0 && b3Body_IsAwake( P->body ) == false )
				continue;
			bool analyse = stride == 1 || ( ( fw->frame + p ) % stride ) == 0;
			b3F_Decision dec;
			if ( P->kind == b3F_kindChunk )
			{
				if ( !( doFracture && b3F_impactChunkPiece( fw, p ) ) && analyse )
				{
					b3F_analyzeChunkPiece( fw, 0, p, doFracture, &dec );
					if ( dec.fracture )
						b3F_severChunkPiece( fw, p, &dec );
				}
				continue;
			}
			if ( !( doFracture && b3F_impactFracture( fw, p ) ) && analyse )
			{
				b3F_analyzePiece( fw, 0, p, doFracture, &dec );
				if ( dec.fracture )
					b3F_severVoxelPiece( fw, p, &dec );
			}
		}
	}

	float updateMs = b3GetMilliseconds( profT1 );
	world->profile.fractureSever = fw->profSeverMs;
	world->profile.fractureAnalyze = b3F_max( updateMs - fw->profSeverMs, 0.0f );
	uint64_t profT2 = b3GetTicks();

	if ( fw->tuning.maxDebris > 0 )
	{
		int burst = 64;
		while ( debrisLive > fw->tuning.maxDebris && fw->debrisHead < fw->debrisQueue.count && burst > 0 )
		{
			int dp = fw->debrisQueue.data[fw->debrisHead++];
			b3FracturePiece* D = fw->pieces.data + dp;
			if ( !D->active || D->isStatic || D->voxels.count >= 2 )
				continue; // stale queue entry
			if ( D->kind == b3F_kindVoxel )
				b3F_destroyVoxels( fw, D->voxels.data, D->voxels.count );
			else
				for ( int i = 0; i < D->voxels.count; ++i )
					fw->chunks.data[D->voxels.data[i]].piece = -1;
			b3F_destroyPieceBody( fw, dp );
			debrisLive--;
			burst--;
		}
	}
	if ( fw->debrisHead > 0 && fw->debrisHead >= fw->debrisQueue.count )
	{
		b3Array_Clear( fw->debrisQueue );
		fw->debrisHead = 0;
	}

	world->profile.fractureDebris = b3GetMilliseconds( profT2 );
	world->profile.fracture = b3GetMilliseconds( profT0 );
}

void b3FractureWorld_Destroy( b3World* world )
{
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	if ( fw == NULL )
		return;
	for ( int p = 0; p < fw->pieces.count; ++p )
		b3Array_Destroy( fw->pieces.data[p].voxels );
	for ( int c = 0; c < fw->chunks.count; ++c )
		if ( fw->chunks.data[c].hull != NULL )
			b3DestroyHull( fw->chunks.data[c].hull );
	b3Array_Destroy( fw->pieces );
	b3Array_Destroy( fw->voxels );
	b3Array_Destroy( fw->materials );
	b3Array_Destroy( fw->chunks );
	b3Array_Destroy( fw->interfaces );
	b3Array_Destroy( fw->chunkIfStart );
	b3Array_Destroy( fw->chunkIfCount );
	b3Array_Destroy( fw->chunkIfList );
	b3Array_Destroy( fw->debrisQueue );
	b3F_mapFree( &fw->cellToVox );
	for ( int w = 0; w < B3_MAX_WORKERS; ++w )
		for ( int i = 0; i < B3F_SC_COUNT; ++i )
			if ( fw->scratch[w][i].ptr )
				b3Free( fw->scratch[w][i].ptr, fw->scratch[w][i].cap );
	b3Free( fw, sizeof( b3FractureWorld ) );
	world->fractureWorld = NULL;
}

static int b3F_pieceInterfaces( b3FractureWorld* fw, int piece, int* out, int max )
{
	int n = 0;
	b3FracturePiece* P = fw->pieces.data + piece;
	for ( int i = 0; i < P->voxels.count; ++i )
	{
		int c = P->voxels.data[i]; // chunk id
		int start = fw->chunkIfStart.data[c];
		int cnt = fw->chunkIfCount.data[c];
		for ( int t = 0; t < cnt; ++t )
		{
			int k = fw->chunkIfList.data[start + t];
			b3F_Interface* I = fw->interfaces.data + k;
			if ( I->broken )
				continue;
			// emit each intra-piece interface exactly once, when we reach its 'a' endpoint
			if ( I->a != c )
				continue;
			if ( fw->chunks.data[I->b].piece != piece )
				continue;
			if ( n < max )
				out[n++] = k;
		}
	}
	for ( int a = 1; a < n; ++a ) // insertion sort into ascending interface-index order
	{
		int key = out[a], j = a - 1;
		while ( j >= 0 && out[j] > key )
		{
			out[j + 1] = out[j];
			j--;
		}
		out[j + 1] = key;
	}
	return n;
}

static int b3F_chunkComponents( b3FractureWorld* fw, const int* ids, int count, const int* pifs, int nif, int* label )
{
	b3F_Map g2l;
	memset( &g2l, 0, sizeof( g2l ) );
	for ( int i = 0; i < count; ++i )
		b3F_mapPut( &g2l, (uint64_t)( ids[i] + 1 ), i ); // +1 so key is never 0
	int* ea = (int*)b3Alloc( (size_t)nif * sizeof( int ) );
	int* eb = (int*)b3Alloc( (size_t)nif * sizeof( int ) );
	for ( int k = 0; k < nif; ++k )
	{
		b3F_Interface* I = fw->interfaces.data + pifs[k];
		ea[k] = b3F_mapGet( &g2l, (uint64_t)( I->a + 1 ) );
		eb[k] = b3F_mapGet( &g2l, (uint64_t)( I->b + 1 ) );
	}
	b3F_mapFree( &g2l );

	for ( int i = 0; i < count; ++i )
		label[i] = -1;
	int* stack = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	int comp = 0;
	for ( int s = 0; s < count; ++s )
	{
		if ( label[s] >= 0 )
			continue;
		int sp = 0;
		stack[sp++] = s;
		label[s] = comp;
		while ( sp > 0 )
		{
			int u = stack[--sp];
			for ( int k = 0; k < nif; ++k )
			{
				int w = -1;
				if ( ea[k] == u )
					w = eb[k];
				else if ( eb[k] == u )
					w = ea[k];
				if ( w >= 0 && label[w] < 0 )
				{
					label[w] = comp;
					stack[sp++] = w;
				}
			}
		}
		comp++;
	}
	b3Free( stack, (size_t)count * sizeof( int ) );
	b3Free( eb, (size_t)nif * sizeof( int ) );
	b3Free( ea, (size_t)nif * sizeof( int ) );
	return comp;
}

static void b3F_splitChunkPiece( b3FractureWorld* fw, int piece )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	int count = P->voxels.count;
	if ( count == 0 )
	{
		b3F_destroyPieceBody( fw, piece );
		return;
	}
	int* ids = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	memcpy( ids, P->voxels.data, (size_t)count * sizeof( int ) );

	int maxIf = fw->interfaces.count;
	int* pifs = (int*)b3Alloc( (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
	int nif = b3F_pieceInterfaces( fw, piece, pifs, maxIf );

	int* label = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	int nComp = b3F_chunkComponents( fw, ids, count, pifs, nif, label );
	if ( nComp <= 1 )
	{
		b3Free( label, (size_t)count * sizeof( int ) );
		b3Free( pifs, (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
		b3Free( ids, (size_t)count * sizeof( int ) );
		return;
	}

	b3WorldTransform xf = b3Body_GetTransform( P->body );
	b3Vec3 pomega = b3Body_GetAngularVelocity( P->body );
	b3Vec3 pvel = b3Body_GetWorldPointVelocity( P->body, xf.p );
	float gravityScale = b3Body_GetGravityScale( P->body );
	float linearDamping = b3Body_GetLinearDamping( P->body );
	float angularDamping = b3Body_GetAngularDamping( P->body );

	b3F_destroyPieceBody( fw, piece );

	int* compIds = (int*)b3Alloc( (size_t)count * sizeof( int ) );
	for ( int comp = 0; comp < nComp; ++comp )
	{
		int m = 0;
		bool childStatic = false;
		for ( int i = 0; i < count; ++i )
			if ( label[i] == comp )
			{
				compIds[m++] = ids[i];
				if ( fw->chunks.data[ids[i]].anchor )
					childStatic = true;
			}
		if ( m == 0 )
			continue;
		int target = ( comp == 0 ) ? piece : b3F_allocPiece( fw );
		// a fractured fragment is an arbitrary (possibly non-convex) subset of chunks: it must
		// collide as its individual chunk hulls, so no intact hull here
		b3F_formChunkBody( fw, target, compIds, m, xf, childStatic ? b3Vec3_zero : pvel,
						   childStatic ? b3Vec3_zero : pomega, childStatic, NULL );
		b3BodyId childBody = fw->pieces.data[target].body;
		b3Body_SetGravityScale( childBody, gravityScale );
		b3Body_SetLinearDamping( childBody, linearDamping );
		b3Body_SetAngularDamping( childBody, angularDamping );
		if ( !childStatic && m == 1 )
			b3Array_Push( fw->debrisQueue, target ); // candidate for the tuning.maxDebris cap
	}

	b3Free( compIds, (size_t)count * sizeof( int ) );
	b3Free( label, (size_t)count * sizeof( int ) );
	b3Free( pifs, (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
	b3Free( ids, (size_t)count * sizeof( int ) );
}

static bool b3F_impactChunkPiece( b3FractureWorld* fw, int piece )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	if ( P->peakApproach <= fw->tuning.impactSpeed )
		return false;
	int count = P->voxels.count;
	bool any = false;
	for ( int i = 0; i < count; ++i )
	{
		b3F_Chunk* ch = fw->chunks.data + P->voxels.data[i];
		float density = fw->materials.data[ch->mat].density;
		float vol = ch->mass / ( density > B3F_EPS ? density : 1.0f );
		float area = powf( vol > B3F_EPS ? vol : B3F_EPS, 2.0f / 3.0f );
		float strength = fw->materials.data[ch->mat].strength * fw->tuning.strengthScale * fw->tuning.impactBearing;
		if ( strength <= B3F_EPS )
			continue;
		float bearing = b3Length( ch->forceRaw ) / ( area > B3F_EPS ? area : B3F_EPS );
		if ( bearing > strength )
		{
			ch->stress = 1.0f; // flag hit chunks (visual)
			any = true;
		}
		else
			ch->stress = 0.0f;
	}
	if ( !any )
		return false;
	uint64_t profTicks = b3GetTicks(); // the impact response (sever + split) is sever time

	int maxIf = fw->interfaces.count;
	int* pifs = (int*)b3F_scratch( fw, B3F_SC_C_PIFS, (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
	int nif = b3F_pieceInterfaces( fw, piece, pifs, maxIf ); // this piece's unbroken interfaces, ascending
	for ( int t = 0; t < nif; ++t )
	{
		b3F_Interface* I = fw->interfaces.data + pifs[t];
		if ( fw->chunks.data[I->a].stress >= 1.0f || fw->chunks.data[I->b].stress >= 1.0f )
			I->broken = 1;
	}
	b3F_splitChunkPiece( fw, piece );
	fw->profSeverMs += b3GetMilliseconds( profTicks );
	return true;
}

static void b3F_analyzeChunkPiece( b3FractureWorld* fw, int worker, int piece, bool doFracture, b3F_Decision* dec )
{
	b3FracturePiece* P = fw->pieces.data + piece;
	dec->fracture = false;
	int nc = P->voxels.count;
	for ( int i = 0; i < nc; ++i )
	{
		b3F_Chunk* ch = fw->chunks.data + P->voxels.data[i];
		ch->stressShown += 0.15f * ( ch->stress - ch->stressShown );
		ch->stress = 0.0f;
	}
	if ( nc < 2 )
		return;
	if ( !b3F_analysisEligible( fw, P ) )
		return; // overloadStreak freezes until the body can fracture again

	float gmag = b3Length( fw->gravity );
	b3Vec3 up = ( gmag > B3F_EPS ) ? b3MulSV( -1.0f / gmag, fw->gravity ) : (b3Vec3){ 0, 1, 0 };
	b3WorldTransform xf = b3Body_GetTransform( P->body );

	b3Vec3* wc = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_WC, (size_t)nc * sizeof( b3Vec3 ) );
	float* mass = (float*)b3F_scratchW( fw, worker, B3F_SC_C_MASS, (size_t)nc * sizeof( float ) );
	b3Vec3* fext = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_FEXT, (size_t)nc * sizeof( b3Vec3 ) );
	char* support = (char*)b3F_scratchW( fw, worker, B3F_SC_C_SUPPORT, (size_t)nc );
	int nsup = 0;
	for ( int i = 0; i < nc; ++i )
	{
		b3F_Chunk* ch = fw->chunks.data + P->voxels.data[i];
		ch->scratchLocal = i; // O(1) global chunk id -> local index for the interface pass below
		wc[i] = b3ToVec3( b3TransformWorldPoint( xf, ch->centroid ) );
		mass[i] = ch->mass;
		b3Vec3 load = ch->force;
		float upc = b3Dot( load, up );
		if ( upc > 0.0f )
			load = b3MulSub( load, upc, up );
		fext[i] = b3Add( b3MulSV( mass[i], fw->gravity ), load );
		support[i] = ( ch->anchor || ch->restOn != -1 ) ? 1 : 0;
		nsup += support[i];
	}

	if ( nsup == 0 || nsup == nc )
		return;
	{
		int maxIf = fw->interfaces.count;
		int* pifs = (int*)b3F_scratchW( fw, worker, B3F_SC_C_PIFS, (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
		int nif = b3F_pieceInterfaces( fw, piece, pifs, maxIf );
		int* ia = (int*)b3F_scratchW( fw, worker, B3F_SC_C_IA, (size_t)( nif > 0 ? nif : 1 ) * sizeof( int ) );
		int* ib = (int*)b3F_scratchW( fw, worker, B3F_SC_C_IB, (size_t)( nif > 0 ? nif : 1 ) * sizeof( int ) );
		b3Vec3* icen = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_ICEN, (size_t)( nif > 0 ? nif : 1 ) * sizeof( b3Vec3 ) );
		float* iarea = (float*)b3F_scratchW( fw, worker, B3F_SC_C_IAREA, (size_t)( nif > 0 ? nif : 1 ) * sizeof( float ) );
		float* istr = (float*)b3F_scratchW( fw, worker, B3F_SC_C_ISTR, (size_t)( nif > 0 ? nif : 1 ) * sizeof( float ) );
		for ( int k = 0; k < nif; ++k )
		{
			b3F_Interface* I = fw->interfaces.data + pifs[k];
			ia[k] = fw->chunks.data[I->a].scratchLocal;
			ib[k] = fw->chunks.data[I->b].scratchLocal;
			icen[k] = b3ToVec3( b3TransformWorldPoint( xf, I->centroid ) );
			iarea[k] = I->area;
			float sa = fw->materials.data[fw->chunks.data[I->a].mat].strength;
			float sb = fw->materials.data[fw->chunks.data[I->b].mat].strength;
			istr[k] = ( sa < sb ? sa : sb ) * fw->tuning.strengthScale;
		}

		float bestFail = 1.0f;
		int bestAxis = -1;
		float bestCut = 0.0f;
		float* coord = (float*)b3F_scratchW( fw, worker, B3F_SC_C_COORD, (size_t)nc * sizeof( float ) );
		float* sorted = (float*)b3F_scratchW( fw, worker, B3F_SC_C_SORTED, (size_t)nc * sizeof( float ) );
		int* sidx = (int*)b3F_scratchW( fw, worker, B3F_SC_C_SIDX, (size_t)nc * sizeof( int ) );
		int* supPre = (int*)b3F_scratchW( fw, worker, B3F_SC_C_SUPPRE, (size_t)( nc + 1 ) * sizeof( int ) );
		int* straddle = (int*)b3F_scratchW( fw, worker, B3F_SC_C_STRADDLE, (size_t)( nif > 0 ? nif : 1 ) * sizeof( int ) );
		b3Vec3* preF = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_PREF, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		b3Vec3* preCross = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_PRECROSS, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		b3Vec3* preMP = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_PREMP, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		float* preM = (float*)b3F_scratchW( fw, worker, B3F_SC_C_PREM, (size_t)( nc + 1 ) * sizeof( float ) );
		b3Vec3* sufF = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_SUFF, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		b3Vec3* sufCross = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_SUFCROSS, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		b3Vec3* sufMP = (b3Vec3*)b3F_scratchW( fw, worker, B3F_SC_C_SUFMP, (size_t)( nc + 1 ) * sizeof( b3Vec3 ) );
		float* sufM = (float*)b3F_scratchW( fw, worker, B3F_SC_C_SUFM, (size_t)( nc + 1 ) * sizeof( float ) );
		b3Vec3 ref = wc[0]; // a member position: keeps every moment lever arm piece-sized

		for ( int axis = 0; axis < 3; ++axis )
		{
			b3Vec3 e = b3Vec3_zero;
			( &e.x )[axis] = 1.0f;
			b3Vec3 normal = b3RotateVector( xf.q, e );
			for ( int i = 0; i < nc; ++i )
				coord[i] = b3Dot( wc[i], normal );
			for ( int i = 0; i < nc; ++i )
				sidx[i] = i;
			for ( int a = 1; a < nc; ++a ) // insertion sort of indices by coord
			{
				int key = sidx[a];
				float kc = coord[key];
				int j = a - 1;
				while ( j >= 0 && coord[sidx[j]] > kc )
				{
					sidx[j + 1] = sidx[j];
					j--;
				}
				sidx[j + 1] = key;
			}
			for ( int i = 0; i < nc; ++i )
				sorted[i] = coord[sidx[i]];
			supPre[0] = 0; // supPre[g] = supported chunks among the g smallest coords
			for ( int i = 0; i < nc; ++i )
				supPre[i + 1] = supPre[i] + support[sidx[i]];
			preF[0] = b3Vec3_zero;
			preCross[0] = b3Vec3_zero;
			preMP[0] = b3Vec3_zero;
			preM[0] = 0.0f;
			for ( int g = 0; g < nc; ++g )
			{
				int i = sidx[g];
				preF[g + 1] = b3Add( preF[g], fext[i] );
				preCross[g + 1] = b3Add( preCross[g], b3Cross( b3Sub( wc[i], ref ), fext[i] ) );
				preMP[g + 1] = b3MulAdd( preMP[g], mass[i], b3Sub( wc[i], ref ) );
				preM[g + 1] = preM[g] + mass[i];
			}
			sufF[nc] = b3Vec3_zero;
			sufCross[nc] = b3Vec3_zero;
			sufMP[nc] = b3Vec3_zero;
			sufM[nc] = 0.0f;
			for ( int g = nc - 1; g >= 0; --g )
			{
				int i = sidx[g];
				sufF[g] = b3Add( sufF[g + 1], fext[i] );
				sufCross[g] = b3Add( sufCross[g + 1], b3Cross( b3Sub( wc[i], ref ), fext[i] ) );
				sufMP[g] = b3MulAdd( sufMP[g + 1], mass[i], b3Sub( wc[i], ref ) );
				sufM[g] = sufM[g + 1] + mass[i];
			}

			for ( int g = 0; g + 1 < nc; ++g )
			{
				if ( sorted[g + 1] - sorted[g] < 1e-5f )
					continue;
				float cut = 0.5f * ( sorted[g] + sorted[g + 1] );

				int nlow = g + 1;
				while ( nlow < nc && sorted[nlow] <= cut )
					nlow++;
				int nhigh = nc - nlow;
				if ( nhigh == 0 )
					continue;
				int slow = supPre[nlow], shigh = nsup - slow;
				bool sLow = slow > 0, sHigh = shigh > 0;
				if ( sLow == sHigh )
					continue;
				bool freeIsHigh = !sHigh;

				float A = 0.0f;
				b3Vec3 c0 = b3Vec3_zero;
				float strength = 1e30f, cfac = 1.0f;
				int nStraddle = 0;
				for ( int k = 0; k < nif; ++k )
				{
					float da = coord[ia[k]] - cut, db = coord[ib[k]] - cut;
					if ( da * db >= 0.0f )
						continue;
					A += iarea[k];
					c0 = b3MulAdd( c0, iarea[k], icen[k] );
					if ( istr[k] < strength )
					{
						strength = istr[k];
						cfac = fw->materials.data[fw->chunks.data[fw->interfaces.data[pifs[k]].a].mat].compressiveFactor;
					}
					straddle[nStraddle++] = k;
				}
				if ( nStraddle == 0 || A <= B3F_EPS || strength <= B3F_EPS )
					continue;
				c0 = b3MulSV( 1.0f / A, c0 );

				b3Vec3 F, crossSum, MP;
				float mFree;
				if ( freeIsHigh )
				{
					F = sufF[nlow];
					crossSum = sufCross[nlow];
					MP = sufMP[nlow];
					mFree = sufM[nlow];
				}
				else
				{
					F = preF[nlow];
					crossSum = preCross[nlow];
					MP = preMP[nlow];
					mFree = preM[nlow];
				}
				if ( mFree <= B3F_EPS )
					continue;
				b3Vec3 Mv = b3Sub( crossSum, b3Cross( b3Sub( c0, ref ), F ) );
				b3Vec3 comFree = b3Add( ref, b3MulSV( 1.0f / mFree, MP ) );

				float dn = b3Dot( normal, b3Sub( comFree, c0 ) );
				b3Vec3 nOut = b3MulSV( dn < 0.0f ? -1.0f : 1.0f, normal );
				float nAx = b3Dot( F, nOut ) / A;

				b3Vec3 Mplane = b3Sub( Mv, b3MulSV( b3Dot( Mv, normal ), normal ) );
				float Mmag = b3Length( Mplane );
				float bend = 0.0f;
				if ( Mmag > B3F_EPS )
				{
					b3Vec3 d = b3Cross( normal, b3MulSV( 1.0f / Mmag, Mplane ) );
					float sum = 0.0f, cext = 0.0f;
					for ( int si = 0; si < nStraddle; ++si )
					{
						int k = straddle[si];
						float s = b3Dot( b3Sub( icen[k], c0 ), d );
						sum += iarea[k] * s * s + iarea[k] * iarea[k] / 12.0f;
						float ce = fabsf( s ) + 0.5f * sqrtf( iarea[k] );
						if ( ce > cext )
							cext = ce;
					}
					if ( sum > B3F_EPS )
						bend = Mmag * cext / sum;
				}

				float tFib = b3F_max( 0.0f, nAx + bend );
				float cFib = b3F_max( 0.0f, bend - nAx );
				float view = ( fabsf( nAx ) + bend ) / strength;
				float fail = b3F_max( tFib, cFib / b3F_max( 1.0f, cfac ) ) / strength;

				for ( int si = 0; si < nStraddle; ++si )
				{
					int k = straddle[si];
					b3F_Chunk* ca = fw->chunks.data + P->voxels.data[ia[k]];
					b3F_Chunk* cb = fw->chunks.data + P->voxels.data[ib[k]];
					if ( view > ca->stress )
						ca->stress = view;
					if ( view > cb->stress )
						cb->stress = view;
				}
				if ( fail > bestFail )
				{
					bestFail = fail;
					bestAxis = axis;
					bestCut = cut;
				}
			}
		}

		if ( bestAxis >= 0 )
			P->overloadStreak += b3F_stride( fw ); // one analysis stands in for K frames of overload
		else
			P->overloadStreak = 0;
		bool sustained = P->overloadStreak >= fw->tuning.fractureHoldFrames;
		bool impactNow = P->peakApproach > fw->tuning.impactSpeed;

		if ( doFracture && bestAxis >= 0 && ( impactNow || sustained ) )
		{
			dec->axis = bestAxis;
			dec->cut = bestCut;
			dec->fracture = true;
		}
	}
}

static void b3F_severChunkPiece( b3FractureWorld* fw, int piece, const b3F_Decision* dec )
{
	uint64_t profTicks = b3GetTicks();
	b3FracturePiece* P = fw->pieces.data + piece;
	b3WorldTransform xf = b3Body_GetTransform( P->body );
	b3Vec3 e = b3Vec3_zero;
	( &e.x )[dec->axis] = 1.0f;
	b3Vec3 normal = b3RotateVector( xf.q, e );

	int maxIf = fw->interfaces.count;
	int* pifs = (int*)b3F_scratch( fw, B3F_SC_C_PIFS, (size_t)( maxIf > 0 ? maxIf : 1 ) * sizeof( int ) );
	int nif = b3F_pieceInterfaces( fw, piece, pifs, maxIf );
	for ( int k = 0; k < nif; ++k )
	{
		b3F_Interface* I = fw->interfaces.data + pifs[k];
		b3Vec3 wa = b3ToVec3( b3TransformWorldPoint( xf, fw->chunks.data[I->a].centroid ) );
		b3Vec3 wb = b3ToVec3( b3TransformWorldPoint( xf, fw->chunks.data[I->b].centroid ) );
		float da = b3Dot( wa, normal ) - dec->cut, db = b3Dot( wb, normal ) - dec->cut;
		if ( da * db < 0.0f )
			I->broken = 1;
	}
	b3F_splitChunkPiece( fw, piece );
	fw->profSeverMs += b3GetMilliseconds( profTicks );
}

void b3World_EnableFracture( b3WorldId worldId, float voxel, float groundY )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world != NULL );
	b3FractureWorld* fw = b3F_getOrCreate( world, worldId, voxel > 0.0f ? voxel : 1.0f, groundY );
	fw->voxel = voxel > 0.0f ? voxel : 1.0f;
	fw->radius = 0.5f * fw->voxel;
	fw->groundY = groundY;
}

bool b3World_IsFractureEnabled( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world != NULL && world->fractureWorld != NULL;
}

void b3World_SetFractureTuning( b3WorldId worldId, b3FractureTuning tuning )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->fractureWorld == NULL )
		return;
	( (b3FractureWorld*)world->fractureWorld )->tuning = tuning;
}

b3FractureTuning b3World_GetFractureTuning( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world != NULL && world->fractureWorld != NULL )
		return ( (b3FractureWorld*)world->fractureWorld )->tuning;
	return b3DefaultFractureTuning();
}

int b3World_CreateFractureVoxels( b3WorldId worldId, const b3Vec3i* cells, int count, b3FractureMaterial material,
								  const b3FractureDef* def )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world != NULL );
	if ( count <= 0 )
		return -1;
	b3FractureDef local = def ? *def : b3DefaultFractureDef();
	b3FractureWorld* fw = b3F_getOrCreate( world, worldId, 1.0f, 0.0f );
	return b3F_addBody( fw, cells, count, material, &local );
}

int b3World_CreateFractureBox( b3WorldId worldId, b3Vec3 center, b3Vec3 halfExtents, b3FractureMaterial material,
							   const b3FractureDef* def )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world != NULL );
	b3FractureWorld* fw = b3F_getOrCreate( world, worldId, 1.0f, 0.0f );
	float voxel = fw->voxel;

	int nx = (int)lroundf( 2.0f * halfExtents.x / voxel );
	int ny = (int)lroundf( 2.0f * halfExtents.y / voxel );
	int nz = (int)lroundf( 2.0f * halfExtents.z / voxel );
	if ( nx < 1 )
		nx = 1;
	if ( ny < 1 )
		ny = 1;
	if ( nz < 1 )
		nz = 1;
	int ox = (int)lroundf( ( center.x - halfExtents.x ) / voxel );
	int oy = (int)lroundf( ( center.y - halfExtents.y ) / voxel );
	int oz = (int)lroundf( ( center.z - halfExtents.z ) / voxel );

	int count = nx * ny * nz;
	b3Vec3i* cells = (b3Vec3i*)b3Alloc( (size_t)count * sizeof( b3Vec3i ) );
	int i = 0;
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
				cells[i++] = (b3Vec3i){ ox + x, oy + y, oz + z };

	b3FractureDef local = def ? *def : b3DefaultFractureDef();
	local.merge = true; // a "normal" solid body
	int result = b3F_addBody( fw, cells, count, material, &local );
	b3Free( cells, (size_t)count * sizeof( b3Vec3i ) );
	return result;
}

int b3World_CreateFractureConvex( b3WorldId worldId, const b3HullData* hull, b3WorldTransform transform, int chunkCount,
								  b3FractureMaterial material, const b3FractureDef* def )
{
	b3World* world = b3GetWorldFromId( worldId );
	B3_ASSERT( world != NULL );
	if ( hull == NULL || chunkCount < 1 )
		return -1;
	b3FractureDef local = def ? *def : b3DefaultFractureDef();
	b3FractureWorld* fw = b3F_getOrCreate( world, worldId, 1.0f, 0.0f );
	return b3F_addChunkBody( fw, hull, transform, chunkCount, material, &local );
}

int b3World_MakeBodyFracture( b3WorldId worldId, b3BodyId bodyId, b3FractureMaterial material, const b3FractureDef* def )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || !b3Body_IsValid( bodyId ) )
		return -1;
	if ( b3Body_GetType( bodyId ) != b3_dynamicBody )
		return -1;
	if ( b3Body_GetJointCount( bodyId ) > 0 )
		return -1; // converting a jointed body would orphan the joint

	b3FractureWorld* existing = (b3FractureWorld*)world->fractureWorld;
	if ( existing != NULL )
		for ( int p = 0; p < existing->pieces.count; ++p )
			if ( existing->pieces.data[p].active && B3_ID_EQUALS( existing->pieces.data[p].body, bodyId ) )
				return -1;

	int nsh = b3Body_GetShapeCount( bodyId );
	if ( nsh < 1 || nsh > 16 )
		return -1;
	b3ShapeId shapes[16];
	int got = b3Body_GetShapes( bodyId, shapes, nsh );
	const b3HullData* hulls[16];
	for ( int i = 0; i < got; ++i )
	{
		if ( b3Shape_GetType( shapes[i] ) != b3_hullShape )
			return -1;
		hulls[i] = b3Shape_GetHull( shapes[i] );
	}
	if ( got < 1 )
		return -1;
	const b3HullData* hull = hulls[0]; // the sliceable hull for the single-shape path

	b3WorldTransform xf = b3Body_GetTransform( bodyId );
	b3Vec3 lv = b3Body_GetWorldPointVelocity( bodyId, xf.p );
	b3Vec3 av = b3Body_GetAngularVelocity( bodyId );

	float gravityScale = b3Body_GetGravityScale( bodyId );
	float linearDamping = b3Body_GetLinearDamping( bodyId );
	float angularDamping = b3Body_GetAngularDamping( bodyId );
	b3MotionLocks motionLocks = b3Body_GetMotionLocks( bodyId );
	float sleepThreshold = b3Body_GetSleepThreshold( bodyId );
	bool sleepEnabled = b3Body_IsSleepEnabled( bodyId );
	bool bullet = b3Body_IsBullet( bodyId );

	float totalVolume = 0.0f; // sum of all hull volumes (mass at unit density)
	for ( int i = 0; i < got; ++i )
		totalVolume += b3ComputeHullMass( hulls[i], 1.0f ).mass;
	int chunks = (int)( totalVolume * 2.0f ) + 6; // seed count for the single-shape slice
	if ( chunks < 8 )
		chunks = 8;
	if ( chunks > 28 )
		chunks = 28;

	float originalMass = b3Body_GetMass( bodyId );
	if ( totalVolume > 1e-9f && originalMass > 0.0f && material.density > 0.0f )
	{
		float newDensity = originalMass / totalVolume;
		material.strength *= newDensity / material.density;
		material.density = newDensity;
	}

	b3FractureWorld* fw = b3F_getOrCreate( world, worldId, 1.0f, 0.0f );
	b3FractureDef local = def ? *def : b3DefaultFractureDef();
	local.velocity = lv;
	int piece = got == 1 ? b3F_addChunkBody( fw, hull, xf, chunks, material, &local )		 // slice into chunks
						 : b3F_addCompoundBody( fw, hulls, got, xf, material, &local ); // each shape -> one chunk
	if ( piece < 0 )
		return -1;
	b3BodyId nb = fw->pieces.data[piece].body;
	b3Body_SetAngularVelocity( nb, av );
	b3Body_SetGravityScale( nb, gravityScale );
	b3Body_SetLinearDamping( nb, linearDamping );
	b3Body_SetAngularDamping( nb, angularDamping );
	b3Body_SetMotionLocks( nb, motionLocks );
	b3Body_SetSleepThreshold( nb, sleepThreshold );
	b3Body_EnableSleep( nb, sleepEnabled );
	b3Body_SetBullet( nb, bullet );
	b3DestroyBody( bodyId );
	return piece;
}

void b3World_ApplyFractureColors( b3WorldId worldId, b3FractureColorMode mode )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->fractureWorld == NULL )
		return;
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	for ( int v = 0; v < fw->voxels.count; ++v )
	{
		b3FractureVoxel* vx = fw->voxels.data + v;
		if ( vx->piece < 0 || !b3Shape_IsValid( vx->shape ) )
			continue;
		uint32_t col;
		if ( mode == b3_fractureColorStress )
			col = b3F_heatColor( vx->stressShown );
		else if ( mode == b3_fractureColorFragment )
			col = b3F_fragmentColor( vx->piece );
		else
			col = fw->materials.data[vx->mat].color;
		b3SurfaceMaterial sm = b3Shape_GetSurfaceMaterial( vx->shape );
		sm.customColor = col ? col : 1u;
		b3Shape_SetSurfaceMaterial( vx->shape, sm );
	}
	for ( int c = 0; c < fw->chunks.count; ++c )
	{
		b3F_Chunk* ch = fw->chunks.data + c;
		if ( ch->piece < 0 || !b3Shape_IsValid( ch->shape ) )
			continue;
		uint32_t col;
		if ( mode == b3_fractureColorStress )
			col = b3F_heatColor( ch->stressShown );
		else if ( mode == b3_fractureColorFragment )
			col = b3F_fragmentColor( ch->piece );
		else
			col = fw->materials.data[ch->mat].color;
		b3SurfaceMaterial sm = b3Shape_GetSurfaceMaterial( ch->shape );
		sm.customColor = col ? col : 1u;
		b3Shape_SetSurfaceMaterial( ch->shape, sm );
	}
}

int b3World_GetFractureBodyCount( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->fractureWorld == NULL )
		return 0;
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	int n = 0;
	for ( int p = 0; p < fw->pieces.count; ++p )
		if ( fw->pieces.data[p].active )
			n++;
	return n;
}

int b3World_GetFractureVoxelCount( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->fractureWorld == NULL )
		return 0;
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	int n = 0;
	for ( int v = 0; v < fw->voxels.count; ++v )
		if ( fw->voxels.data[v].piece >= 0 )
			n++;
	return n;
}

float b3World_GetFractureMaxStress( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	if ( world == NULL || world->fractureWorld == NULL )
		return 0.0f;
	b3FractureWorld* fw = (b3FractureWorld*)world->fractureWorld;
	float m = 0.0f;
	for ( int v = 0; v < fw->voxels.count; ++v )
		if ( fw->voxels.data[v].piece >= 0 )
			m = b3F_max( m, fw->voxels.data[v].stressShown );
	return m;
}
