#include "voxel_collide.h"

#include "core.h"
#include "shape.h"
#include "voxel_shape.h"

#include "box3d/collision.h"
#include "box3d/constants.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>

static inline float b3Voxel_comp( b3Vec3 v, int i )
{
	return ( &v.x )[i];
}

static float b3Obb_projRadius( const b3VoxelOBB* b, b3Vec3 a )
{
	return b3AbsFloat( b3Dot( a, b->axes[0] ) ) * b->half.x + b3AbsFloat( b3Dot( a, b->axes[1] ) ) * b->half.y +
		   b3AbsFloat( b3Dot( a, b->axes[2] ) ) * b->half.z;
}

static b3Vec3 b3Obb_support( const b3VoxelOBB* b, b3Vec3 d )
{
	b3Vec3 p = b->center;
	for ( int i = 0; i < 3; ++i )
	{
		float dp = b3Dot( d, b->axes[i] );
		float h = b3Voxel_comp( b->half, i );
		if ( dp > 1e-8f )
			p = b3MulAdd( p, h, b->axes[i] );
		else if ( dp < -1e-8f )
			p = b3MulAdd( p, -h, b->axes[i] );
	}
	return p;
}

b3AABB b3VoxelOBB_Bounds( const b3VoxelOBB* b )
{
	b3Vec3 r;
	for ( int c = 0; c < 3; ++c )
	{
		float rc = b3AbsFloat( b3Voxel_comp( b->axes[0], c ) ) * b->half.x +
				   b3AbsFloat( b3Voxel_comp( b->axes[1], c ) ) * b->half.y +
				   b3AbsFloat( b3Voxel_comp( b->axes[2], c ) ) * b->half.z;
		( &r.x )[c] = rc;
	}
	return (b3AABB){ b3Sub( b->center, r ), b3Add( b->center, r ) };
}

typedef struct b3ObbSat
{
	float minOverlap;
	b3Vec3 minAxis; // B -> A
	int srcBox;		// which box owns the min face axis (0 or 1); irrelevant for edges
	bool isEdge;
	int sepCount;
	b3Vec3 sepAxis;
	bool separated;
} b3ObbSat;

static bool b3Obb_testAxis( b3ObbSat* sat, const b3VoxelOBB* o0, const b3VoxelOBB* o1, b3Vec3 delta, b3Vec3 a, int srcBox,
							bool isEdge, float contactDist )
{
	float lenSq = b3Dot( a, a );
	float gate = isEdge ? 1e-4f : 1e-10f;
	if ( lenSq <= gate )
		return true; // near-parallel edge cross / degenerate: not a real axis
	a = b3MulSV( 1.0f / sqrtf( lenSq ), a );

	float proj = b3Dot( a, delta );
	float overlap = b3Obb_projRadius( o0, a ) + b3Obb_projRadius( o1, a ) - b3AbsFloat( proj );

	if ( overlap + contactDist <= 0.0f )
	{
		sat->separated = true;
		return false;
	}
	if ( overlap <= 0.0f )
	{
		if ( sat->sepCount == 0 || b3AbsFloat( b3Dot( a, sat->sepAxis ) ) < 0.999f )
		{
			sat->sepCount += 1;
			sat->sepAxis = a;
			if ( sat->sepCount > 1 )
			{
				sat->separated = true;
				return false;
			}
		}
	}

	bool improve = isEdge ? ( overlap + 1e-4f < sat->minOverlap ) : ( overlap < sat->minOverlap + 1e-4f );
	if ( improve )
	{
		sat->minOverlap = overlap;
		sat->minAxis = ( proj >= 0.0f ) ? a : b3Neg( a );
		sat->srcBox = srcBox;
		sat->isEdge = isEdge;
	}
	return true;
}

static void b3Obb_facePolygon( const b3VoxelOBB* b, b3Vec3 d, b3Vec3 out[4] )
{
	int best = 0;
	float bestAbs = -FLT_MAX;
	float sign = 1.0f;
	for ( int i = 0; i < 3; ++i )
	{
		float dp = b3Dot( d, b->axes[i] );
		if ( b3AbsFloat( dp ) > bestAbs )
		{
			bestAbs = b3AbsFloat( dp );
			best = i;
			sign = dp >= 0.0f ? 1.0f : -1.0f;
		}
	}
	int u = ( best + 1 ) % 3, w = ( best + 2 ) % 3;
	float hb = b3Voxel_comp( b->half, best ), hu = b3Voxel_comp( b->half, u ), hw = b3Voxel_comp( b->half, w );
	b3Vec3 fc = b3MulAdd( b->center, sign * hb, b->axes[best] );
	b3Vec3 au = b->axes[u], aw = b->axes[w];
	out[0] = b3MulAdd( b3MulAdd( fc, -hu, au ), -hw, aw );
	out[1] = b3MulAdd( b3MulAdd( fc, +hu, au ), -hw, aw );
	out[2] = b3MulAdd( b3MulAdd( fc, +hu, au ), +hw, aw );
	out[3] = b3MulAdd( b3MulAdd( fc, -hu, au ), +hw, aw );
}

static int b3Obb_clipPlane( const b3Vec3* in, int nin, b3Vec3 n, float offset, b3Vec3* out )
{
	int nout = 0;
	for ( int i = 0; i < nin; ++i )
	{
		b3Vec3 a = in[i], b = in[( i + 1 ) % nin];
		float da = b3Dot( n, a ) - offset;
		float db = b3Dot( n, b ) - offset;
		bool ain = da <= 1e-8f, bin = db <= 1e-8f;
		if ( ain )
			out[nout++] = a;
		if ( ain != bin )
		{
			float denom = da - db;
			if ( b3AbsFloat( denom ) > 1e-12f )
			{
				float t = da / denom;
				out[nout++] = b3MulAdd( a, t, b3Sub( b, a ) );
			}
		}
	}
	return nout;
}

static int b3Obb_reduce( const b3VoxelContact* cand, int ncand, int maxContacts, b3VoxelContact* out )
{
	if ( ncand <= maxContacts )
	{
		for ( int i = 0; i < ncand; ++i )
			out[i] = cand[i];
		return ncand;
	}
	bool used[8] = { false };
	int deepest = 0;
	for ( int i = 1; i < ncand; ++i )
		if ( cand[i].initialPenetration > cand[deepest].initialPenetration )
			deepest = i;
	int nsel = 0;
	out[nsel++] = cand[deepest];
	used[deepest] = true;
	while ( nsel < maxContacts )
	{
		int bestIdx = -1;
		float bestMin = -1.0f;
		for ( int i = 0; i < ncand; ++i )
		{
			if ( used[i] )
				continue;
			float minD = FLT_MAX;
			for ( int j = 0; j < nsel; ++j )
			{
				b3Vec3 diff = b3Sub( cand[i].body0Point, out[j].body0Point );
				float d = b3Dot( diff, diff );
				if ( d < minD )
					minD = d;
			}
			if ( minD > bestMin )
			{
				bestMin = minD;
				bestIdx = i;
			}
		}
		if ( bestIdx < 0 )
			break;
		out[nsel++] = cand[bestIdx];
		used[bestIdx] = true;
	}
	return nsel;
}

int b3VoxelCollideOBB( const b3VoxelOBB* o0, const b3VoxelOBB* o1, float contactDistance, int maxContacts, b3VoxelContact* out )
{
	if ( maxContacts < 1 )
		return 0;
	if ( maxContacts > 4 )
		maxContacts = 4;

	b3Vec3 delta = b3Sub( o0->center, o1->center );
	b3ObbSat sat = { FLT_MAX, { 0.0f, 0.0f, 1.0f }, 0, false, 0, { 0.0f, 0.0f, 0.0f }, false };

	for ( int i = 0; i < 3; ++i )
		if ( !b3Obb_testAxis( &sat, o0, o1, delta, o0->axes[i], 0, false, contactDistance ) )
			return 0;
	for ( int i = 0; i < 3; ++i )
		if ( !b3Obb_testAxis( &sat, o0, o1, delta, o1->axes[i], 1, false, contactDistance ) )
			return 0;
	// 9 edge-edge cross axes.
	for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j )
			if ( !b3Obb_testAxis( &sat, o0, o1, delta, b3Cross( o0->axes[i], o1->axes[j] ), 0, true, contactDistance ) )
				return 0;

	if ( sat.minOverlap == FLT_MAX )
		return 0; // never found a valid axis (degenerate boxes)

	if ( maxContacts == 1 || sat.isEdge || sat.minOverlap > 0.5f )
	{
		b3Vec3 mid = b3MulSV( 0.5f, b3Add( b3Obb_support( o0, b3Neg( sat.minAxis ) ), b3Obb_support( o1, sat.minAxis ) ) );
		out[0].normal = sat.minAxis;
		out[0].initialPenetration = sat.minOverlap;
		out[0].penetrationDepth = sat.minOverlap > 0.0f ? sat.minOverlap : sat.minOverlap + contactDistance;
		out[0].body0Point = b3MulAdd( mid, -( sat.minOverlap * 0.5f ), sat.minAxis );
		out[0].body1Point = b3MulAdd( mid, +( sat.minOverlap * 0.5f ), sat.minAxis );
		return 1;
	}

	bool refIsBody0 = ( sat.srcBox == 0 );
	const b3VoxelOBB* refBox = refIsBody0 ? o0 : o1;
	const b3VoxelOBB* incBox = refIsBody0 ? o1 : o0;
	b3Vec3 refNormal = refIsBody0 ? b3Neg( sat.minAxis ) : sat.minAxis; // ref face outward normal
	b3Vec3 refFaceCenter = b3Obb_support( refBox, refNormal );

	int refAxis = 0;
	float bestAbs = -FLT_MAX;
	for ( int i = 0; i < 3; ++i )
	{
		float dp = b3AbsFloat( b3Dot( refNormal, refBox->axes[i] ) );
		if ( dp > bestAbs )
		{
			bestAbs = dp;
			refAxis = i;
		}
	}

	b3Vec3 poly[8], tmp[8];
	b3Obb_facePolygon( incBox, b3Neg( refNormal ), poly );
	int npoly = 4;
	for ( int i = 0; i < 3; ++i )
	{
		if ( i == refAxis )
			continue;
		float c = b3Dot( refBox->axes[i], refBox->center );
		float h = b3Voxel_comp( refBox->half, i );
		npoly = b3Obb_clipPlane( poly, npoly, refBox->axes[i], c + h, tmp );
		for ( int k = 0; k < npoly; ++k )
			poly[k] = tmp[k];
		npoly = b3Obb_clipPlane( poly, npoly, b3Neg( refBox->axes[i] ), h - c, tmp );
		for ( int k = 0; k < npoly; ++k )
			poly[k] = tmp[k];
	}

	b3VoxelContact cand[8];
	int ncand = 0;
	for ( int i = 0; i < npoly && ncand < 8; ++i )
	{
		b3Vec3 pt = poly[i];
		float sd = b3Dot( b3Sub( pt, refFaceCenter ), refNormal );
		float initPen = -sd;
		if ( initPen + contactDistance <= 0.0f )
			continue;
		b3Vec3 refPt = b3MulAdd( pt, -sd, refNormal );
		b3VoxelContact c;
		c.normal = sat.minAxis;
		c.initialPenetration = initPen;
		c.penetrationDepth = initPen > 0.0f ? initPen : initPen + contactDistance;
		if ( refIsBody0 )
		{
			c.body0Point = refPt; // on box0 (A) ref face
			c.body1Point = pt;	  // on box1 (B) incident face
		}
		else
		{
			c.body0Point = pt;
			c.body1Point = refPt;
		}
		bool dup = false;
		for ( int j = 0; j < ncand; ++j )
		{
			b3Vec3 diff = b3Sub( cand[j].body0Point, c.body0Point );
			if ( b3Dot( diff, diff ) < 1e-12f )
			{
				dup = true;
				break;
			}
		}
		if ( !dup )
			cand[ncand++] = c;
	}

	if ( ncand == 0 )
	{
		b3Vec3 mid = b3MulSV( 0.5f, b3Add( b3Obb_support( o0, b3Neg( sat.minAxis ) ), b3Obb_support( o1, sat.minAxis ) ) );
		out[0].normal = sat.minAxis;
		out[0].initialPenetration = sat.minOverlap;
		out[0].penetrationDepth = sat.minOverlap > 0.0f ? sat.minOverlap : sat.minOverlap + contactDistance;
		out[0].body0Point = b3MulAdd( mid, -( sat.minOverlap * 0.5f ), sat.minAxis );
		out[0].body1Point = b3MulAdd( mid, +( sat.minOverlap * 0.5f ), sat.minAxis );
		return 1;
	}

	return b3Obb_reduce( cand, ncand, maxContacts, out );
}

int b3VoxelCollideAABB( const b3AABB* a0, const b3AABB* a1, float contactDistance, int maxContacts, b3VoxelContact* out )
{
	if ( maxContacts < 1 )
		return 0;
	if ( maxContacts > 4 )
		maxContacts = 4;

	float overlap[3], rectMin[3], rectMax[3], c0[3], c1[3];
	int sep = 0;
	for ( int i = 0; i < 3; ++i )
	{
		float lo0 = b3Voxel_comp( a0->lowerBound, i ), hi0 = b3Voxel_comp( a0->upperBound, i );
		float lo1 = b3Voxel_comp( a1->lowerBound, i ), hi1 = b3Voxel_comp( a1->upperBound, i );
		rectMin[i] = b3MaxFloat( lo0, lo1 );
		rectMax[i] = b3MinFloat( hi0, hi1 );
		overlap[i] = rectMax[i] - rectMin[i];
		c0[i] = 0.5f * ( lo0 + hi0 );
		c1[i] = 0.5f * ( lo1 + hi1 );
		if ( overlap[i] + contactDistance <= 0.0f )
			return 0;
		if ( overlap[i] < 0.0f && ++sep > 1 )
			return 0;
	}

	int axis = 0;
	if ( overlap[1] < overlap[axis] )
		axis = 1;
	if ( overlap[2] < overlap[axis] )
		axis = 2;

	float ov = overlap[axis];
	b3Vec3 normal = b3Vec3_zero;
	( &normal.x )[axis] = c0[axis] >= c1[axis] ? 1.0f : -1.0f; // B -> A
	float depth = ov > 0.0f ? ov : ov + contactDistance;
	float planePos = 0.5f * ( rectMin[axis] + rectMax[axis] );

	int t1 = ( axis + 1 ) % 3, t2 = ( axis + 2 ) % 3;

	const float minPos = 1e-6f;
	float t1lo = rectMin[t1], t1hi = b3MaxFloat( rectMax[t1], rectMin[t1] + minPos );
	float t2lo = rectMin[t2], t2hi = b3MaxFloat( rectMax[t2], rectMin[t2] + minPos );
	float corners[4][2] = { { t1lo, t2lo }, { t1hi, t2hi }, { t1lo, t2hi }, { t1hi, t2lo } };

	int n = ( maxContacts < 4 ) ? maxContacts : 4;
	if ( ov > 0.4f * b3MinFloat( overlap[t1] + minPos, overlap[t2] + minPos ) )
	{
		n = 1;
	}
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 patch;
		( &patch.x )[axis] = planePos;
		( &patch.x )[t1] = ( n == 1 ) ? 0.5f * ( t1lo + t1hi ) : corners[i][0];
		( &patch.x )[t2] = ( n == 1 ) ? 0.5f * ( t2lo + t2hi ) : corners[i][1];
		out[i].normal = normal;
		out[i].initialPenetration = ov;
		out[i].penetrationDepth = depth;
		out[i].body0Point = b3MulAdd( patch, -( ov * 0.5f ), normal );
		out[i].body1Point = b3MulAdd( patch, +( ov * 0.5f ), normal );
	}
	return n;
}

#define B3_VOXEL_MAX_CONTACTS 64

static inline b3Vec3 b3Voxel_xfPoint( b3Transform xf, b3Vec3 local )
{
	return b3Add( xf.p, b3RotateVector( xf.q, local ) );
}

static inline b3Vec3 b3Voxel_invXfPoint( b3Transform xf, b3Vec3 world )
{
	return b3InvRotateVector( xf.q, b3Sub( world, xf.p ) );
}

static b3AABB b3Voxel_mapBounds( b3AABB box, b3Transform xf, bool inverse )
{
	b3AABB r = { { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };
	for ( int i = 0; i < 8; ++i )
	{
		b3Vec3 c = { ( i & 1 ) ? box.upperBound.x : box.lowerBound.x, ( i & 2 ) ? box.upperBound.y : box.lowerBound.y,
					 ( i & 4 ) ? box.upperBound.z : box.lowerBound.z };
		b3Vec3 w = inverse ? b3Voxel_invXfPoint( xf, c ) : b3Voxel_xfPoint( xf, c );
		r.lowerBound.x = b3MinFloat( r.lowerBound.x, w.x );
		r.lowerBound.y = b3MinFloat( r.lowerBound.y, w.y );
		r.lowerBound.z = b3MinFloat( r.lowerBound.z, w.z );
		r.upperBound.x = b3MaxFloat( r.upperBound.x, w.x );
		r.upperBound.y = b3MaxFloat( r.upperBound.y, w.y );
		r.upperBound.z = b3MaxFloat( r.upperBound.z, w.z );
	}
	return r;
}

static inline b3AABB b3Voxel_expandB( b3AABB b, float a )
{
	if ( a <= 0.0f )
		return b;
	b3Vec3 e = { a, a, a };
	return (b3AABB){ b3Sub( b.lowerBound, e ), b3Add( b.upperBound, e ) };
}

static inline bool b3Voxel_isect( b3AABB a, b3AABB b )
{
	return !( a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x || a.upperBound.y < b.lowerBound.y ||
			  a.lowerBound.y > b.upperBound.y || a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z );
}

static b3VoxelOBB b3Voxel_cellOBB( b3Vec3i cell, float voxelSize, b3Transform xf )
{
	b3VoxelOBB o;
	b3Vec3 localCenter = { cell.x * voxelSize, cell.y * voxelSize, cell.z * voxelSize };
	o.center = b3Voxel_xfPoint( xf, localCenter );
	o.axes[0] = b3RotateVector( xf.q, ( b3Vec3 ){ 1.0f, 0.0f, 0.0f } );
	o.axes[1] = b3RotateVector( xf.q, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } );
	o.axes[2] = b3RotateVector( xf.q, ( b3Vec3 ){ 0.0f, 0.0f, 1.0f } );
	float h = 0.5f * voxelSize;
	o.half = ( b3Vec3 ){ h, h, h };
	return o;
}

static b3Vec3i b3Voxel_dominantOffset( b3Vec3 d )
{
	float ax = b3AbsFloat( d.x ), ay = b3AbsFloat( d.y ), az = b3AbsFloat( d.z );
	if ( ax >= ay && ax >= az )
		return (b3Vec3i){ d.x >= 0.0f ? 1 : -1, 0, 0 };
	if ( ay >= az )
		return (b3Vec3i){ 0, d.y >= 0.0f ? 1 : -1, 0 };
	return (b3Vec3i){ 0, 0, d.z >= 0.0f ? 1 : -1 };
}

static bool b3Voxel_facesExposed( const b3VoxelData* v0, b3Vec3i cell0, b3Transform xf0, const b3VoxelData* v1,
								   b3Vec3i cell1, b3Transform xf1, b3Vec3 normalWorld )
{
	b3Vec3 d0 = b3InvRotateVector( xf0.q, b3Neg( normalWorld ) );
	b3Vec3i o0 = b3Voxel_dominantOffset( d0 );
	if ( b3VoxelData_IsSolid( v0, (b3Vec3i){ cell0.x + o0.x, cell0.y + o0.y, cell0.z + o0.z } ) )
		return false;
	b3Vec3 d1 = b3InvRotateVector( xf1.q, normalWorld );
	b3Vec3i o1 = b3Voxel_dominantOffset( d1 );
	if ( b3VoxelData_IsSolid( v1, (b3Vec3i){ cell1.x + o1.x, cell1.y + o1.y, cell1.z + o1.z } ) )
		return false;
	return true;
}

static float b3Voxel_spreadScore( const b3VoxelContact* c, const b3VoxelContact* set, int n, int skip )
{
	float minD = FLT_MAX;
	for ( int j = 0; j < n; ++j )
	{
		if ( j == skip )
			continue;
		b3Vec3 diff = b3Sub( c->body0Point, set[j].body0Point );
		float d = b3Dot( diff, diff );
		if ( d < minD )
			minD = d;
	}
	if ( minD == FLT_MAX )
		minD = 0.0f;
	return c->penetrationDepth * ( 1.0f + sqrtf( minD ) );
}

void b3Voxel_addReduced( const b3VoxelContact* c, b3VoxelContact* acc, int* nacc, int maxContacts )
{
	if ( *nacc < maxContacts )
	{
		acc[( *nacc )++] = *c;
		return;
	}
	float newScore = b3Voxel_spreadScore( c, acc, *nacc, -1 );
	int weakest = 0;
	float weakScore = FLT_MAX;
	for ( int i = 0; i < *nacc; ++i )
	{
		float score = b3Voxel_spreadScore( &acc[i], acc, *nacc, i );
		if ( score < weakScore )
		{
			weakScore = score;
			weakest = i;
		}
	}
	if ( newScore >= weakScore )
		acc[weakest] = *c;
}

static uint32_t b3Voxel_cellHash( b3Vec3i c )
{
	uint32_t h = (uint32_t)( c.x * 73856093 ) ^ (uint32_t)( c.y * 19349663 ) ^ (uint32_t)( c.z * 83492791 );
	h ^= h >> 13;
	h *= 0x5bd1e995u;
	h ^= h >> 15;
	return h;
}

static uint32_t b3Voxel_pairId( b3Vec3i a, b3Vec3i b, int k )
{
	uint32_t h = b3Voxel_cellHash( a ) * 0x9e3779b1u + b3Voxel_cellHash( b );
	h ^= (uint32_t)k * 0x85ebca6bu;
	return h ? h : 1u;
}

static void b3Voxel_order( b3VoxelContact* c, int n )
{
	for ( int i = 1; i < n; ++i )
	{
		b3VoxelContact key = c[i];
		int j = i - 1;
		while ( j >= 0 )
		{
			const b3VoxelContact* a = &c[j];
			bool greater = a->initialPenetration < key.initialPenetration ||
						   ( a->initialPenetration == key.initialPenetration &&
							 ( a->body0Point.x > key.body0Point.x ||
							   ( a->body0Point.x == key.body0Point.x && a->body0Point.y > key.body0Point.y ) ) );
			if ( !greater )
				break;
			c[j + 1] = c[j];
			j--;
		}
		c[j + 1] = key;
	}
}

int b3VoxelCollide( const b3VoxelData* v0, b3Transform xf0, const b3VoxelData* v1, b3Transform xf1, float contactDistance,
					int maxContacts, b3VoxelContact* out )
{
	if ( maxContacts < 1 )
		return 0;
	if ( maxContacts > B3_VOXEL_MAX_CONTACTS )
		maxContacts = B3_VOXEL_MAX_CONTACTS;

	b3AABB lb0, lb1;
	if ( !b3Voxel_GetLocalBounds( v0, &lb0 ) || !b3Voxel_GetLocalBounds( v1, &lb1 ) )
		return 0;
	if ( contactDistance < 0.0f )
		contactDistance = 0.0f;

	b3AABB wb0 = b3Voxel_mapBounds( lb0, xf0, false );
	b3AABB wb1 = b3Voxel_mapBounds( lb1, xf1, false );
	if ( !b3Voxel_isect( b3Voxel_expandB( wb0, contactDistance ), wb1 ) )
		return 0;

	float vs0 = b3Voxel_GetVoxelSize( v0 );
	float vs1 = b3Voxel_GetVoxelSize( v1 );

	// Solid cells of v0 in the region v1 could reach.
	b3AABB q0 = b3Voxel_mapBounds( b3Voxel_expandB( wb1, contactDistance ), xf0, true );
	int cap0 = b3Voxel_GetCellCount( v0 );
	if ( cap0 > 8192 )
		cap0 = 8192;
	b3Vec3i* cells0 = (b3Vec3i*)b3Alloc( (size_t)cap0 * sizeof( b3Vec3i ) );
	int nc0 = b3Voxel_QueryCells( v0, q0, cells0, cap0 );

	b3VoxelContact acc[B3_VOXEL_MAX_CONTACTS];
	int nacc = 0;
	b3Vec3i cells1[256];

	for ( int a = 0; a < nc0; ++a )
	{
		b3VoxelOBB obb0 = b3Voxel_cellOBB( cells0[a], vs0, xf0 );
		b3AABB ewb0 = b3Voxel_expandB( b3VoxelOBB_Bounds( &obb0 ), contactDistance );
		b3AABB q1 = b3Voxel_mapBounds( ewb0, xf1, true );
		int nc1 = b3Voxel_QueryCells( v1, q1, cells1, 256 );

		for ( int b = 0; b < nc1; ++b )
		{
			b3VoxelOBB obb1 = b3Voxel_cellOBB( cells1[b], vs1, xf1 );
			if ( !b3Voxel_isect( ewb0, b3VoxelOBB_Bounds( &obb1 ) ) )
				continue;

			b3VoxelContact c[4];
			// Request the face-clip manifold (up to 4 corner points) per voxel pair, not a single
			// support point. A single point lands at the voxel FACE CENTRE, so an N-wide resting
			// face gets a support polygon of span (N-1)*voxelSize and a single-voxel contact
			// collapses to one central point with ZERO base -> free to rock/tip. The clipped
			// corners reach the true face edges, restoring a full-width support base. Edge and
			// deep-overlap contacts still fall back to the single-point branch inside CollideOBB.
			int nc = b3VoxelCollideOBB( &obb0, &obb1, contactDistance, 4, c );
			for ( int k = 0; k < nc; ++k )
			{
				if ( !b3Voxel_facesExposed( v0, cells0[a], xf0, v1, cells1[b], xf1, c[k].normal ) )
					continue;
				c[k].featureId = b3Voxel_pairId( cells0[a], cells1[b], k );
				b3Voxel_addReduced( &c[k], acc, &nacc, maxContacts );
			}
		}
	}

	b3Free( cells0, (size_t)cap0 * sizeof( b3Vec3i ) );

	b3Voxel_order( acc, nacc );
	for ( int i = 0; i < nacc; ++i )
		out[i] = acc[i];
	return nacc;
}

// ---------------------------------------------------------------------------
// Ray cast and overlap query (shape-local space).
// ---------------------------------------------------------------------------

// Amanatides-Woo 3D DDA over the cell-centred grid. Cell c occupies
// [c*s - 0.5s, c*s + 0.5s]; the grid coordinate u = p/s + 0.5 puts cell c at
// the integer interval [c, c+1). Reports the fraction (of the ray translation)
// at which the ray first enters a solid cell and the face normal it crossed.
b3CastOutput b3RayCastVoxel( const b3VoxelData* v, const b3RayCastInput* input )
{
	b3CastOutput out = { 0 };

	if ( b3Voxel_GetCellCount( v ) == 0 )
	{
		return out;
	}

	float s = b3Voxel_GetVoxelSize( v );
	float inv = 1.0f / s;

	b3Vec3 p0 = input->origin;
	b3Vec3 d = input->translation;

	// Clip the ray to the local bounds to bound the march (and to find the entry t).
	b3AABB bounds;
	b3Voxel_GetLocalBounds( v, &bounds );

	float tmin = 0.0f;
	float tmax = input->maxFraction;
	for ( int a = 0; a < 3; ++a )
	{
		float o = ( &p0.x )[a];
		float dir = ( &d.x )[a];
		float lo = ( &bounds.lowerBound.x )[a];
		float hi = ( &bounds.upperBound.x )[a];
		if ( b3AbsFloat( dir ) < 1e-9f )
		{
			if ( o < lo || o > hi )
			{
				return out;
			}
		}
		else
		{
			float t1 = ( lo - o ) / dir;
			float t2 = ( hi - o ) / dir;
			if ( t1 > t2 )
			{
				float tt = t1;
				t1 = t2;
				t2 = tt;
			}
			tmin = b3MaxFloat( tmin, t1 );
			tmax = b3MinFloat( tmax, t2 );
			if ( tmin > tmax )
			{
				return out;
			}
		}
	}

	// Entry point and grid setup.
	b3Vec3 pe = { p0.x + tmin * d.x, p0.y + tmin * d.y, p0.z + tmin * d.z };

	int cell[3];
	int step[3];
	float tMax[3];
	float tDelta[3];
	for ( int a = 0; a < 3; ++a )
	{
		float ue = ( &pe.x )[a] * inv + 0.5f;
		float du = ( &d.x )[a] * inv;
		cell[a] = (int)floorf( ue );
		if ( du > 1e-12f )
		{
			step[a] = 1;
			tMax[a] = tmin + ( (float)( cell[a] + 1 ) - ue ) / du;
			tDelta[a] = 1.0f / du;
		}
		else if ( du < -1e-12f )
		{
			step[a] = -1;
			tMax[a] = tmin + ( (float)cell[a] - ue ) / du;
			tDelta[a] = -1.0f / du;
		}
		else
		{
			step[a] = 0;
			tMax[a] = FLT_MAX;
			tDelta[a] = FLT_MAX;
		}
	}

	float tEnter = tmin;
	int lastAxis = -1;
	for ( int iter = 0; iter < 4096; ++iter )
	{
		if ( b3VoxelData_IsSolid( v, (b3Vec3i){ cell[0], cell[1], cell[2] } ) )
		{
			out.hit = true;
			out.fraction = tEnter;
			out.point = (b3Vec3){ p0.x + tEnter * d.x, p0.y + tEnter * d.y, p0.z + tEnter * d.z };
			if ( lastAxis < 0 )
			{
				// Ray began inside a solid cell.
				out.normal = b3Normalize( b3Neg( d ) );
			}
			else
			{
				out.normal = b3Vec3_zero;
				( &out.normal.x )[lastAxis] = (float)( -step[lastAxis] );
			}
			return out;
		}

		int mi = 0;
		if ( tMax[1] < tMax[mi] )
			mi = 1;
		if ( tMax[2] < tMax[mi] )
			mi = 2;

		if ( tMax[mi] > tmax )
		{
			break;
		}

		tEnter = tMax[mi];
		cell[mi] += step[mi];
		lastAxis = mi;
		tMax[mi] += tDelta[mi];
	}

	return out;
}

// Overlap query: is any solid cell within the query proxy? The proxy is mapped
// into shape-local space, then each candidate cell's box is distance-tested
// against it (mirrors b3OverlapHeightField's per-triangle GJK test).
bool b3OverlapVoxel( const b3VoxelData* v, b3Transform xf, const b3ShapeProxy* proxy )
{
	if ( b3Voxel_GetCellCount( v ) == 0 )
	{
		return false;
	}

	b3Vec3 buffer[B3_MAX_SHAPE_CAST_POINTS];
	b3ShapeProxy localProxy = b3MakeLocalProxy( proxy, xf, buffer );
	b3AABB aabb = b3ComputeProxyAABB( &localProxy );

	float s = b3Voxel_GetVoxelSize( v );
	float half = 0.5f * s;

	// Grow the broadphase query by a half cell so a proxy grazing a cell face is
	// still considered; the GJK test below decides the precise result.
	b3AABB query;
	query.lowerBound = (b3Vec3){ aabb.lowerBound.x - half, aabb.lowerBound.y - half, aabb.lowerBound.z - half };
	query.upperBound = (b3Vec3){ aabb.upperBound.x + half, aabb.upperBound.y + half, aabb.upperBound.z + half };

	b3Vec3i cells[256];
	int n = b3Voxel_QueryCells( v, query, cells, 256 );
	if ( n == 0 )
	{
		return false;
	}

	b3DistanceInput input;
	input.proxyB = localProxy;
	input.transform = b3Transform_identity;
	input.useRadii = true;

	b3SimplexCache cache = { 0 };
	float tolerance = 0.1f * B3_LINEAR_SLOP;

	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 c = { cells[i].x * s, cells[i].y * s, cells[i].z * s };
		b3Vec3 corners[8];
		for ( int k = 0; k < 8; ++k )
		{
			corners[k] = (b3Vec3){
				c.x + ( ( k & 1 ) ? half : -half ),
				c.y + ( ( k & 2 ) ? half : -half ),
				c.z + ( ( k & 4 ) ? half : -half ),
			};
		}

		input.proxyA = (b3ShapeProxy){ corners, 8, 0.0f };
		cache.count = 0;

		b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
		if ( output.distance < tolerance )
		{
			return true;
		}
	}

	return false;
}
