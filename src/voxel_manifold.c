#include "contact.h"

#include "arena_allocator.h" // b3Bump
#include "core.h"
#include "manifold.h" // b3MakeFeatureId
#include "physics_world.h"
#include "shape.h"
#include "voxel_collide.h"
#include "voxel_shape.h"

#include "box3d/collision.h" // b3CollideHullAnd*, b3MakeOffsetBoxHull
#include "box3d/constants.h"
#include "box3d/types.h"

#include <float.h>
#include <string.h>

#define B3_VOXEL_MAX_CONTACTS 64

#define B3_VOXEL_CLUSTER_DOT 0.996f

#define B3_VOXEL_NORMAL_MATCH 0.995f

typedef struct b3VoxelOldManifold
{
	b3Vec3 normal;
	b3Vec3 frictionImpulse;
	b3Vec3 rollingImpulse;
	float twistImpulse;
	int pointCount;
	uint32_t featureId[B3_MAX_MANIFOLD_POINTS];
	float normalImpulse[B3_MAX_MANIFOLD_POINTS];
	bool claimed[B3_MAX_MANIFOLD_POINTS];
	bool consumed;
} b3VoxelOldManifold;

static int b3Voxel_reduceCluster( const b3VoxelContact* contacts, const int* members, int n, int maxPts, int* outIdx )
{
	if ( n <= maxPts )
	{
		for ( int i = 0; i < n; ++i )
			outIdx[i] = members[i];
		return n;
	}

	bool used[B3_VOXEL_MAX_CONTACTS] = { false };

	int deepest = 0;
	for ( int i = 1; i < n; ++i )
	{
		if ( contacts[members[i]].initialPenetration > contacts[members[deepest]].initialPenetration )
			deepest = i;
	}

	int nsel = 0;
	outIdx[nsel++] = members[deepest];
	used[deepest] = true;

	while ( nsel < maxPts )
	{
		int bestIdx = -1;
		float bestMin = -1.0f;
		for ( int i = 0; i < n; ++i )
		{
			if ( used[i] )
				continue;
			float minD = FLT_MAX;
			for ( int j = 0; j < nsel; ++j )
			{
				b3Vec3 diff = b3Sub( contacts[members[i]].body0Point, contacts[outIdx[j]].body0Point );
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
		outIdx[nsel++] = members[bestIdx];
		used[bestIdx] = true;
	}
	return nsel;
}

static bool b3Voxel_emitManifolds( b3World* world, b3Contact* contact, const b3VoxelContact* contacts, int count,
								   b3WorldTransform xfA, b3WorldTransform xfB, const b3Shape* shapeA, const b3Shape* shapeB )
{
	if ( count == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
		}
		return false;
	}

	b3VoxelOldManifold oldManifolds[B3_VOXEL_MAX_CONTACTS];
	int oldCount = contact->manifoldCount;
	if ( oldCount > B3_VOXEL_MAX_CONTACTS )
		oldCount = B3_VOXEL_MAX_CONTACTS;
	for ( int i = 0; i < oldCount; ++i )
	{
		const b3Manifold* m = contact->manifolds + i;
		b3VoxelOldManifold* o = oldManifolds + i;
		o->normal = m->normal;
		o->frictionImpulse = m->frictionImpulse;
		o->rollingImpulse = m->rollingImpulse;
		o->twistImpulse = m->twistImpulse;
		o->pointCount = m->pointCount;
		o->consumed = false;
		for ( int j = 0; j < m->pointCount && j < B3_MAX_MANIFOLD_POINTS; ++j )
		{
			o->featureId[j] = m->points[j].featureId;
			o->normalImpulse[j] = m->points[j].normalImpulse;
			o->claimed[j] = false;
		}
	}

	int clusterOf[B3_VOXEL_MAX_CONTACTS];
	b3Vec3 clusterNormal[B3_VOXEL_MAX_CONTACTS];
	int clusterCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		int cl = -1;
		for ( int k = 0; k < clusterCount; ++k )
		{
			if ( b3Dot( clusterNormal[k], contacts[i].normal ) > B3_VOXEL_CLUSTER_DOT )
			{
				cl = k;
				break;
			}
		}
		if ( cl < 0 )
		{
			cl = clusterCount;
			clusterNormal[clusterCount] = contacts[i].normal;
			clusterCount += 1;
		}
		clusterOf[i] = cl;
	}

	if ( contact->manifoldCount != clusterCount )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
		}
		contact->manifolds = b3AllocateManifolds( world, clusterCount );
		contact->manifoldCount = (uint16_t)clusterCount;
	}
	else
	{
		memset( contact->manifolds, 0, contact->manifoldCount * sizeof( b3Manifold ) );
	}

	b3Matrix3 matrixA = b3MakeMatrixFromQuat( xfA.q );
	b3Vec3 offsetAB = b3SubPos( xfA.p, xfB.p );

	for ( int cl = 0; cl < clusterCount; ++cl )
	{
		int members[B3_VOXEL_MAX_CONTACTS];
		int n = 0;
		for ( int i = 0; i < count; ++i )
		{
			if ( clusterOf[i] == cl )
				members[n++] = i;
		}

		int sel[B3_MAX_MANIFOLD_POINTS];
		int nsel = b3Voxel_reduceCluster( contacts, members, n, B3_MAX_MANIFOLD_POINTS, sel );

		b3Manifold* manifold = contact->manifolds + cl;
		manifold->pointCount = nsel;

		manifold->normal = b3MulMV( matrixA, b3Neg( clusterNormal[cl] ) );

		for ( int j = 0; j < nsel; ++j )
		{
			const b3VoxelContact* c = contacts + sel[j];
			b3ManifoldPoint* mp = manifold->points + j;
			b3Vec3 point = b3MulMV( matrixA, b3MulSV( 0.5f, b3Add( c->body0Point, c->body1Point ) ) );
			mp->anchorA = point;
			mp->anchorB = b3Add( point, offsetAB );
			mp->separation = -c->initialPenetration;
			mp->featureId = c->featureId;
			mp->triangleIndex = B3_NULL_INDEX;
			mp->normalVelocity = 0.0f;
			mp->totalNormalImpulse = 0.0f;
			mp->normalImpulse = 0.0f;
			mp->persisted = false;
		}

		int best = -1;
		float bestDot = B3_VOXEL_NORMAL_MATCH;
		for ( int k = 0; k < oldCount; ++k )
		{
			if ( oldManifolds[k].consumed )
				continue;
			float d = b3Dot( oldManifolds[k].normal, manifold->normal );
			if ( d > bestDot )
			{
				bestDot = d;
				best = k;
			}
		}
		if ( best >= 0 )
		{
			b3VoxelOldManifold* om = oldManifolds + best;
			manifold->frictionImpulse = om->frictionImpulse;
			manifold->rollingImpulse = om->rollingImpulse;
			manifold->twistImpulse = om->twistImpulse;
			om->consumed = true;

			for ( int j = 0; j < nsel; ++j )
			{
				b3ManifoldPoint* mp = manifold->points + j;
				for ( int k = 0; k < om->pointCount; ++k )
				{
					if ( !om->claimed[k] && om->featureId[k] == mp->featureId )
					{
						mp->normalImpulse = om->normalImpulse[k];
						mp->persisted = true;
						om->claimed[k] = true; // claim so it isn't reused (featureId space is unrestricted)
						break;
					}
				}
			}
		}
	}

	const b3SurfaceMaterial* materialA = b3GetShapeMaterials( shapeA );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB );
	contact->friction =
		world->frictionCallback( materialA->friction, materialA->userMaterialId, materialB->friction, materialB->userMaterialId );
	contact->restitution = world->restitutionCallback( materialA->restitution, materialA->userMaterialId, materialB->restitution,
													   materialB->userMaterialId );
	contact->rollingResistance = 0.0f;
	contact->tangentVelocity =
		b3Sub( b3RotateVector( xfA.q, materialA->tangentVelocity ), b3RotateVector( xfB.q, materialB->tangentVelocity ) );

	return true;
}

static uint32_t b3Voxel_cellId( b3Vec3i c )
{
	uint32_t h = (uint32_t)( c.x * 73856093 ) ^ (uint32_t)( c.y * 19349663 ) ^ (uint32_t)( c.z * 83492791 );
	h ^= h >> 16;
	h *= 0x7feb352du;
	h ^= h >> 15;
	h *= 0x846ca68bu;
	h ^= h >> 16;
	return h ? h : 1u;
}

static int b3VoxelCollideConvex( const b3VoxelData* v, const b3Shape* convex, b3Transform btoa, float contactDistance,
								 int maxContacts, b3VoxelContact* out, b3Arena* arena )
{
	float vs = b3Voxel_GetVoxelSize( v );
	float h = 0.5f * vs;

	b3Vec3 lo, hi;
	if ( convex->type == b3_sphereShape )
	{
		b3Vec3 c = b3TransformPoint( btoa, convex->sphere.center );
		float r = convex->sphere.radius;
		lo = ( b3Vec3 ){ c.x - r, c.y - r, c.z - r };
		hi = ( b3Vec3 ){ c.x + r, c.y + r, c.z + r };
	}
	else if ( convex->type == b3_capsuleShape )
	{
		b3Vec3 c1 = b3TransformPoint( btoa, convex->capsule.center1 );
		b3Vec3 c2 = b3TransformPoint( btoa, convex->capsule.center2 );
		float r = convex->capsule.radius;
		lo = ( b3Vec3 ){ b3MinFloat( c1.x, c2.x ) - r, b3MinFloat( c1.y, c2.y ) - r, b3MinFloat( c1.z, c2.z ) - r };
		hi = ( b3Vec3 ){ b3MaxFloat( c1.x, c2.x ) + r, b3MaxFloat( c1.y, c2.y ) + r, b3MaxFloat( c1.z, c2.z ) + r };
	}
	else
	{
		b3AABB hb = b3AABB_Transform( btoa, convex->hull->aabb );
		lo = hb.lowerBound;
		hi = hb.upperBound;
	}
	float cd = contactDistance;
	b3AABB query = { { lo.x - cd, lo.y - cd, lo.z - cd }, { hi.x + cd, hi.y + cd, hi.z + cd } };

	float invVs = vs > 0.0f ? 1.0f / vs : 0.0f;
	long long ex = (long long)( ( query.upperBound.x - query.lowerBound.x ) * invVs ) + 2;
	long long ey = (long long)( ( query.upperBound.y - query.lowerBound.y ) * invVs ) + 2;
	long long ez = (long long)( ( query.upperBound.z - query.lowerBound.z ) * invVs ) + 2;
	long long est = ex * ey * ez;
	int cellCap = est > 8192 ? 8192 : ( est < 64 ? 64 : (int)est );
	b3Vec3i* cells = (b3Vec3i*)b3Bump( arena, cellCap * (int)sizeof( b3Vec3i ) );
	int ncells = b3Voxel_QueryCells( v, query, cells, cellCap );

	int nacc = 0;
	for ( int a = 0; a < ncells; ++a )
	{
		b3Vec3 center = { cells[a].x * vs, cells[a].y * vs, cells[a].z * vs };
		b3BoxHull cellHull = b3MakeOffsetBoxHull( h, h, h, center );

		b3LocalManifoldPoint pts[8];
		b3LocalManifold m = { 0 };
		m.points = pts;

		switch ( convex->type )
		{
			case b3_sphereShape:
			{
				b3SimplexCache sc = { 0 };
				b3CollideHullAndSphere( &m, 8, &cellHull.base, &convex->sphere, btoa, &sc );
				break;
			}
			case b3_capsuleShape:
			{
				b3SimplexCache sc = { 0 };
				b3CollideHullAndCapsule( &m, 8, &cellHull.base, &convex->capsule, btoa, &sc );
				break;
			}
			case b3_hullShape:
			{
				b3SATCache sc = { 0 };
				b3CollideHulls( &m, 8, &cellHull.base, convex->hull, btoa, &sc );
				break;
			}
			default:
				return nacc;
		}

		uint32_t cellId = b3Voxel_cellId( cells[a] );
		for ( int k = 0; k < m.pointCount; ++k )
		{
			if ( m.points[k].separation >= contactDistance )
				continue; // outside the contact band

			b3VoxelContact c;
			c.normal = b3Neg( m.normal ); // cell -> convex (A -> B) flipped to B -> A
			c.initialPenetration = -m.points[k].separation;
			c.penetrationDepth = c.initialPenetration > 0.0f ? c.initialPenetration : 0.0f;
			c.body0Point = m.points[k].point;
			c.body1Point = m.points[k].point;
			c.featureId = cellId ^ ( b3MakeFeatureId( m.points[k].pair ) * 2654435761u );
			b3Voxel_addReduced( &c, out, &nacc, maxContacts );
		}
	}
	return nacc;
}

bool b3ComputeVoxelManifolds( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA, b3WorldTransform xfA,
							  const b3Shape* shapeB, b3WorldTransform xfB, b3Arena arena )
{
	B3_UNUSED( workerIndex );

	b3Transform transformBtoA = b3InvMulWorldTransforms( xfA, xfB );

	float contactDistance = ( contact->flags & b3_enableSpeculativePoints ) ? B3_SPECULATIVE_DISTANCE : B3_LINEAR_SLOP;

	b3VoxelContact contacts[B3_VOXEL_MAX_CONTACTS];
	int count;
	if ( shapeB->type == b3_voxelShape )
	{
		count = b3VoxelCollide( shapeA->voxel, b3Transform_identity, shapeB->voxel, transformBtoA, contactDistance,
								B3_VOXEL_MAX_CONTACTS, contacts );
	}
	else
	{
		count = b3VoxelCollideConvex( shapeA->voxel, shapeB, transformBtoA, contactDistance, B3_VOXEL_MAX_CONTACTS, contacts,
									  &arena );
	}

	return b3Voxel_emitManifolds( world, contact, contacts, count, xfA, xfB, shapeA, shapeB );
}
