#include "joint_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "core.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/math_functions.h"

#include <stdlib.h>
#include <string.h>

#define B3_JOINT_LDL_MAX_ROWS 256
#define B3_JOINT_DIAG_EPS 1.0e-8f
#define B3_JOINT_CFM_TREE 5.0e-4f
#define B3_JOINT_BAUMGARTE 0.2f
#define B3_JOINT_BAUMGARTE_SNAP 1.0f
#define B3_JOINT_MAX_LINEAR_BIAS 2.0f
#define B3_JOINT_MAX_ANGULAR_BIAS 2.0f
#define B3_JOINT_MAX_LINEAR_BIAS_SNAP 80.0f
#define B3_JOINT_MAX_ANGULAR_BIAS_SNAP 20.0f
#define B3_JOINT_SNAP_LINEAR 0.25f
#define B3_JOINT_SNAP_ANGULAR 0.5f
#define B3_JOINT_LINEAR_SLOP 0.0005f
#define B3_JOINT_ANGULAR_SLOP 0.001f
#define B3_JOINT_MAX_SPEED 200.0f

typedef struct b3PairOrd
{
	float score;
	int index;
} b3PairOrd;

static int b3ComparePairOrd( const void* a, const void* b )
{
	const b3PairOrd* pa = (const b3PairOrd*)a;
	const b3PairOrd* pb = (const b3PairOrd*)b;
	if ( pa->score < pb->score )
	{
		return 1;
	}
	if ( pa->score > pb->score )
	{
		return -1;
	}
	return pa->index - pb->index;
}

static int b3Align16( int n )
{
	return ( n + 15 ) & ~15;
}

static void* b3CursorBump( char** cursor, int size )
{
	size = b3Align16( size );
	void* p = *cursor;
	*cursor += size;
	return p;
}

typedef struct b3JointRow
{
	int indexA;
	int indexB;
	b3Vec3 jLinA;
	b3Vec3 jAngA;
	b3Vec3 jLinB;
	b3Vec3 jAngB;
	float invMassA;
	float invMassB;
	b3Matrix3 invIA;
	b3Matrix3 invIB;
	float rhs;
	float posErr;
	float cfmRel;
	float* lambda;
} b3JointRow;

static int b3UfFind( int* parent, int i )
{
	while ( parent[i] != i )
	{
		parent[i] = parent[parent[i]];
		i = parent[i];
	}
	return i;
}

static void b3UfUnion( int* parent, int* rank, int a, int b )
{
	a = b3UfFind( parent, a );
	b = b3UfFind( parent, b );
	if ( a == b )
	{
		return;
	}

	if ( rank[a] < rank[b] )
	{
		parent[a] = b;
	}
	else if ( rank[b] < rank[a] )
	{
		parent[b] = a;
	}
	else
	{
		parent[b] = a;
		rank[a] += 1;
	}
}

static void b3FillRowMass( b3JointRow* row, const b3JointSim* base )
{
	row->invMassA = base->invMassA;
	row->invMassB = base->invMassB;
	row->invIA = base->invIA;
	row->invIB = base->invIB;
}

static b3JointRow* b3PushRow( b3JointRow* rows, int* count, int capacity )
{
	B3_ASSERT( *count < capacity );
	b3JointRow* row = rows + *count;
	*count += 1;
	memset( row, 0, sizeof( b3JointRow ) );
	return row;
}

static void b3EmitPointToPoint( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context, bool useBias,
								int indexA, int indexB, b3Vec3 rA, b3Vec3 rB, b3Vec3 deltaCenter, b3Vec3* linearImpulse )
{
	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : context->states + indexA;
	b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : context->states + indexB;

	b3Vec3 cdot = b3Sub( b3Add( stateB->linearVelocity, b3Cross( stateB->angularVelocity, rB ) ),
						 b3Add( stateA->linearVelocity, b3Cross( stateA->angularVelocity, rA ) ) );

	b3Vec3 bias = b3Vec3_zero;
	b3Vec3 separation = b3Vec3_zero;
	if ( useBias )
	{
		separation = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), b3Sub( rB, rA ) ), deltaCenter );
		float err2 = b3LengthSquared( separation );
		if ( err2 > B3_JOINT_LINEAR_SLOP * B3_JOINT_LINEAR_SLOP )
		{
			float beta = B3_JOINT_BAUMGARTE;
			float cap = B3_JOINT_MAX_LINEAR_BIAS;
			if ( err2 > B3_JOINT_SNAP_LINEAR * B3_JOINT_SNAP_LINEAR )
			{
				beta = B3_JOINT_BAUMGARTE_SNAP;
				cap = B3_JOINT_MAX_LINEAR_BIAS_SNAP;
			}
			b3Vec3 raw = b3MulSV( beta * context->inv_h, separation );
			float length = b3Length( raw );
			if ( length > cap && length > 0.0f )
			{
				raw = b3MulSV( cap / length, raw );
			}
			bias = raw;
		}
	}

	const b3Vec3 axes[3] = { b3Vec3_axisX, b3Vec3_axisY, b3Vec3_axisZ };
	float* lambda[3] = { &linearImpulse->x, &linearImpulse->y, &linearImpulse->z };

	for ( int k = 0; k < 3; ++k )
	{
		b3JointRow* row = b3PushRow( rows, count, capacity );
		row->indexA = indexA;
		row->indexB = indexB;
		row->jLinA = b3Neg( axes[k] );
		row->jAngA = b3Cross( axes[k], rA );
		row->jLinB = axes[k];
		row->jAngB = b3Cross( rB, axes[k] );
		b3FillRowMass( row, base );
		row->rhs = b3GetByIndex( cdot, k ) + b3GetByIndex( bias, k );
		row->posErr = b3GetByIndex( separation, k );
		row->lambda = lambda[k];
	}
}

static void b3EmitAngularEqual( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context, bool useBias,
								int indexA, int indexB, b3Vec3 angularError, b3Vec3* angularImpulse )
{
	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : context->states + indexA;
	b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : context->states + indexB;

	b3Vec3 cdot = b3Sub( stateB->angularVelocity, stateA->angularVelocity );
	b3Vec3 bias = b3Vec3_zero;
	if ( useBias )
	{
		float err2 = b3LengthSquared( angularError );
		if ( err2 > B3_JOINT_ANGULAR_SLOP * B3_JOINT_ANGULAR_SLOP )
		{
			float cap = err2 > B3_JOINT_SNAP_ANGULAR * B3_JOINT_SNAP_ANGULAR ? B3_JOINT_MAX_ANGULAR_BIAS_SNAP
																			: B3_JOINT_MAX_ANGULAR_BIAS;
			b3Vec3 raw = b3MulSV( B3_JOINT_BAUMGARTE * context->inv_h, angularError );
			float length = b3Length( raw );
			if ( length > cap && length > 0.0f )
			{
				raw = b3MulSV( cap / length, raw );
			}
			bias = raw;
		}
	}

	const b3Vec3 axes[3] = { b3Vec3_axisX, b3Vec3_axisY, b3Vec3_axisZ };
	float* lambda[3] = { &angularImpulse->x, &angularImpulse->y, &angularImpulse->z };

	for ( int k = 0; k < 3; ++k )
	{
		b3JointRow* row = b3PushRow( rows, count, capacity );
		row->indexA = indexA;
		row->indexB = indexB;
		row->jAngA = b3Neg( axes[k] );
		row->jAngB = axes[k];
		b3FillRowMass( row, base );
		row->rhs = b3GetByIndex( cdot, k ) + b3GetByIndex( bias, k );
		row->lambda = lambda[k];
	}
}

static void b3EmitAxisAngular( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context, bool useBias,
							   int indexA, int indexB, b3Vec3 axis, float c, float* lambda )
{
	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : context->states + indexA;
	b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : context->states + indexB;

	float cdot = b3Dot( b3Sub( stateB->angularVelocity, stateA->angularVelocity ), axis );
	float bias = 0.0f;
	if ( useBias && b3AbsFloat( c ) > B3_JOINT_ANGULAR_SLOP )
	{
		float cap = b3AbsFloat( c ) > B3_JOINT_SNAP_ANGULAR ? B3_JOINT_MAX_ANGULAR_BIAS_SNAP : B3_JOINT_MAX_ANGULAR_BIAS;
		bias = b3ClampFloat( B3_JOINT_BAUMGARTE * context->inv_h * c, -cap, cap );
	}

	b3JointRow* row = b3PushRow( rows, count, capacity );
	row->indexA = indexA;
	row->indexB = indexB;
	row->jAngA = b3Neg( axis );
	row->jAngB = axis;
	b3FillRowMass( row, base );
	row->rhs = cdot + bias;
	row->lambda = lambda;
}

static void b3EmitJoint( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context, bool useBias )
{
	switch ( base->type )
	{
		case b3_weldJoint:
		{
			b3WeldJoint* joint = &base->weldJoint;
			if ( joint->linearHertz > 0.0f && joint->angularHertz > 0.0f )
			{
				return;
			}

			b3BodyState dummyState = b3_identityBodyState;
			b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
			b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;
			b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
			b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );

			if ( joint->linearHertz == 0.0f )
			{
				b3EmitPointToPoint( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, rA, rB,
									joint->deltaCenter, &joint->linearImpulse );
			}

			if ( joint->angularHertz == 0.0f && base->fixedRotation == false )
			{
				b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
				b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
				if ( b3DotQuat( quatA, quatB ) < 0.0f )
				{
					quatB = b3NegateQuat( quatB );
				}

				b3Quat relQ = b3InvMulQuat( quatA, quatB );
				b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
				b3Vec3 angularError = b3Neg( b3RotateVector( quatA, deltaRotation ) );
				b3EmitAngularEqual( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, angularError,
									&joint->angularImpulse );
			}
		}
		break;

		case b3_sphericalJoint:
		{
			b3SphericalJoint* joint = &base->sphericalJoint;
			b3BodyState dummyState = b3_identityBodyState;
			b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
			b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;
			b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
			b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
			b3EmitPointToPoint( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, rA, rB,
								joint->deltaCenter, &joint->linearImpulse );
		}
		break;

		case b3_revoluteJoint:
		{
			b3RevoluteJoint* joint = &base->revoluteJoint;
			b3BodyState dummyState = b3_identityBodyState;
			b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
			b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;
			b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
			b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
			b3EmitPointToPoint( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, rA, rB,
								joint->deltaCenter, &joint->linearImpulse );

			if ( base->fixedRotation == false )
			{
				b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
				b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
				if ( b3DotQuat( quatA, quatB ) < 0.0f )
				{
					quatB = b3NegateQuat( quatB );
				}

				b3Quat relQ = b3InvMulQuat( quatA, quatB );
				b3Vec3 perpAxisX = b3MulSV(
					0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisX ), b3Cross( relQ.v, b3Vec3_axisX ) ) ) );
				b3Vec3 perpAxisY = b3MulSV(
					0.5f, b3RotateVector( quatA, b3Add( b3MulSV( relQ.s, b3Vec3_axisY ), b3Cross( relQ.v, b3Vec3_axisY ) ) ) );
				b3EmitAxisAngular( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, perpAxisX, relQ.v.x,
								   &joint->perpImpulse.x );
				b3EmitAxisAngular( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, perpAxisY, relQ.v.y,
								   &joint->perpImpulse.y );
			}
		}
		break;

		case b3_prismaticJoint:
		{
			b3PrismaticJoint* joint = &base->prismaticJoint;
			b3BodyState dummyState = b3_identityBodyState;
			b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
			b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;
			b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->frameA.p );
			b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->frameB.p );
			b3Vec3 d = b3Add( b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b3Sub( rB, rA ) );

			if ( base->fixedRotation == false )
			{
				b3Quat quatA = b3MulQuat( stateA->deltaRotation, joint->frameA.q );
				b3Quat quatB = b3MulQuat( stateB->deltaRotation, joint->frameB.q );
				b3Quat relQ = b3InvMulQuat( quatA, quatB );
				b3Vec3 deltaRotation = b3DeltaQuatToRotation( relQ, b3Quat_identity );
				b3Vec3 angularError = b3Neg( b3RotateVector( quatA, deltaRotation ) );
				b3EmitAngularEqual( rows, count, capacity, base, context, useBias, joint->indexA, joint->indexB, angularError,
									&joint->angularImpulse );
			}

			b3Vec3 perpY = b3RotateVector( stateA->deltaRotation, joint->perpAxisY );
			b3Vec3 perpZ = b3RotateVector( stateA->deltaRotation, joint->perpAxisZ );
			b3Vec3 rAd = b3Add( rA, d );
			b3Vec3 vRel = b3Sub( b3Add( stateB->linearVelocity, b3Cross( stateB->angularVelocity, rB ) ),
								 b3Add( stateA->linearVelocity, b3Cross( stateA->angularVelocity, rAd ) ) );

			b3Vec3 perps[2] = { perpY, perpZ };
			float cs[2] = { b3Dot( perpY, d ), b3Dot( perpZ, d ) };
			float* lambdas[2] = { &joint->perpImpulse.x, &joint->perpImpulse.y };
			for ( int k = 0; k < 2; ++k )
			{
				b3Vec3 n = perps[k];
				float cdot = b3Dot( n, vRel );
				float bias = 0.0f;
				if ( useBias && b3AbsFloat( cs[k] ) > B3_JOINT_LINEAR_SLOP )
				{
					float beta = B3_JOINT_BAUMGARTE;
					float cap = B3_JOINT_MAX_LINEAR_BIAS;
					if ( b3AbsFloat( cs[k] ) > B3_JOINT_SNAP_LINEAR )
					{
						beta = B3_JOINT_BAUMGARTE_SNAP;
						cap = B3_JOINT_MAX_LINEAR_BIAS_SNAP;
					}
					bias = b3ClampFloat( beta * context->inv_h * cs[k], -cap, cap );
				}
				b3JointRow* row = b3PushRow( rows, count, capacity );
				row->indexA = joint->indexA;
				row->indexB = joint->indexB;
				row->jLinA = b3Neg( n );
				row->jAngA = b3Neg( b3Cross( rAd, n ) );
				row->jLinB = n;
				row->jAngB = b3Cross( rB, n );
				b3FillRowMass( row, base );
				row->rhs = cdot + bias;
				row->posErr = cs[k];
				row->lambda = lambdas[k];
			}
		}
		break;

		case b3_distanceJoint:
		{
			b3DistanceJoint* joint = &base->distanceJoint;
			if ( joint->enableSpring && ( joint->minLength < joint->maxLength || joint->enableLimit == false ) )
			{
				return;
			}

			b3BodyState dummyState = b3_identityBodyState;
			b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
			b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;
			b3Vec3 rA = b3RotateVector( stateA->deltaRotation, joint->anchorA );
			b3Vec3 rB = b3RotateVector( stateB->deltaRotation, joint->anchorB );
			b3Vec3 ds = b3Add( b3Sub( stateB->deltaPosition, stateA->deltaPosition ), b3Sub( rB, rA ) );
			b3Vec3 separation = b3Add( joint->deltaCenter, ds );
			float length = b3Length( separation );
			if ( length < 1.0e-6f )
			{
				return;
			}

			b3Vec3 axis = b3Normalize( separation );
			b3Vec3 vr = b3Add( b3Sub( stateB->linearVelocity, stateA->linearVelocity ),
							   b3Sub( b3Cross( stateB->angularVelocity, rB ), b3Cross( stateA->angularVelocity, rA ) ) );
			float cdot = b3Dot( axis, vr );
			float c = length - joint->length;
			float bias = 0.0f;
			if ( useBias && b3AbsFloat( c ) > B3_JOINT_LINEAR_SLOP )
			{
				float beta = B3_JOINT_BAUMGARTE;
				float cap = B3_JOINT_MAX_LINEAR_BIAS;
				if ( b3AbsFloat( c ) > B3_JOINT_SNAP_LINEAR )
				{
					beta = B3_JOINT_BAUMGARTE_SNAP;
					cap = B3_JOINT_MAX_LINEAR_BIAS_SNAP;
				}
				bias = b3ClampFloat( beta * context->inv_h * c, -cap, cap );
			}

			b3JointRow* row = b3PushRow( rows, count, capacity );
			row->indexA = joint->indexA;
			row->indexB = joint->indexB;
			row->jLinA = b3Neg( axis );
			row->jAngA = b3Neg( b3Cross( rA, axis ) );
			row->jLinB = axis;
			row->jAngB = b3Cross( rB, axis );
			b3FillRowMass( row, base );
			row->rhs = cdot + bias;
			row->posErr = c;
			row->lambda = &joint->impulse;
		}
		break;

		default:
			break;
	}
}

static float b3RowDotMinv( const b3JointRow* a, const b3JointRow* b )
{
	float s = 0.0f;
	if ( a->indexA != B3_NULL_INDEX )
	{
		if ( a->indexA == b->indexA )
		{
			s += a->invMassA * b3Dot( a->jLinA, b->jLinA ) + b3Dot( a->jAngA, b3MulMV( a->invIA, b->jAngA ) );
		}
		if ( a->indexA == b->indexB )
		{
			s += a->invMassA * b3Dot( a->jLinA, b->jLinB ) + b3Dot( a->jAngA, b3MulMV( a->invIA, b->jAngB ) );
		}
	}

	if ( a->indexB != B3_NULL_INDEX )
	{
		if ( a->indexB == b->indexA )
		{
			s += a->invMassB * b3Dot( a->jLinB, b->jLinA ) + b3Dot( a->jAngB, b3MulMV( a->invIB, b->jAngA ) );
		}
		if ( a->indexB == b->indexB )
		{
			s += a->invMassB * b3Dot( a->jLinB, b->jLinB ) + b3Dot( a->jAngB, b3MulMV( a->invIB, b->jAngB ) );
		}
	}

	return s;
}

static void b3DenseLdl( float* a, int n )
{
	for ( int j = 0; j < n; ++j )
	{
		float d = a[j * n + j];
		for ( int k = 0; k < j; ++k )
		{
			float ljk = a[j * n + k];
			d -= ljk * ljk * a[k * n + k];
		}

		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}

		a[j * n + j] = d;
		float invD = 1.0f / d;
		for ( int i = j + 1; i < n; ++i )
		{
			float s = a[i * n + j];
			for ( int k = 0; k < j; ++k )
			{
				s -= a[i * n + k] * a[j * n + k] * a[k * n + k];
			}
			a[i * n + j] = s * invD;
		}
	}
}

static void b3DenseLdlSolve( const float* a, float* x, const float* b, int n )
{
	for ( int i = 0; i < n; ++i )
	{
		float s = b[i];
		for ( int k = 0; k < i; ++k )
		{
			s -= a[i * n + k] * x[k];
		}
		x[i] = s;
	}

	for ( int i = 0; i < n; ++i )
	{
		x[i] /= a[i * n + i];
	}

	for ( int i = n - 1; i >= 0; --i )
	{
		float s = x[i];
		for ( int k = i + 1; k < n; ++k )
		{
			s -= a[k * n + i] * x[k];
		}
		x[i] = s;
	}
}

static void b3JointMatVec( const b3JointRow* rows, const int* idx, int n, const float* p, float* ap, b3Vec3* dLin, b3Vec3* dAng )
{
	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* row = rows + idx[k];
		if ( row->indexA != B3_NULL_INDEX )
		{
			dLin[row->indexA] = b3Vec3_zero;
			dAng[row->indexA] = b3Vec3_zero;
		}
		if ( row->indexB != B3_NULL_INDEX )
		{
			dLin[row->indexB] = b3Vec3_zero;
			dAng[row->indexB] = b3Vec3_zero;
		}
	}

	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* row = rows + idx[k];
		float pk = p[k];
		if ( row->indexA != B3_NULL_INDEX )
		{
			dLin[row->indexA] = b3MulAdd( dLin[row->indexA], row->invMassA * pk, row->jLinA );
			dAng[row->indexA] = b3Add( dAng[row->indexA], b3MulMV( row->invIA, b3MulSV( pk, row->jAngA ) ) );
		}
		if ( row->indexB != B3_NULL_INDEX )
		{
			dLin[row->indexB] = b3MulAdd( dLin[row->indexB], row->invMassB * pk, row->jLinB );
			dAng[row->indexB] = b3Add( dAng[row->indexB], b3MulMV( row->invIB, b3MulSV( pk, row->jAngB ) ) );
		}
	}

	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* row = rows + idx[k];
		float a = 0.0f;
		if ( row->indexA != B3_NULL_INDEX )
		{
			a += b3Dot( row->jLinA, dLin[row->indexA] ) + b3Dot( row->jAngA, dAng[row->indexA] );
		}
		if ( row->indexB != B3_NULL_INDEX )
		{
			a += b3Dot( row->jLinB, dLin[row->indexB] ) + b3Dot( row->jAngB, dAng[row->indexB] );
		}
		ap[k] = a;
	}
}

static void b3ApplyDelta( b3StepContext* context, const b3JointRow* row, float impulse )
{
	if ( impulse == 0.0f || b3IsValidFloat( impulse ) == false )
	{
		return;
	}

	if ( row->indexA != B3_NULL_INDEX )
	{
		b3BodyState* state = context->states + row->indexA;
		if ( state->flags & b3_dynamicFlag )
		{
			state->linearVelocity = b3MulAdd( state->linearVelocity, row->invMassA * impulse, row->jLinA );
			state->angularVelocity = b3Add( state->angularVelocity, b3MulMV( row->invIA, b3MulSV( impulse, row->jAngA ) ) );
		}
	}

	if ( row->indexB != B3_NULL_INDEX )
	{
		b3BodyState* state = context->states + row->indexB;
		if ( state->flags & b3_dynamicFlag )
		{
			state->linearVelocity = b3MulAdd( state->linearVelocity, row->invMassB * impulse, row->jLinB );
			state->angularVelocity = b3Add( state->angularVelocity, b3MulMV( row->invIB, b3MulSV( impulse, row->jAngB ) ) );
		}
	}

	*row->lambda += impulse;
}

static bool b3JointSolutionValid( const float* x, int n )
{
	for ( int i = 0; i < n; ++i )
	{
		if ( b3IsValidFloat( x[i] ) == false )
		{
			return false;
		}
	}

	return true;
}

static void b3ApplyIsland( b3StepContext* context, const b3JointRow* rows, const int* idx, int n, const float* x,
						   b3Vec3* savedLin, b3Vec3* savedAng )
{
	if ( b3JointSolutionValid( x, n ) == false )
	{
		return;
	}

	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* row = rows + idx[i];
		if ( row->indexA != B3_NULL_INDEX )
		{
			const b3BodyState* state = context->states + row->indexA;
			savedLin[row->indexA] = state->linearVelocity;
			savedAng[row->indexA] = state->angularVelocity;
		}
		if ( row->indexB != B3_NULL_INDEX )
		{
			const b3BodyState* state = context->states + row->indexB;
			savedLin[row->indexB] = state->linearVelocity;
			savedAng[row->indexB] = state->angularVelocity;
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		b3ApplyDelta( context, rows + idx[i], x[i] );
	}

	bool exploded = false;
	float maxDelta = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* row = rows + idx[i];
		int ids[2] = { row->indexA, row->indexB };
		for ( int k = 0; k < 2; ++k )
		{
			if ( ids[k] == B3_NULL_INDEX )
			{
				continue;
			}

			const b3BodyState* state = context->states + ids[k];
			if ( b3IsValidVec3( state->linearVelocity ) == false || b3IsValidVec3( state->angularVelocity ) == false )
			{
				exploded = true;
				break;
			}

			maxDelta = b3MaxFloat( maxDelta, b3Length( b3Sub( state->linearVelocity, savedLin[ids[k]] ) ) );
			maxDelta = b3MaxFloat( maxDelta, b3Length( b3Sub( state->angularVelocity, savedAng[ids[k]] ) ) );
		}
		if ( exploded )
		{
			break;
		}
	}

	if ( exploded == false && maxDelta < B3_JOINT_MAX_SPEED )
	{
		return;
	}

	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* row = rows + idx[i];
		if ( row->indexA != B3_NULL_INDEX )
		{
			b3BodyState* state = context->states + row->indexA;
			state->linearVelocity = savedLin[row->indexA];
			state->angularVelocity = savedAng[row->indexA];
		}
		if ( row->indexB != B3_NULL_INDEX )
		{
			b3BodyState* state = context->states + row->indexB;
			state->linearVelocity = savedLin[row->indexB];
			state->angularVelocity = savedAng[row->indexB];
		}
		if ( row->lambda != NULL )
		{
			*row->lambda = 0.0f;
		}
	}

	if ( exploded || maxDelta <= 0.0f )
	{
		return;
	}

	float scale = B3_JOINT_MAX_SPEED / maxDelta;
	for ( int i = 0; i < n; ++i )
	{
		b3ApplyDelta( context, rows + idx[i], scale * x[i] );
	}
}

static void b3SolveIslandDense( b3StepContext* context, const b3JointRow* rows, const int* idx, int n, float* a, float* b,
								float* x, float* scale, b3Vec3* savedLin, b3Vec3* savedAng )
{
	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* ri = rows + idx[i];
		b[i] = -ri->rhs;
		for ( int j = 0; j <= i; ++j )
		{
			float s = b3RowDotMinv( ri, rows + idx[j] );
			a[i * n + j] = s;
			a[j * n + i] = s;
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		float d = a[i * n + i];
		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}
		scale[i] = 1.0f / sqrtf( d );
	}

	for ( int i = 0; i < n; ++i )
	{
		b[i] *= scale[i];
		for ( int j = 0; j <= i; ++j )
		{
			float s = a[i * n + j] * scale[i] * scale[j];
			if ( i == j )
			{
				s += B3_JOINT_DIAG_EPS + B3_JOINT_CFM_TREE;
			}
			a[i * n + j] = s;
			a[j * n + i] = s;
		}
	}

	b3DenseLdl( a, n );
	b3DenseLdlSolve( a, x, b, n );

	for ( int i = 0; i < n; ++i )
	{
		x[i] *= scale[i];
	}

	b3ApplyIsland( context, rows, idx, n, x, savedLin, savedAng );
}

static void b3JointPcgSolve( const b3JointRow* rows, const int* idx, int n, float* x, const float* b, float* r, float* z,
							 float* p, float* ap, const float* invDiag, const float* scale, b3Vec3* dLin, b3Vec3* dAng )
{
	const float cfm = B3_JOINT_DIAG_EPS + B3_JOINT_CFM_TREE;
	for ( int i = 0; i < n; ++i )
	{
		x[i] = 0.0f;
		r[i] = b[i];
		z[i] = invDiag[i] * r[i];
		p[i] = z[i];
	}

	float rz = 0.0f;
	float b2 = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		rz += r[i] * z[i];
		b2 += b[i] * b[i];
	}

	if ( b2 < 1.0e-20f )
	{
		return;
	}

	int maxIter = n + 32;
	if ( maxIter > 512 )
	{
		maxIter = 512;
	}

	const float tol = 1.0e-8f * b2;
	for ( int iter = 0; iter < maxIter; ++iter )
	{
		for ( int i = 0; i < n; ++i )
		{
			z[i] = scale[i] * p[i];
		}
		b3JointMatVec( rows, idx, n, z, ap, dLin, dAng );
		for ( int i = 0; i < n; ++i )
		{
			ap[i] = scale[i] * ap[i] + cfm * p[i];
		}

		float pap = 0.0f;
		for ( int i = 0; i < n; ++i )
		{
			pap += p[i] * ap[i];
		}
		if ( pap <= 1.0e-20f )
		{
			break;
		}

		float alpha = rz / pap;
		float r2 = 0.0f;
		for ( int i = 0; i < n; ++i )
		{
			x[i] += alpha * p[i];
			r[i] -= alpha * ap[i];
			r2 += r[i] * r[i];
		}

		if ( r2 <= tol )
		{
			break;
		}

		float rzNew = 0.0f;
		for ( int i = 0; i < n; ++i )
		{
			z[i] = invDiag[i] * r[i];
			rzNew += r[i] * z[i];
		}

		float beta = rzNew / rz;
		rz = rzNew;
		for ( int i = 0; i < n; ++i )
		{
			p[i] = z[i] + beta * p[i];
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		x[i] *= scale[i];
	}
}

static void b3SolveIslandPcg( b3StepContext* context, const b3JointRow* rows, const int* idx, int n, float* x, float* b, float* r,
							  float* z, float* p, float* ap, float* invDiag, float* scale, b3Vec3* dLin, b3Vec3* dAng )
{
	const float cfm = B3_JOINT_DIAG_EPS + B3_JOINT_CFM_TREE;
	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* row = rows + idx[i];
		float d = b3RowDotMinv( row, row );
		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}
		scale[i] = 1.0f / sqrtf( d );
		invDiag[i] = 1.0f / ( 1.0f + cfm );
		b[i] = -row->rhs * scale[i];
	}

	b3JointPcgSolve( rows, idx, n, x, b, r, z, p, ap, invDiag, scale, dLin, dAng );
	b3ApplyIsland( context, rows, idx, n, x, dLin, dAng );
}

void b3SolveJoints_Direct( b3StepContext* context, bool useBias )
{
	b3TracyCZoneNC( joint_direct, "JointDirect", b3_colorLemonChiffon, true );

	b3World* world = context->world;
	b3ConstraintGraph* graph = context->graph;
	int jointCount = 0;
	for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		jointCount += graph->colors[colorIndex].jointSims.count;
	}

	if ( jointCount == 0 )
	{
		b3TracyCZoneEnd( joint_direct );
		return;
	}

	int bodyCount = world->solverSets.data[b3_awakeSet].bodyStates.count;
	if ( bodyCount == 0 )
	{
		b3TracyCZoneEnd( joint_direct );
		return;
	}

	int rowCapacity = 6 * jointCount;
	b3Stack* stack = &world->stack;

	int phase1 = 0;
	phase1 += b3Align16( rowCapacity * (int)sizeof( b3JointRow ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( b3PairOrd ) );
	phase1 += b3Align16( rowCapacity * (int)sizeof( int ) );

	char* blob1 = (char*)b3StackAlloc( stack, phase1, "joint direct 1" );
	char* cursor = blob1;
	b3JointRow* rows = (b3JointRow*)b3CursorBump( &cursor, rowCapacity * (int)sizeof( b3JointRow ) );
	int* parent = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* rank = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* pinned = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* islandHead = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* pairA = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );
	int* pairB = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );
	int* pairBegin = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );
	int* pairEnd = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );
	b3PairOrd* pairOrd = (b3PairOrd*)b3CursorBump( &cursor, jointCount * (int)sizeof( b3PairOrd ) );
	int* rowCycle = (int*)b3CursorBump( &cursor, rowCapacity * (int)sizeof( int ) );

	for ( int i = 0; i < bodyCount; ++i )
	{
		parent[i] = i;
		rank[i] = 0;
		pinned[i] = 0;
		islandHead[i] = B3_NULL_INDEX;
	}

	int rowCount = 0;
	int pairCount = 0;
	for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b3GraphColor* color = graph->colors + colorIndex;
		int count = color->jointSims.count;
		b3JointSim* joints = color->jointSims.data;
		for ( int i = 0; i < count; ++i )
		{
			int begin = rowCount;
			b3EmitJoint( rows, &rowCount, rowCapacity, joints + i, context, useBias );
			if ( rowCount == begin )
			{
				continue;
			}

			int indexA = rows[begin].indexA;
			int indexB = rows[begin].indexB;
			pairA[pairCount] = indexA;
			pairB[pairCount] = indexB;
			pairBegin[pairCount] = begin;
			pairEnd[pairCount] = rowCount;
			pairCount += 1;
		}
	}

	if ( rowCount == 0 )
	{
		b3StackFree( stack, blob1 );
		b3TracyCZoneEnd( joint_direct );
		return;
	}

	for ( int i = 0; i < rowCount; ++i )
	{
		rowCycle[i] = 0;
		if ( rows[i].lambda != NULL )
		{
			*rows[i].lambda = 0.0f;
		}
	}

	// Stretched joints first so a yanked middle link stays in the spanning
	// tree instead of being dropped as a redundant loop.
	for ( int p = 0; p < pairCount; ++p )
	{
		float score = 0.0f;
		for ( int r = pairBegin[p]; r < pairEnd[p]; ++r )
		{
			score = b3MaxFloat( score, b3AbsFloat( rows[r].rhs ) );
			score = b3MaxFloat( score, b3AbsFloat( rows[r].posErr ) * context->inv_h );
		}
		pairOrd[p].score = score;
		pairOrd[p].index = p;
	}
	qsort( pairOrd, (size_t)pairCount, sizeof( b3PairOrd ), b3ComparePairOrd );

	for ( int o = 0; o < pairCount; ++o )
	{
		int p = pairOrd[o].index;
		int indexA = pairA[p];
		int indexB = pairB[p];
		int cycle = 0;

		if ( indexA == B3_NULL_INDEX && indexB == B3_NULL_INDEX )
		{
			cycle = 1;
		}
		else if ( indexA == B3_NULL_INDEX || indexB == B3_NULL_INDEX )
		{
			int dynamicIndex = indexA != B3_NULL_INDEX ? indexA : indexB;
			int root = b3UfFind( parent, dynamicIndex );
			if ( pinned[root] )
			{
				cycle = 1;
			}
			else
			{
				pinned[root] = 1;
			}
		}
		else
		{
			int rootA = b3UfFind( parent, indexA );
			int rootB = b3UfFind( parent, indexB );
			if ( rootA == rootB || ( pinned[rootA] && pinned[rootB] ) )
			{
				cycle = 1;
			}
			else
			{
				int pin = pinned[rootA] | pinned[rootB];
				b3UfUnion( parent, rank, indexA, indexB );
				pinned[b3UfFind( parent, indexA )] = pin;
			}
		}

		if ( cycle )
		{
			for ( int r = pairBegin[p]; r < pairEnd[p]; ++r )
			{
				rowCycle[r] = 1;
			}
		}
	}

	int maxN = 0;
	for ( int i = 0; i < bodyCount; ++i )
	{
		rank[i] = 0;
	}

	int phase2 = b3Align16( rowCount * (int)sizeof( int ) );
	char* blob2 = (char*)b3StackAlloc( stack, phase2 + 16, "joint direct 2a" );
	cursor = blob2;
	int* rowNext = (int*)b3CursorBump( &cursor, rowCount * (int)sizeof( int ) );

	for ( int i = 0; i < rowCount; ++i )
	{
		if ( rowCycle[i] )
		{
			continue;
		}

		int indexA = rows[i].indexA;
		int indexB = rows[i].indexB;
		int key = indexA != B3_NULL_INDEX ? indexA : indexB;
		B3_ASSERT( key != B3_NULL_INDEX && key < bodyCount );
		int root = b3UfFind( parent, key );
		rowNext[i] = islandHead[root];
		islandHead[root] = i;
		rank[root] += 1;
		if ( rank[root] > maxN )
		{
			maxN = rank[root];
		}
	}

	if ( maxN < 6 )
	{
		maxN = 6;
	}

	int denseN = maxN < B3_JOINT_LDL_MAX_ROWS ? maxN : B3_JOINT_LDL_MAX_ROWS;
	if ( denseN < 1 )
	{
		denseN = 1;
	}

	int phase3 = 0;
	phase3 += b3Align16( maxN * (int)sizeof( int ) );
	phase3 += b3Align16( denseN * denseN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( bodyCount * (int)sizeof( b3Vec3 ) );
	phase3 += b3Align16( bodyCount * (int)sizeof( b3Vec3 ) );

	char* blob3 = (char*)b3StackAlloc( stack, phase3, "joint direct 3" );
	cursor = blob3;
	int* idx = (int*)b3CursorBump( &cursor, maxN * (int)sizeof( int ) );
	float* workA = (float*)b3CursorBump( &cursor, denseN * denseN * (int)sizeof( float ) );
	float* workB = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workX = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workR = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workZ = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workP = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workAp = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workInvDiag = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workScale = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	b3Vec3* dLin = (b3Vec3*)b3CursorBump( &cursor, bodyCount * (int)sizeof( b3Vec3 ) );
	b3Vec3* dAng = (b3Vec3*)b3CursorBump( &cursor, bodyCount * (int)sizeof( b3Vec3 ) );

	for ( int root = 0; root < bodyCount; ++root )
	{
		int head = islandHead[root];
		if ( head == B3_NULL_INDEX )
		{
			continue;
		}

		int n = 0;
		for ( int k = head; k != B3_NULL_INDEX; k = rowNext[k] )
		{
			idx[n++] = k;
		}

		if ( n <= B3_JOINT_LDL_MAX_ROWS )
		{
			b3SolveIslandDense( context, rows, idx, n, workA, workB, workX, workScale, dLin, dAng );
		}
		else
		{
			b3SolveIslandPcg( context, rows, idx, n, workX, workB, workR, workZ, workP, workAp, workInvDiag, workScale, dLin,
							  dAng );
		}
	}

	b3StackFree( stack, blob3 );
	b3StackFree( stack, blob2 );
	b3StackFree( stack, blob1 );

	b3TracyCZoneEnd( joint_direct );
}
