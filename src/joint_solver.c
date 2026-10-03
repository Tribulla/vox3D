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

#define B3_JOINT_DIAG_EPS 1.0e-8f
#define B3_JOINT_CFM_TREE 1.0e-6f
#define B3_JOINT_POSITION_CFM 1.0e-2f
#define B3_JOINT_BAUMGARTE 0.2f
#define B3_JOINT_MAX_LINEAR_BIAS 10.0f
#define B3_JOINT_MAX_ANGULAR_BIAS 10.0f
#define B3_JOINT_LINEAR_SLOP 0.0005f
#define B3_JOINT_ANGULAR_SLOP 0.001f

typedef struct b3PairOrd
{
	int key;
	int index;
} b3PairOrd;

static int b3ComparePairOrd( const void* a, const void* b )
{
	const b3PairOrd* pa = (const b3PairOrd*)a;
	const b3PairOrd* pb = (const b3PairOrd*)b;
	if ( pa->key < pb->key )
	{
		return -1;
	}
	if ( pa->key > pb->key )
	{
		return 1;
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
	b3Vec3 massAngA;
	b3Vec3 massAngB;
	float rhs;
	float bias;
	float* lambda;
} b3JointRow;

typedef struct b3IslandRow
{
	int localA;
	int localB;
	float invMassA;
	float invMassB;
	b3Vec3 jLinA;
	b3Vec3 jAngA;
	b3Vec3 jLinB;
	b3Vec3 jAngB;
} b3IslandRow;

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

static void b3FillRowMass( b3JointRow* row, const b3JointSim* base, const b3Matrix3* invInertias )
{
	row->invMassA = base->invMassA;
	row->invMassB = base->invMassB;
	row->massAngA = row->indexA == B3_NULL_INDEX ? b3Vec3_zero : b3MulMV( invInertias[row->indexA], row->jAngA );
	row->massAngB = row->indexB == B3_NULL_INDEX ? b3Vec3_zero : b3MulMV( invInertias[row->indexB], row->jAngB );
}

static b3JointRow* b3PushRow( b3JointRow* rows, int* count, int capacity )
{
	B3_UNUSED( capacity );
	B3_ASSERT( *count < capacity );
	b3JointRow* row = rows + *count;
	*count += 1;
	memset( row, 0, sizeof( b3JointRow ) );
	return row;
}

static b3Vec3 b3JointVectorBias( b3Vec3 error, float slop, float maxSpeed, float inv_h )
{
	float length = b3Length( error );
	if ( length <= slop )
	{
		return b3Vec3_zero;
	}

	// Correct only the error outside the tolerance. Switching the entire
	// correction on at the tolerance injects a finite velocity into resting joints.
	float speed = b3MinFloat( B3_JOINT_BAUMGARTE * inv_h * ( length - slop ), maxSpeed );
	return b3MulSV( speed / length, error );
}

static float b3JointScalarBias( float error, float slop, float maxSpeed, float inv_h )
{
	float correction = b3MaxFloat( b3AbsFloat( error ) - slop, 0.0f );
	float speed = b3MinFloat( B3_JOINT_BAUMGARTE * inv_h * correction, maxSpeed );
	return error < 0.0f ? -speed : speed;
}

static void b3EmitPointToPoint( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context,
								const b3Matrix3* invInertias, bool useBias,
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
		bias = b3JointVectorBias( separation, B3_JOINT_LINEAR_SLOP, B3_JOINT_MAX_LINEAR_BIAS, context->inv_h );
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
		b3FillRowMass( row, base, invInertias );
		row->bias = b3GetByIndex( bias, k );
		row->rhs = b3GetByIndex( cdot, k ) + row->bias;
		row->lambda = lambda[k];
	}
}

static void b3EmitAngularEqual( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context,
								const b3Matrix3* invInertias, bool useBias,
								int indexA, int indexB, b3Vec3 angularError, b3Vec3* angularImpulse )
{
	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : context->states + indexA;
	b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : context->states + indexB;

	b3Vec3 cdot = b3Sub( stateB->angularVelocity, stateA->angularVelocity );
	b3Vec3 bias = b3Vec3_zero;
	if ( useBias )
	{
		bias = b3JointVectorBias( angularError, B3_JOINT_ANGULAR_SLOP, B3_JOINT_MAX_ANGULAR_BIAS, context->inv_h );
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
		b3FillRowMass( row, base, invInertias );
		row->bias = b3GetByIndex( bias, k );
		row->rhs = b3GetByIndex( cdot, k ) + row->bias;
		row->lambda = lambda[k];
	}
}

static void b3EmitAxisAngular( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context,
							   const b3Matrix3* invInertias, bool useBias,
							   int indexA, int indexB, b3Vec3 axis, float c, float* lambda )
{
	b3BodyState dummyState = b3_identityBodyState;
	b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : context->states + indexA;
	b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : context->states + indexB;

	float cdot = b3Dot( b3Sub( stateB->angularVelocity, stateA->angularVelocity ), axis );
	float bias = 0.0f;
	if ( useBias )
	{
		bias = b3JointScalarBias( c, B3_JOINT_ANGULAR_SLOP, B3_JOINT_MAX_ANGULAR_BIAS, context->inv_h );
	}

	b3JointRow* row = b3PushRow( rows, count, capacity );
	row->indexA = indexA;
	row->indexB = indexB;
	row->jAngA = b3Neg( axis );
	row->jAngB = axis;
	b3FillRowMass( row, base, invInertias );
	row->bias = bias;
	row->rhs = cdot + bias;
	row->lambda = lambda;
}

static void b3EmitJoint( b3JointRow* rows, int* count, int capacity, b3JointSim* base, b3StepContext* context,
						 const b3Matrix3* invInertias, bool useBias )
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
				b3EmitPointToPoint( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, rA, rB,
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
				b3EmitAngularEqual( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, angularError,
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
			b3EmitPointToPoint( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, rA, rB,
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
			b3EmitPointToPoint( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, rA, rB,
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
				b3EmitAxisAngular( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, perpAxisX, relQ.v.x,
								   &joint->perpImpulse.x );
				b3EmitAxisAngular( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, perpAxisY, relQ.v.y,
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
				b3EmitAngularEqual( rows, count, capacity, base, context, invInertias, useBias, joint->indexA, joint->indexB, angularError,
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
				if ( useBias )
				{
					bias = b3JointScalarBias( cs[k], B3_JOINT_LINEAR_SLOP, B3_JOINT_MAX_LINEAR_BIAS, context->inv_h );
				}
				b3JointRow* row = b3PushRow( rows, count, capacity );
				row->indexA = joint->indexA;
				row->indexB = joint->indexB;
				row->jLinA = b3Neg( n );
				row->jAngA = b3Neg( b3Cross( rAd, n ) );
				row->jLinB = n;
				row->jAngB = b3Cross( rB, n );
				b3FillRowMass( row, base, invInertias );
				row->bias = bias;
				row->rhs = cdot + bias;
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
			if ( useBias )
			{
				bias = b3JointScalarBias( c, B3_JOINT_LINEAR_SLOP, B3_JOINT_MAX_LINEAR_BIAS, context->inv_h );
			}

			b3JointRow* row = b3PushRow( rows, count, capacity );
			row->indexA = joint->indexA;
			row->indexB = joint->indexB;
			row->jLinA = b3Neg( axis );
			row->jAngA = b3Neg( b3Cross( rA, axis ) );
			row->jLinB = axis;
			row->jAngB = b3Cross( rB, axis );
			b3FillRowMass( row, base, invInertias );
			row->bias = bias;
			row->rhs = cdot + bias;
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
			s += a->invMassA * b3Dot( a->jLinA, b->jLinA ) + b3Dot( a->jAngA, b->massAngA );
		}
		if ( a->indexA == b->indexB )
		{
			s += a->invMassA * b3Dot( a->jLinA, b->jLinB ) + b3Dot( a->jAngA, b->massAngB );
		}
	}

	if ( a->indexB != B3_NULL_INDEX )
	{
		if ( a->indexB == b->indexA )
		{
			s += a->invMassB * b3Dot( a->jLinB, b->jLinA ) + b3Dot( a->jAngB, b->massAngA );
		}
		if ( a->indexB == b->indexB )
		{
			s += a->invMassB * b3Dot( a->jLinB, b->jLinB ) + b3Dot( a->jAngB, b->massAngB );
		}
	}

	return s;
}

static void b3DenseLdl( double* a, const int* first, int n )
{
	double v[B3_JOINT_LDL_MAX_ROWS];
	for ( int j = 0; j < n; ++j )
	{
		double d = a[j * n + j];
		for ( int k = first[j]; k < j; ++k )
		{
			double ljk = a[j * n + k];
			d -= ljk * ljk * a[k * n + k];
		}

		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}

		a[j * n + j] = d;
		double invD = 1.0 / d;

		for ( int k = first[j]; k < j; ++k )
		{
			v[k] = a[j * n + k] * a[k * n + k];
		}

		for ( int i = j + 1; i < n; ++i )
		{
			if ( first[i] > j )
			{
				continue;
			}
			double s = a[i * n + j];
			const double* rowI = a + i * n;
			for ( int k = b3MaxInt( first[i], first[j] ); k < j; ++k )
			{
				s -= rowI[k] * v[k];
			}
			a[i * n + j] = s * invD;
		}
	}
}

static void b3DenseLdlSolve( const double* a, const int* first, float* x, const float* b, int n )
{
	double y[B3_JOINT_LDL_MAX_ROWS];
	for ( int i = 0; i < n; ++i )
	{
		double s = b[i];
		for ( int k = first[i]; k < i; ++k )
		{
			s -= a[i * n + k] * y[k];
		}
		y[i] = s;
	}

	for ( int i = 0; i < n; ++i )
	{
		y[i] /= a[i * n + i];
	}

	for ( int i = n - 1; i >= 0; --i )
	{
		for ( int k = first[i]; k < i; ++k )
		{
			y[k] -= a[i * n + k] * y[i];
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		x[i] = (float)y[i];
	}
}

static void b3JointMatVec( const b3IslandRow* __restrict pcgRows, int n, const float* __restrict p, float* __restrict ap,
						   const b3Matrix3* __restrict islandInvI, int islandBodyCount,
						   b3Vec3* __restrict dLin, b3Vec3* __restrict dAng )
{
	memset( dLin, 0, islandBodyCount * sizeof( b3Vec3 ) );
	memset( dAng, 0, islandBodyCount * sizeof( b3Vec3 ) );

	for ( int k = 0; k < n; ++k )
	{
		const b3IslandRow* row = pcgRows + k;
		float pk = p[k];
		if ( pk == 0.0f )
		{
			continue;
		}
		int la = row->localA;
		int lb = row->localB;
		if ( la != B3_NULL_INDEX )
		{
			dLin[la] = b3MulAdd( dLin[la], row->invMassA * pk, row->jLinA );
			dAng[la] = b3MulAdd( dAng[la], pk, row->jAngA );
		}
		if ( lb != B3_NULL_INDEX )
		{
			dLin[lb] = b3MulAdd( dLin[lb], row->invMassB * pk, row->jLinB );
			dAng[lb] = b3MulAdd( dAng[lb], pk, row->jAngB );
		}
	}

	for ( int i = 0; i < islandBodyCount; ++i )
	{
		dAng[i] = b3MulMV( islandInvI[i], dAng[i] );
	}

	for ( int k = 0; k < n; ++k )
	{
		const b3IslandRow* row = pcgRows + k;
		int la = row->localA;
		int lb = row->localB;
		float a = 0.0f;
		if ( la != B3_NULL_INDEX )
		{
			a += b3Dot( row->jLinA, dLin[la] ) + b3Dot( row->jAngA, dAng[la] );
		}
		if ( lb != B3_NULL_INDEX )
		{
			a += b3Dot( row->jLinB, dLin[lb] ) + b3Dot( row->jAngB, dAng[lb] );
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
			state->angularVelocity = b3Add( state->angularVelocity, b3MulSV( impulse, row->massAngA ) );
		}
	}

	if ( row->indexB != B3_NULL_INDEX )
	{
		b3BodyState* state = context->states + row->indexB;
		if ( state->flags & b3_dynamicFlag )
		{
			state->linearVelocity = b3MulAdd( state->linearVelocity, row->invMassB * impulse, row->jLinB );
			state->angularVelocity = b3Add( state->angularVelocity, b3MulSV( impulse, row->massAngB ) );
		}
	}

	if ( row->lambda != NULL )
	{
		*row->lambda += impulse;
	}
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
						   const int* islandBodies, int islandBodyCount,
						   b3Vec3* savedLin, b3Vec3* savedAng, bool positionProjection )
{
	if ( b3JointSolutionValid( x, n ) == false )
	{
		return;
	}

	for ( int i = 0; i < islandBodyCount; ++i )
	{
		int b = islandBodies[i];
		b3BodyState* state = context->states + b;
		savedLin[i] = state->linearVelocity;
		savedAng[i] = state->angularVelocity;
		if ( positionProjection )
		{
			state->linearVelocity = b3Vec3_zero;
			state->angularVelocity = b3Vec3_zero;
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		b3ApplyDelta( context, rows + idx[i], x[i] );
	}

	// Geometric corrections share a trust region. Physical speed limits are
	// handled after this solve so their changes can be projected through J.
	if ( positionProjection )
	{
		float maxCorrectionSpeed = 0.2f * context->inv_h;
		float maxCorrectionSpeedSquared = maxCorrectionSpeed * maxCorrectionSpeed;
		float motionScale = 1.0f;
		for ( int i = 0; i < islandBodyCount; ++i )
		{
			b3BodyState* state = context->states + islandBodies[i];
			state->linearVelocity.x = ( state->flags & b3_lockLinearX ) ? 0.0f : state->linearVelocity.x;
			state->linearVelocity.y = ( state->flags & b3_lockLinearY ) ? 0.0f : state->linearVelocity.y;
			state->linearVelocity.z = ( state->flags & b3_lockLinearZ ) ? 0.0f : state->linearVelocity.z;
			state->angularVelocity.x = ( state->flags & b3_lockAngularX ) ? 0.0f : state->angularVelocity.x;
			state->angularVelocity.y = ( state->flags & b3_lockAngularY ) ? 0.0f : state->angularVelocity.y;
			state->angularVelocity.z = ( state->flags & b3_lockAngularZ ) ? 0.0f : state->angularVelocity.z;
			if ( b3LengthSquared( state->linearVelocity ) > maxCorrectionSpeedSquared )
			{
				motionScale = b3MinFloat( motionScale, maxCorrectionSpeed / b3Length( state->linearVelocity ) );
			}
			if ( b3LengthSquared( state->angularVelocity ) > maxCorrectionSpeedSquared )
			{
				motionScale = b3MinFloat( motionScale, maxCorrectionSpeed / b3Length( state->angularVelocity ) );
			}
		}
		if ( motionScale < 1.0f )
		{
			for ( int i = 0; i < islandBodyCount; ++i )
			{
				b3BodyState* state = context->states + islandBodies[i];
				state->linearVelocity = b3MulSV( motionScale, state->linearVelocity );
				state->angularVelocity = b3MulSV( motionScale, state->angularVelocity );
			}
		}
	}

	bool exploded = false;
	for ( int i = 0; i < islandBodyCount; ++i )
	{
		int b = islandBodies[i];
		const b3BodyState* state = context->states + b;
		if ( b3IsValidVec3( state->linearVelocity ) == false || b3IsValidVec3( state->angularVelocity ) == false )
		{
			exploded = true;
			break;
		}
	}

	if ( exploded )
	{
		for ( int i = 0; i < islandBodyCount; ++i )
		{
			int b = islandBodies[i];
			b3BodyState* state = context->states + b;
			state->linearVelocity = savedLin[i];
			state->angularVelocity = savedAng[i];
		}

		for ( int i = 0; i < n; ++i )
		{
			const b3JointRow* row = rows + idx[i];
			if ( row->lambda != NULL )
			{
				*row->lambda = 0.0f;
			}
		}
	}
	else if ( positionProjection )
	{
		for ( int i = 0; i < islandBodyCount; ++i )
		{
			b3BodyState* state = context->states + islandBodies[i];
			state->deltaPosition = b3MulAdd( state->deltaPosition, context->h, state->linearVelocity );
			state->deltaRotation = b3IntegrateRotation( state->deltaRotation, b3MulSV( context->h, state->angularVelocity ) );
			state->linearVelocity = savedLin[i];
			state->angularVelocity = savedAng[i];
		}
	}
}

static float b3JointRowVelocity( const b3StepContext* context, const b3JointRow* row )
{
	float velocity = 0.0f;
	if ( row->indexA != B3_NULL_INDEX )
	{
		const b3BodyState* state = context->states + row->indexA;
		velocity += b3Dot( row->jLinA, state->linearVelocity ) + b3Dot( row->jAngA, state->angularVelocity );
	}
	if ( row->indexB != B3_NULL_INDEX )
	{
		const b3BodyState* state = context->states + row->indexB;
		velocity += b3Dot( row->jLinB, state->linearVelocity ) + b3Dot( row->jAngB, state->angularVelocity );
	}
	return velocity;
}

static void b3SaveJointSpeeds( b3StepContext* context, const int* bodies, int count, b3Vec3* linear, b3Vec3* angular )
{
	for ( int i = 0; i < count; ++i )
	{
		linear[i] = context->states[bodies[i]].linearVelocity;
		angular[i] = context->states[bodies[i]].angularVelocity;
	}
}

static bool b3ValidateJointSpeeds( b3StepContext* context, const b3JointRow* rows, const int* idx, int n,
								 const int* bodies, int count, const b3Vec3* linear, const b3Vec3* angular )
{
	for ( int i = 0; i < count; ++i )
	{
		const b3BodyState* state = context->states + bodies[i];
		if ( b3IsValidVec3( state->linearVelocity ) == false || b3IsValidVec3( state->angularVelocity ) == false )
		{
			for ( int j = 0; j < count; ++j )
			{
				context->states[bodies[j]].linearVelocity = linear[j];
				context->states[bodies[j]].angularVelocity = angular[j];
			}
			for ( int j = 0; j < n; ++j )
			{
				float* impulse = rows[idx[j]].lambda;
				if ( impulse != NULL )
				{
					*impulse = 0.0f;
				}
			}
			return false;
		}
	}
	return true;
}

static float b3LimitJointSpeeds( b3StepContext* context, const int* bodies, int count, bool clip )
{
	float linearLimit = context->maxLinearVelocity;
	float angularLimit = B3_MAX_ROTATION * context->inv_dt;
	float scale = 1.0f;
	for ( int i = 0; i < count; ++i )
	{
		b3BodyState* state = context->states + bodies[i];
		float v2 = b3LengthSquared( state->linearVelocity );
		float w2 = b3LengthSquared( state->angularVelocity );
		if ( v2 > linearLimit * linearLimit )
		{
			float s = linearLimit / sqrtf( v2 );
			scale = b3MinFloat( scale, s );
			if ( clip )
			{
				state->linearVelocity = b3MulSV( s, state->linearVelocity );
				state->flags |= b3_isSpeedCapped;
			}
		}
		if ( w2 > angularLimit * angularLimit && ( state->flags & b3_allowFastRotation ) == 0 )
		{
			float s = angularLimit / sqrtf( w2 );
			scale = b3MinFloat( scale, s );
			if ( clip )
			{
				state->angularVelocity = b3MulSV( s, state->angularVelocity );
				state->flags |= b3_isSpeedCapped;
			}
		}
	}
	return scale;
}

static void b3ScaleJointSpeeds( b3StepContext* context, const int* bodies, int count )
{
	float scale = b3LimitJointSpeeds( context, bodies, count, false );
	if ( scale < 1.0f )
	{
		for ( int i = 0; i < count; ++i )
		{
			b3BodyState* state = context->states + bodies[i];
			state->linearVelocity = b3MulSV( scale, state->linearVelocity );
			state->angularVelocity = b3MulSV( scale, state->angularVelocity );
			state->flags |= b3_isSpeedCapped;
		}
	}
}

static void b3SolveIslandDense( b3StepContext* context, const b3JointRow* rows, int* idx, int n, double* a, float* b,
								float* x, float* scale,
								int* islandBodies, int* bodyTag,
								b3Vec3* savedLin, b3Vec3* savedAng, bool positionProjection )
{
	if ( n > 128 )
	{
		b3PairOrd order[B3_JOINT_LDL_MAX_ROWS];
		for ( int i = 0; i < n; ++i )
		{
			const b3JointRow* row = rows + idx[i];
			int key = row->indexA == B3_NULL_INDEX ? row->indexB : row->indexA;
			if ( row->indexB != B3_NULL_INDEX )
			{
				key = b3MinInt( key, row->indexB );
			}
			order[i] = (b3PairOrd){ key, idx[i] };
		}
		qsort( order, n, sizeof( b3PairOrd ), b3ComparePairOrd );
		for ( int i = 0; i < n; ++i )
		{
			idx[i] = order[i].index;
		}
	}
	int first[B3_JOINT_LDL_MAX_ROWS];
	int islandBodyCount = 0;
	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* row = rows + idx[k];
		if ( row->indexA != B3_NULL_INDEX && bodyTag[row->indexA] == -1 )
		{
			bodyTag[row->indexA] = islandBodyCount;
			islandBodies[islandBodyCount++] = row->indexA;
		}
		if ( row->indexB != B3_NULL_INDEX && bodyTag[row->indexB] == -1 )
		{
			bodyTag[row->indexB] = islandBodyCount;
			islandBodies[islandBodyCount++] = row->indexB;
		}
	}

	int bodyHead[2 * B3_JOINT_LDL_MAX_ROWS];
	int next[2 * B3_JOINT_LDL_MAX_ROWS];
	int visited[B3_JOINT_LDL_MAX_ROWS];
	int candidates[B3_JOINT_LDL_MAX_ROWS];
	for ( int i = 0; i < islandBodyCount; ++i )
	{
		bodyHead[i] = B3_NULL_INDEX;
	}
	for ( int i = 0; i < n; ++i )
	{
		visited[i] = B3_NULL_INDEX;
	}

	for ( int i = 0; i < n; ++i )
	{
		first[i] = i;
		const b3JointRow* ri = rows + idx[i];
		b[i] = -ri->rhs;
		a[i * n + i] = b3RowDotMinv( ri, ri );
		int candidateCount = 0;
		int firstCandidate = i;
		int bodies[2] = { ri->indexA, ri->indexB };
		for ( int side = 0; side < 2; ++side )
		{
			if ( bodies[side] == B3_NULL_INDEX )
			{
				continue;
			}
			int body = bodyTag[bodies[side]];
			for ( int link = bodyHead[body]; link != B3_NULL_INDEX; link = next[link] )
			{
				int j = link / 2;
				if ( visited[j] != i )
				{
					visited[j] = i;
					candidates[candidateCount++] = j;
					firstCandidate = b3MinInt( firstCandidate, j );
				}
			}
		}
		memset( a + i * n + firstCandidate, 0, ( i - firstCandidate ) * sizeof( double ) );
		for ( int k = 0; k < candidateCount; ++k )
		{
			int j = candidates[k];
			float s = b3RowDotMinv( ri, rows + idx[j] );
			if ( s != 0.0f )
			{
				first[i] = b3MinInt( first[i], j );
			}
			a[i * n + j] = s;
		}
		for ( int side = 0; side < 2; ++side )
		{
			if ( bodies[side] != B3_NULL_INDEX )
			{
				int body = bodyTag[bodies[side]];
				int link = 2 * i + side;
				next[link] = bodyHead[body];
				bodyHead[body] = link;
			}
		}
	}
	for ( int i = 0; i < islandBodyCount; ++i )
	{
		bodyTag[islandBodies[i]] = -1;
	}

	for ( int i = 0; i < n; ++i )
	{
		float d = (float)a[i * n + i];
		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}
		scale[i] = 1.0f / sqrtf( d );
	}

	for ( int i = 0; i < n; ++i )
	{
		b[i] *= scale[i];
		for ( int j = first[i]; j <= i; ++j )
		{
			double s = a[i * n + j] * scale[i] * scale[j];
			if ( i == j )
			{
				s += B3_JOINT_DIAG_EPS + ( positionProjection ? B3_JOINT_POSITION_CFM : B3_JOINT_CFM_TREE );
			}
			a[i * n + j] = s;
		}
	}

	b3DenseLdl( a, first, n );
	b3DenseLdlSolve( a, first, x, b, n );

	for ( int i = 0; i < n; ++i )
	{
		x[i] *= scale[i];
	}

	b3ApplyIsland( context, rows, idx, n, x, islandBodies, islandBodyCount, savedLin, savedAng, positionProjection );
	if ( positionProjection == false && b3LimitJointSpeeds( context, islandBodies, islandBodyCount, false ) < 1.0f )
	{
		b3SaveJointSpeeds( context, islandBodies, islandBodyCount, savedLin, savedAng );
		float target[B3_JOINT_LDL_MAX_ROWS];
		for ( int i = 0; i < n; ++i )
		{
			target[i] = b3JointRowVelocity( context, rows + idx[i] );
		}
		for ( int iteration = 0; iteration < 8; ++iteration )
		{
			if ( b3LimitJointSpeeds( context, islandBodies, islandBodyCount, true ) >= 0.99999f )
			{
				break;
			}
			for ( int i = 0; i < n; ++i )
			{
				b[i] = ( target[i] - b3JointRowVelocity( context, rows + idx[i] ) ) * scale[i];
			}
			b3DenseLdlSolve( a, first, x, b, n );
			if ( b3JointSolutionValid( x, n ) == false )
			{
				break;
			}
			for ( int i = 0; i < n; ++i )
			{
				b3ApplyDelta( context, rows + idx[i], x[i] * scale[i] );
			}
			if ( b3ValidateJointSpeeds( context, rows, idx, n, islandBodies, islandBodyCount, savedLin, savedAng ) == false )
			{
				break;
			}
		}
		b3ScaleJointSpeeds( context, islandBodies, islandBodyCount );
	}
}

static void b3JointPcgSolve( const b3IslandRow* __restrict pcgRows, int n, float* __restrict x, const float* __restrict b,
							 float* __restrict r, float* __restrict z, float* __restrict p, float* __restrict ap,
							 const float* __restrict scale,
							 const b3Matrix3* __restrict islandInvI, int islandBodyCount,
							 b3Vec3* __restrict dLin, b3Vec3* __restrict dAng, bool useBias, float cfm )
{
	const float invDiag = 1.0f / ( 1.0f + cfm );
	for ( int i = 0; i < n; ++i )
	{
		x[i] = 0.0f;
		r[i] = b[i];
		z[i] = invDiag * r[i];
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

	int maxIter = useBias ? ( n < 24 ? n : 24 ) : ( n < 8 ? n : 8 );
	const float tol = 1.0e-5f * b2;
	for ( int iter = 0; iter < maxIter; ++iter )
	{
		for ( int i = 0; i < n; ++i )
		{
			z[i] = scale[i] * p[i];
		}
		b3JointMatVec( pcgRows, n, z, ap, islandInvI, islandBodyCount, dLin, dAng );
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
			z[i] = invDiag * r[i];
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

static void b3SolveIslandPcg( b3StepContext* context, const b3JointRow* rows, const int* idx, int n,
							  b3IslandRow* pcgRows, float* x, float* b, float* r,
							  float* z, float* p, float* ap, float* target, float* scale,
							  int* islandBodies, b3Matrix3* islandInvI, int* bodyTag,
							  b3Vec3* dLin, b3Vec3* dAng, b3Vec3* savedLin, b3Vec3* savedAng,
							  const b3Matrix3* invInertias, bool useBias, bool positionProjection )
{
	int islandBodyCount = 0;
	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* row = rows + idx[k];
		if ( row->indexA != B3_NULL_INDEX && bodyTag[row->indexA] == -1 )
		{
			bodyTag[row->indexA] = islandBodyCount;
			islandBodies[islandBodyCount] = row->indexA;
			islandInvI[islandBodyCount] = invInertias[row->indexA];
			islandBodyCount += 1;
		}
		if ( row->indexB != B3_NULL_INDEX && bodyTag[row->indexB] == -1 )
		{
			bodyTag[row->indexB] = islandBodyCount;
			islandBodies[islandBodyCount] = row->indexB;
			islandInvI[islandBodyCount] = invInertias[row->indexB];
			islandBodyCount += 1;
		}
	}

	for ( int k = 0; k < n; ++k )
	{
		const b3JointRow* src = rows + idx[k];
		b3IslandRow* dst = pcgRows + k;
		dst->localA = src->indexA != B3_NULL_INDEX ? bodyTag[src->indexA] : B3_NULL_INDEX;
		dst->localB = src->indexB != B3_NULL_INDEX ? bodyTag[src->indexB] : B3_NULL_INDEX;
		dst->invMassA = src->invMassA;
		dst->invMassB = src->invMassB;
		dst->jLinA = src->jLinA;
		dst->jAngA = src->jAngA;
		dst->jLinB = src->jLinB;
		dst->jAngB = src->jAngB;
	}

	for ( int i = 0; i < islandBodyCount; ++i )
	{
		bodyTag[islandBodies[i]] = -1;
	}

	const float cfm = B3_JOINT_DIAG_EPS + ( positionProjection ? B3_JOINT_POSITION_CFM : B3_JOINT_CFM_TREE );
	for ( int i = 0; i < n; ++i )
	{
		const b3JointRow* row = rows + idx[i];
		float d = b3RowDotMinv( row, row );
		if ( d < B3_JOINT_DIAG_EPS )
		{
			d = B3_JOINT_DIAG_EPS;
		}
		scale[i] = 1.0f / sqrtf( d );
		b[i] = -row->rhs * scale[i];
	}

	b3JointPcgSolve( pcgRows, n, x, b, r, z, p, ap, scale, islandInvI, islandBodyCount, dLin, dAng, useBias, cfm );
	b3ApplyIsland( context, rows, idx, n, x, islandBodies, islandBodyCount, savedLin, savedAng, positionProjection );
	if ( positionProjection == false && b3LimitJointSpeeds( context, islandBodies, islandBodyCount, false ) < 1.0f )
	{
		b3SaveJointSpeeds( context, islandBodies, islandBodyCount, savedLin, savedAng );
		for ( int i = 0; i < n; ++i )
		{
			target[i] = b3JointRowVelocity( context, rows + idx[i] );
		}
		for ( int iteration = 0; iteration < 8; ++iteration )
		{
			if ( b3LimitJointSpeeds( context, islandBodies, islandBodyCount, true ) >= 0.99999f )
			{
				break;
			}
			for ( int i = 0; i < n; ++i )
			{
				b[i] = ( target[i] - b3JointRowVelocity( context, rows + idx[i] ) ) * scale[i];
			}
			b3JointPcgSolve( pcgRows, n, x, b, r, z, p, ap, scale, islandInvI, islandBodyCount,
							dLin, dAng, true, cfm );
			if ( b3JointSolutionValid( x, n ) == false )
			{
				break;
			}
			for ( int i = 0; i < n; ++i )
			{
				b3ApplyDelta( context, rows + idx[i], x[i] );
			}
			if ( b3ValidateJointSpeeds( context, rows, idx, n, islandBodies, islandBodyCount, savedLin, savedAng ) == false )
			{
				break;
			}
		}
		b3ScaleJointSpeeds( context, islandBodies, islandBodyCount );
	}
}

static bool b3SolveJointsDirectInternal( b3StepContext* context, bool useBias, bool resetImpulses, bool positionProjection )
{
	b3TracyCZoneNC( joint_direct, "JointDirect", b3_colorLemonChiffon, true );

	b3World* world = context->world;
	b3ConstraintGraph* graph = context->graph;
	int jointCount = 0;
	int rowCapacity = 0;
	for ( int colorIndex = 0; colorIndex < B3_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b3GraphColor* color = graph->colors + colorIndex;
		jointCount += color->jointSims.count;
		for ( int i = 0; i < color->jointSims.count; ++i )
		{
			const b3JointSim* joint = color->jointSims.data + i;
			switch ( joint->type )
			{
				case b3_weldJoint:
					rowCapacity += 6;
					break;
				case b3_sphericalJoint:
					rowCapacity += 3;
					break;
				case b3_revoluteJoint:
				case b3_prismaticJoint:
					rowCapacity += 5;
					break;
				case b3_distanceJoint:
					rowCapacity += 1;
					break;
				default:
					break;
			}
		}
	}

	if ( rowCapacity == 0 )
	{
		b3TracyCZoneEnd( joint_direct );
		return false;
	}

	int bodyCount = world->solverSets.data[b3_awakeSet].bodyStates.count;
	if ( bodyCount == 0 )
	{
		b3TracyCZoneEnd( joint_direct );
		return false;
	}

	b3Stack* stack = &world->stack;

	int phase1 = 0;
	phase1 += b3Align16( bodyCount * (int)sizeof( b3Matrix3 ) );
	phase1 += b3Align16( rowCapacity * (int)sizeof( b3JointRow ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );
	phase1 += b3Align16( jointCount * (int)sizeof( int ) );

	char* blob1 = (char*)b3StackAlloc( stack, phase1, "joint direct 1" );
	char* cursor = blob1;
	b3Matrix3* invInertias = (b3Matrix3*)b3CursorBump( &cursor, bodyCount * (int)sizeof( b3Matrix3 ) );
	b3JointRow* rows = (b3JointRow*)b3CursorBump( &cursor, rowCapacity * (int)sizeof( b3JointRow ) );
	int* parent = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* rank = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* islandHead = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	int* pairA = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );
	int* pairB = (int*)b3CursorBump( &cursor, jointCount * (int)sizeof( int ) );

	for ( int i = 0; i < bodyCount; ++i )
	{
		invInertias[i] = b3RotateInertia( context->states[i].deltaRotation, context->sims[i].invInertiaWorld );
		if ( positionProjection && context->sims[i].invMass > 0.0f )
		{
			float massScale = 1.0f / sqrtf( context->sims[i].invMass );
			invInertias[i].cx = b3MulSV( massScale, invInertias[i].cx );
			invInertias[i].cy = b3MulSV( massScale, invInertias[i].cy );
			invInertias[i].cz = b3MulSV( massScale, invInertias[i].cz );
		}
		parent[i] = i;
		rank[i] = 0;
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
			b3EmitJoint( rows, &rowCount, rowCapacity, joints + i, context, invInertias, useBias );
			if ( rowCount == begin )
			{
				continue;
			}

			int indexA = rows[begin].indexA;
			int indexB = rows[begin].indexB;
			if ( indexA != B3_NULL_INDEX && ( context->states[indexA].flags & b3_dynamicFlag ) == 0 )
			{
				indexA = B3_NULL_INDEX;
			}
			if ( indexB != B3_NULL_INDEX && ( context->states[indexB].flags & b3_dynamicFlag ) == 0 )
			{
				indexB = B3_NULL_INDEX;
			}
			if ( indexA == B3_NULL_INDEX && indexB == B3_NULL_INDEX )
			{
				rowCount = begin;
				continue;
			}
			for ( int k = begin; k < rowCount; ++k )
			{
				rows[k].indexA = indexA;
				rows[k].indexB = indexB;
				if ( positionProjection )
				{
					rows[k].invMassA = sqrtf( rows[k].invMassA );
					rows[k].invMassB = sqrtf( rows[k].invMassB );
				}
			}
			pairA[pairCount] = indexA;
			pairB[pairCount] = indexB;
			pairCount += 1;
		}
	}

	if ( rowCount == 0 )
	{
		b3StackFree( stack, blob1 );
		b3TracyCZoneEnd( joint_direct );
		return false;
	}

	if ( positionProjection )
	{
		bool needsCorrection = false;
		float threshold = B3_JOINT_BAUMGARTE * context->inv_h * 0.001f;
		for ( int i = 0; i < rowCount; ++i )
		{
			needsCorrection |= b3AbsFloat( rows[i].bias ) > threshold;
			rows[i].rhs = 4.0f * rows[i].bias;
			rows[i].lambda = NULL;
		}
		if ( needsCorrection == false )
		{
			b3StackFree( stack, blob1 );
			b3TracyCZoneEnd( joint_direct );
			return false;
		}
	}

	if ( resetImpulses )
	{
		for ( int i = 0; i < rowCount; ++i )
		{
			if ( rows[i].lambda != NULL )
			{
				*rows[i].lambda = 0.0f;
			}
		}
	}

	for ( int p = 0; p < pairCount; ++p )
	{
		int indexA = pairA[p];
		int indexB = pairB[p];
		if ( indexA != B3_NULL_INDEX && indexB != B3_NULL_INDEX )
		{
			b3UfUnion( parent, rank, indexA, indexB );
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

	int denseN = 0;
	for ( int i = 0; i < bodyCount; ++i )
	{
		if ( rank[i] <= B3_JOINT_LDL_MAX_ROWS )
		{
			denseN = b3MaxInt( denseN, rank[i] );
		}
	}

	int maxBodies = b3MinInt( bodyCount, 2 * maxN );
	int phase3 = 0;
	phase3 += b3Align16( maxN * (int)sizeof( int ) );
	phase3 += b3Align16( denseN * denseN * (int)sizeof( double ) );
	phase3 += b3Align16( maxN * (int)sizeof( b3IslandRow ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxN * (int)sizeof( float ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( int ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( b3Matrix3 ) );
	phase3 += b3Align16( bodyCount * (int)sizeof( int ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( b3Vec3 ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( b3Vec3 ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( b3Vec3 ) );
	phase3 += b3Align16( maxBodies * (int)sizeof( b3Vec3 ) );

	char* blob3 = (char*)b3StackAlloc( stack, phase3, "joint direct 3" );
	cursor = blob3;
	int* idx = (int*)b3CursorBump( &cursor, maxN * (int)sizeof( int ) );
	double* workA = (double*)b3CursorBump( &cursor, denseN * denseN * (int)sizeof( double ) );
	b3IslandRow* pcgRows = (b3IslandRow*)b3CursorBump( &cursor, maxN * (int)sizeof( b3IslandRow ) );
	float* workB = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workX = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workR = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workZ = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workP = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workAp = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workTarget = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	float* workScale = (float*)b3CursorBump( &cursor, maxN * (int)sizeof( float ) );
	int* islandBodies = (int*)b3CursorBump( &cursor, maxBodies * (int)sizeof( int ) );
	b3Matrix3* islandInvI = (b3Matrix3*)b3CursorBump( &cursor, maxBodies * (int)sizeof( b3Matrix3 ) );
	int* bodyTag = (int*)b3CursorBump( &cursor, bodyCount * (int)sizeof( int ) );
	b3Vec3* dLin = (b3Vec3*)b3CursorBump( &cursor, maxBodies * (int)sizeof( b3Vec3 ) );
	b3Vec3* dAng = (b3Vec3*)b3CursorBump( &cursor, maxBodies * (int)sizeof( b3Vec3 ) );
	b3Vec3* savedLin = (b3Vec3*)b3CursorBump( &cursor, maxBodies * (int)sizeof( b3Vec3 ) );
	b3Vec3* savedAng = (b3Vec3*)b3CursorBump( &cursor, maxBodies * (int)sizeof( b3Vec3 ) );

	for ( int i = 0; i < bodyCount; ++i )
	{
		bodyTag[i] = -1;
	}

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

		if ( positionProjection )
		{
			bool needsCorrection = false;
			float threshold = B3_JOINT_BAUMGARTE * context->inv_h * 0.001f;
			for ( int k = 0; k < n; ++k )
			{
				needsCorrection |= b3AbsFloat( rows[idx[k]].bias ) > threshold;
			}
			if ( needsCorrection == false )
			{
				continue;
			}
		}

		if ( n <= B3_JOINT_LDL_MAX_ROWS )
		{
			b3SolveIslandDense( context, rows, idx, n, workA, workB, workX, workScale, islandBodies, bodyTag, savedLin, savedAng,
								positionProjection );
		}
		else
		{
			b3SolveIslandPcg( context, rows, idx, n, pcgRows, workX, workB, workR, workZ, workP, workAp, workTarget, workScale,
							  islandBodies, islandInvI, bodyTag, dLin, dAng, savedLin, savedAng, invInertias, useBias, positionProjection );
		}
	}

	b3StackFree( stack, blob3 );
	b3StackFree( stack, blob2 );
	b3StackFree( stack, blob1 );

	b3TracyCZoneEnd( joint_direct );
	return true;
}

void b3SolveJoints_Direct( b3StepContext* context, bool useBias, bool resetImpulses )
{
	b3SolveJointsDirectInternal( context, useBias, resetImpulses, false );
}

void b3ProjectJointPositions( b3StepContext* context )
{
	for ( int i = 0; i < 16; ++i )
	{
		if ( b3SolveJointsDirectInternal( context, true, false, true ) == false )
		{
			break;
		}
	}
}
