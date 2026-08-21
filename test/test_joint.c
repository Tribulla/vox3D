// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

// One sub-test per joint type. Each creates the joint, exercises the shared
// b3Joint_* API plus every type-specific accessor, then steps to make sure the
// joint solves without tripping a validation assert.

typedef struct JointFixture
{
	b3WorldId worldId;
	b3BodyId groundId;
	b3BodyId bodyId;
} JointFixture;

// Static ground plus a dynamic box, anchored so a point-coincident joint starts
// satisfied. Gravity is off so the body stays put across the handful of steps
// each sub-test takes.
static JointFixture CreateJointFixture( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;

	JointFixture f;
	f.worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	f.groundId = b3CreateBody( f.worldId, &groundDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 4.0f, 0.0f };
	f.bodyId = b3CreateBody( f.worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3CreateHullShape( f.bodyId, &shapeDef, &box.base );

	return f;
}

// Place the joint anchor at the dynamic body so both local frames map to the
// same world point.
static void SetCommonFrames( b3JointDef* base, const JointFixture* f )
{
	base->bodyIdA = f->groundId;
	base->bodyIdB = f->bodyId;
	base->localFrameA.p = (b3Vec3){ 0.0f, 4.0f, 0.0f };
	base->localFrameB.p = (b3Vec3){ 0.0f, 0.0f, 0.0f };
}

// Step a few times, destroy the joint, then the world. Destroying the joint
// explicitly also covers b3DestroyJoint and stale-handle detection.
static int FinishJoint( b3JointId jointId, b3WorldId worldId )
{
	for ( int i = 0; i < 8; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3DestroyJoint( jointId, true );
	ENSURE( b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// Exercise the API shared by every joint type. Frames are saved and restored so
// the caller's setup survives.
static int ExerciseJointBase( b3JointId jointId, b3WorldId worldId, b3BodyId bodyIdA, b3BodyId bodyIdB, b3JointType expectedType )
{
	ENSURE( b3Joint_IsValid( jointId ) );
	ENSURE( b3Joint_GetType( jointId ) == expectedType );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyA( jointId ), bodyIdA ) );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( jointId ), bodyIdB ) );

	// B3_ID_EQUALS does not work for world ids
	b3WorldId gotWorld = b3Joint_GetWorld( jointId );
	ENSURE( gotWorld.index1 == worldId.index1 && gotWorld.generation == worldId.generation );

	b3Transform originalA = b3Joint_GetLocalFrameA( jointId );
	b3Transform originalB = b3Joint_GetLocalFrameB( jointId );

	b3Transform frameA = { { 0.1f, 0.2f, 0.3f }, b3Quat_identity };
	b3Joint_SetLocalFrameA( jointId, frameA );
	b3Transform gotA = b3Joint_GetLocalFrameA( jointId );
	ENSURE( gotA.p.x == frameA.p.x && gotA.p.y == frameA.p.y && gotA.p.z == frameA.p.z );

	b3Transform frameB = { { -0.4f, 0.5f, -0.6f }, b3Quat_identity };
	b3Joint_SetLocalFrameB( jointId, frameB );
	b3Transform gotB = b3Joint_GetLocalFrameB( jointId );
	ENSURE( gotB.p.x == frameB.p.x && gotB.p.y == frameB.p.y && gotB.p.z == frameB.p.z );

	b3Joint_SetCollideConnected( jointId, true );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == true );
	b3Joint_SetCollideConnected( jointId, false );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == false );

	int userData = 0;
	b3Joint_SetUserData( jointId, &userData );
	ENSURE( b3Joint_GetUserData( jointId ) == &userData );

	b3Joint_SetConstraintTuning( jointId, 90.0f, 3.0f );
	float hertz = 0.0f;
	float dampingRatio = 0.0f;
	b3Joint_GetConstraintTuning( jointId, &hertz, &dampingRatio );
	ENSURE( hertz == 90.0f );
	ENSURE( dampingRatio == 3.0f );

	b3Joint_SetForceThreshold( jointId, 100.0f );
	ENSURE( b3Joint_GetForceThreshold( jointId ) == 100.0f );

	b3Joint_SetTorqueThreshold( jointId, 200.0f );
	ENSURE( b3Joint_GetTorqueThreshold( jointId ) == 200.0f );

	b3Joint_WakeBodies( jointId );

	// No stable value to assert before the first step, call for coverage
	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	float linearSeparation = b3Joint_GetLinearSeparation( jointId );
	MAYBE_UNUSED( force );
	MAYBE_UNUSED( torque );
	MAYBE_UNUSED( linearSeparation );

	// Wheel joint angular separation is an unimplemented todo in joint.c that
	// asserts. Every other type computes it.
	if ( expectedType != b3_wheelJoint )
	{
		float angularSeparation = b3Joint_GetAngularSeparation( jointId );
		MAYBE_UNUSED( angularSeparation );
	}

	b3Joint_SetLocalFrameA( jointId, originalA );
	b3Joint_SetLocalFrameB( jointId, originalB );

	return 0;
}

static int TestParallelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	SetCommonFrames( &def.base, &f );
	def.hertz = 2.0f;
	def.dampingRatio = 0.5f;
	def.maxTorque = 100.0f;
	b3JointId jointId = b3CreateParallelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_parallelJoint ) != 0 )
	{
		return 1;
	}

	b3ParallelJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3ParallelJoint_GetSpringHertz( jointId ) == 5.0f );

	b3ParallelJoint_SetSpringDampingRatio( jointId, 0.7f );
	ENSURE( b3ParallelJoint_GetSpringDampingRatio( jointId ) == 0.7f );

	b3ParallelJoint_SetMaxTorque( jointId, 250.0f );
	ENSURE( b3ParallelJoint_GetMaxTorque( jointId ) == 250.0f );

	return FinishJoint( jointId, f.worldId );
}

static int TestDistanceJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	SetCommonFrames( &def.base, &f );
	def.length = 2.0f;
	b3JointId jointId = b3CreateDistanceJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_distanceJoint ) != 0 )
	{
		return 1;
	}

	b3DistanceJoint_SetLength( jointId, 3.0f );
	ENSURE( b3DistanceJoint_GetLength( jointId ) == 3.0f );

	b3DistanceJoint_EnableSpring( jointId, true );
	ENSURE( b3DistanceJoint_IsSpringEnabled( jointId ) == true );

	b3DistanceJoint_SetSpringForceRange( jointId, -50.0f, 75.0f );
	float lowerForce = 0.0f;
	float upperForce = 0.0f;
	b3DistanceJoint_GetSpringForceRange( jointId, &lowerForce, &upperForce );
	ENSURE( lowerForce == -50.0f && upperForce == 75.0f );

	b3DistanceJoint_SetSpringHertz( jointId, 4.0f );
	ENSURE( b3DistanceJoint_GetSpringHertz( jointId ) == 4.0f );

	b3DistanceJoint_SetSpringDampingRatio( jointId, 0.6f );
	ENSURE( b3DistanceJoint_GetSpringDampingRatio( jointId ) == 0.6f );

	b3DistanceJoint_EnableLimit( jointId, true );
	ENSURE( b3DistanceJoint_IsLimitEnabled( jointId ) == true );

	b3DistanceJoint_SetLengthRange( jointId, 1.0f, 5.0f );
	ENSURE( b3DistanceJoint_GetMinLength( jointId ) == 1.0f );
	ENSURE( b3DistanceJoint_GetMaxLength( jointId ) == 5.0f );

	float currentLength = b3DistanceJoint_GetCurrentLength( jointId );
	MAYBE_UNUSED( currentLength );

	b3DistanceJoint_EnableMotor( jointId, true );
	ENSURE( b3DistanceJoint_IsMotorEnabled( jointId ) == true );

	b3DistanceJoint_SetMotorSpeed( jointId, 1.5f );
	ENSURE( b3DistanceJoint_GetMotorSpeed( jointId ) == 1.5f );

	b3DistanceJoint_SetMaxMotorForce( jointId, 25.0f );
	ENSURE( b3DistanceJoint_GetMaxMotorForce( jointId ) == 25.0f );

	float motorForce = b3DistanceJoint_GetMotorForce( jointId );
	MAYBE_UNUSED( motorForce );

	return FinishJoint( jointId, f.worldId );
}

static int TestFilterJoint( void )
{
	JointFixture f = CreateJointFixture();

	// The filter joint has no type-specific API. It only disables collision and
	// keeps both bodies in the same island.
	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	b3JointId jointId = b3CreateFilterJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_filterJoint ) != 0 )
	{
		return 1;
	}

	return FinishJoint( jointId, f.worldId );
}

static int TestMotorJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MotorJointDef def = b3DefaultMotorJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateMotorJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_motorJoint ) != 0 )
	{
		return 1;
	}

	b3Vec3 linearVelocity = { 1.0f, 2.0f, 3.0f };
	b3MotorJoint_SetLinearVelocity( jointId, linearVelocity );
	b3Vec3 gotLinear = b3MotorJoint_GetLinearVelocity( jointId );
	ENSURE( gotLinear.x == 1.0f && gotLinear.y == 2.0f && gotLinear.z == 3.0f );

	b3Vec3 angularVelocity = { 0.1f, 0.2f, 0.3f };
	b3MotorJoint_SetAngularVelocity( jointId, angularVelocity );
	b3Vec3 gotAngular = b3MotorJoint_GetAngularVelocity( jointId );
	ENSURE( gotAngular.x == 0.1f && gotAngular.y == 0.2f && gotAngular.z == 0.3f );

	b3MotorJoint_SetMaxVelocityForce( jointId, 500.0f );
	ENSURE( b3MotorJoint_GetMaxVelocityForce( jointId ) == 500.0f );

	b3MotorJoint_SetMaxVelocityTorque( jointId, 600.0f );
	ENSURE( b3MotorJoint_GetMaxVelocityTorque( jointId ) == 600.0f );

	b3MotorJoint_SetLinearHertz( jointId, 3.0f );
	ENSURE( b3MotorJoint_GetLinearHertz( jointId ) == 3.0f );

	b3MotorJoint_SetLinearDampingRatio( jointId, 0.8f );
	ENSURE( b3MotorJoint_GetLinearDampingRatio( jointId ) == 0.8f );

	b3MotorJoint_SetAngularHertz( jointId, 4.0f );
	ENSURE( b3MotorJoint_GetAngularHertz( jointId ) == 4.0f );

	b3MotorJoint_SetAngularDampingRatio( jointId, 0.9f );
	ENSURE( b3MotorJoint_GetAngularDampingRatio( jointId ) == 0.9f );

	b3MotorJoint_SetMaxSpringForce( jointId, 700.0f );
	ENSURE( b3MotorJoint_GetMaxSpringForce( jointId ) == 700.0f );

	b3MotorJoint_SetMaxSpringTorque( jointId, 800.0f );
	ENSURE( b3MotorJoint_GetMaxSpringTorque( jointId ) == 800.0f );

	return FinishJoint( jointId, f.worldId );
}

static int TestPrismaticJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreatePrismaticJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_prismaticJoint ) != 0 )
	{
		return 1;
	}

	b3PrismaticJoint_EnableSpring( jointId, true );
	ENSURE( b3PrismaticJoint_IsSpringEnabled( jointId ) == true );

	b3PrismaticJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3PrismaticJoint_GetSpringHertz( jointId ) == 5.0f );

	b3PrismaticJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3PrismaticJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	b3PrismaticJoint_SetTargetTranslation( jointId, 1.0f );
	ENSURE( b3PrismaticJoint_GetTargetTranslation( jointId ) == 1.0f );

	b3PrismaticJoint_EnableLimit( jointId, true );
	ENSURE( b3PrismaticJoint_IsLimitEnabled( jointId ) == true );

	b3PrismaticJoint_SetLimits( jointId, -2.0f, 2.0f );
	ENSURE( b3PrismaticJoint_GetLowerLimit( jointId ) == -2.0f );
	ENSURE( b3PrismaticJoint_GetUpperLimit( jointId ) == 2.0f );

	b3PrismaticJoint_EnableMotor( jointId, true );
	ENSURE( b3PrismaticJoint_IsMotorEnabled( jointId ) == true );

	b3PrismaticJoint_SetMotorSpeed( jointId, 1.5f );
	ENSURE( b3PrismaticJoint_GetMotorSpeed( jointId ) == 1.5f );

	b3PrismaticJoint_SetMaxMotorForce( jointId, 30.0f );
	ENSURE( b3PrismaticJoint_GetMaxMotorForce( jointId ) == 30.0f );

	float motorForce = b3PrismaticJoint_GetMotorForce( jointId );
	float translation = b3PrismaticJoint_GetTranslation( jointId );
	float speed = b3PrismaticJoint_GetSpeed( jointId );
	MAYBE_UNUSED( motorForce );
	MAYBE_UNUSED( translation );
	MAYBE_UNUSED( speed );

	return FinishJoint( jointId, f.worldId );
}

static int TestRevoluteJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateRevoluteJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_revoluteJoint ) != 0 )
	{
		return 1;
	}

	b3RevoluteJoint_EnableSpring( jointId, true );
	ENSURE( b3RevoluteJoint_IsSpringEnabled( jointId ) == true );

	b3RevoluteJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3RevoluteJoint_GetSpringHertz( jointId ) == 5.0f );

	b3RevoluteJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3RevoluteJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	b3RevoluteJoint_SetTargetAngle( jointId, 0.5f );
	ENSURE( b3RevoluteJoint_GetTargetAngle( jointId ) == 0.5f );

	float angle = b3RevoluteJoint_GetAngle( jointId );
	MAYBE_UNUSED( angle );

	b3RevoluteJoint_EnableLimit( jointId, true );
	ENSURE( b3RevoluteJoint_IsLimitEnabled( jointId ) == true );

	b3RevoluteJoint_SetLimits( jointId, -1.0f, 1.0f );
	ENSURE( b3RevoluteJoint_GetLowerLimit( jointId ) == -1.0f );
	ENSURE( b3RevoluteJoint_GetUpperLimit( jointId ) == 1.0f );

	b3RevoluteJoint_EnableMotor( jointId, true );
	ENSURE( b3RevoluteJoint_IsMotorEnabled( jointId ) == true );

	b3RevoluteJoint_SetMotorSpeed( jointId, 2.0f );
	ENSURE( b3RevoluteJoint_GetMotorSpeed( jointId ) == 2.0f );

	b3RevoluteJoint_SetMaxMotorTorque( jointId, 40.0f );
	ENSURE( b3RevoluteJoint_GetMaxMotorTorque( jointId ) == 40.0f );

	float motorTorque = b3RevoluteJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestSphericalJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateSphericalJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_sphericalJoint ) != 0 )
	{
		return 1;
	}

	b3SphericalJoint_EnableConeLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsConeLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetConeLimit( jointId, 0.5f );
	ENSURE( b3SphericalJoint_GetConeLimit( jointId ) == 0.5f );

	float coneAngle = b3SphericalJoint_GetConeAngle( jointId );
	MAYBE_UNUSED( coneAngle );

	b3SphericalJoint_EnableTwistLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsTwistLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetTwistLimits( jointId, -0.5f, 0.5f );
	ENSURE( b3SphericalJoint_GetLowerTwistLimit( jointId ) == -0.5f );
	ENSURE( b3SphericalJoint_GetUpperTwistLimit( jointId ) == 0.5f );

	float twistAngle = b3SphericalJoint_GetTwistAngle( jointId );
	MAYBE_UNUSED( twistAngle );

	b3SphericalJoint_EnableSpring( jointId, true );
	ENSURE( b3SphericalJoint_IsSpringEnabled( jointId ) == true );

	b3SphericalJoint_SetSpringHertz( jointId, 5.0f );
	ENSURE( b3SphericalJoint_GetSpringHertz( jointId ) == 5.0f );

	b3SphericalJoint_SetSpringDampingRatio( jointId, 0.5f );
	ENSURE( b3SphericalJoint_GetSpringDampingRatio( jointId ) == 0.5f );

	// 90 degrees about z, a unit quaternion that round-trips through storage
	b3Quat targetRotation = { { 0.0f, 0.0f, 0.7071068f }, 0.7071068f };
	b3SphericalJoint_SetTargetRotation( jointId, targetRotation );
	b3Quat gotRotation = b3SphericalJoint_GetTargetRotation( jointId );
	ENSURE_SMALL( gotRotation.v.x - targetRotation.v.x, 1.0e-5f );
	ENSURE_SMALL( gotRotation.v.y - targetRotation.v.y, 1.0e-5f );
	ENSURE_SMALL( gotRotation.v.z - targetRotation.v.z, 1.0e-5f );
	ENSURE_SMALL( gotRotation.s - targetRotation.s, 1.0e-5f );

	b3SphericalJoint_EnableMotor( jointId, true );
	ENSURE( b3SphericalJoint_IsMotorEnabled( jointId ) == true );

	b3Vec3 motorVelocity = { 0.1f, 0.2f, 0.3f };
	b3SphericalJoint_SetMotorVelocity( jointId, motorVelocity );
	b3Vec3 gotMotorVelocity = b3SphericalJoint_GetMotorVelocity( jointId );
	ENSURE( gotMotorVelocity.x == 0.1f && gotMotorVelocity.y == 0.2f && gotMotorVelocity.z == 0.3f );

	b3SphericalJoint_SetMaxMotorTorque( jointId, 50.0f );
	ENSURE( b3SphericalJoint_GetMaxMotorTorque( jointId ) == 50.0f );

	b3Vec3 motorTorque = b3SphericalJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestWeldJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WeldJointDef def = b3DefaultWeldJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWeldJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_weldJoint ) != 0 )
	{
		return 1;
	}

	b3WeldJoint_SetLinearHertz( jointId, 3.0f );
	ENSURE( b3WeldJoint_GetLinearHertz( jointId ) == 3.0f );

	b3WeldJoint_SetLinearDampingRatio( jointId, 0.5f );
	ENSURE( b3WeldJoint_GetLinearDampingRatio( jointId ) == 0.5f );

	b3WeldJoint_SetAngularHertz( jointId, 4.0f );
	ENSURE( b3WeldJoint_GetAngularHertz( jointId ) == 4.0f );

	b3WeldJoint_SetAngularDampingRatio( jointId, 0.7f );
	ENSURE( b3WeldJoint_GetAngularDampingRatio( jointId ) == 0.7f );

	return FinishJoint( jointId, f.worldId );
}

static int TestWheelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WheelJointDef def = b3DefaultWheelJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWheelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_wheelJoint ) != 0 )
	{
		return 1;
	}

	b3WheelJoint_EnableSuspension( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionHertz( jointId, 5.0f );
	ENSURE( b3WheelJoint_GetSuspensionHertz( jointId ) == 5.0f );

	b3WheelJoint_SetSuspensionDampingRatio( jointId, 0.5f );
	ENSURE( b3WheelJoint_GetSuspensionDampingRatio( jointId ) == 0.5f );

	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionLimits( jointId, -1.0f, 1.0f );
	ENSURE( b3WheelJoint_GetLowerSuspensionLimit( jointId ) == -1.0f );
	ENSURE( b3WheelJoint_GetUpperSuspensionLimit( jointId ) == 1.0f );

	b3WheelJoint_EnableSpinMotor( jointId, true );
	ENSURE( b3WheelJoint_IsSpinMotorEnabled( jointId ) == true );

	b3WheelJoint_SetSpinMotorSpeed( jointId, 6.0f );
	ENSURE( b3WheelJoint_GetSpinMotorSpeed( jointId ) == 6.0f );

	b3WheelJoint_SetMaxSpinTorque( jointId, 35.0f );
	ENSURE( b3WheelJoint_GetMaxSpinTorque( jointId ) == 35.0f );

	float spinSpeed = b3WheelJoint_GetSpinSpeed( jointId );
	float spinTorque = b3WheelJoint_GetSpinTorque( jointId );
	MAYBE_UNUSED( spinSpeed );
	MAYBE_UNUSED( spinTorque );

	b3WheelJoint_EnableSteering( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringHertz( jointId, 7.0f );
	ENSURE( b3WheelJoint_GetSteeringHertz( jointId ) == 7.0f );

	b3WheelJoint_SetSteeringDampingRatio( jointId, 0.8f );
	ENSURE( b3WheelJoint_GetSteeringDampingRatio( jointId ) == 0.8f );

	b3WheelJoint_SetMaxSteeringTorque( jointId, 45.0f );
	ENSURE( b3WheelJoint_GetMaxSteeringTorque( jointId ) == 45.0f );

	b3WheelJoint_EnableSteeringLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringLimits( jointId, -0.6f, 0.6f );
	ENSURE( b3WheelJoint_GetLowerSteeringLimit( jointId ) == -0.6f );
	ENSURE( b3WheelJoint_GetUpperSteeringLimit( jointId ) == 0.6f );

	b3WheelJoint_SetTargetSteeringAngle( jointId, 0.25f );
	ENSURE( b3WheelJoint_GetTargetSteeringAngle( jointId ) == 0.25f );

	float steeringAngle = b3WheelJoint_GetSteeringAngle( jointId );
	float steeringTorque = b3WheelJoint_GetSteeringTorque( jointId );
	MAYBE_UNUSED( steeringAngle );
	MAYBE_UNUSED( steeringTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestHangingSphericalChain( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	enum
	{
		kLinkCount = 12
	};
	const float halfExtent = 0.5f;
	b3BoxHull box = b3MakeBoxHull( halfExtent, 0.125f, 0.125f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 5.0f;

	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	b3JointId jointIds[kLinkCount];
	b3BodyId parent = groundId;
	float x = 0.0f;

	for ( int i = 0; i < kLinkCount; ++i )
	{
		x += halfExtent;
		bodyDef.position = (b3Pos){ x, 10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );

		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = ( i == 0 ) ? (b3Vec3){ 0.0f, 10.0f, 0.0f } : (b3Vec3){ halfExtent, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -halfExtent, 0.0f, 0.0f };
		jointIds[i] = b3CreateSphericalJoint( worldId, &jointDef );

		parent = bodyId;
		x += halfExtent;
	}

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float maxSeparation = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		float separation = b3Joint_GetLinearSeparation( jointIds[i] );
		maxSeparation = b3MaxFloat( maxSeparation, separation );
	}

	if ( maxSeparation >= 0.01f )
	{
		printf( "  hanging spherical chain maxSeparation=%g\n", maxSeparation );
	}

	ENSURE( maxSeparation < 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestWeldCantilever( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 2.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 10.0f;
	b3BoxHull box = b3MakeBoxHull( 2.0f, 0.15f, 0.15f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	jointDef.base.bodyIdA = groundId;
	jointDef.base.bodyIdB = bodyId;
	jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, 5.0f, 0.0f };
	jointDef.base.localFrameB.p = (b3Vec3){ -2.0f, 0.0f, 0.0f };
	b3JointId jointId = b3CreateWeldJoint( worldId, &jointDef );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float linearSeparation = b3Joint_GetLinearSeparation( jointId );
	float angularSeparation = b3Joint_GetAngularSeparation( jointId );
	ENSURE( linearSeparation < 0.005f );
	ENSURE( angularSeparation < 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestHangingWeldChain( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	enum
	{
		kLinkCount = 8
	};
	const float hx = 0.5f;
	const float hy = 0.1f;
	b3BoxHull box = b3MakeBoxHull( hx, hy, hy );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 2.0f;

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3JointId jointIds[kLinkCount];
	b3BodyId parent = groundId;
	float x = 0.0f;

	for ( int i = 0; i < kLinkCount; ++i )
	{
		x += hx;
		bodyDef.position = (b3Pos){ x, 10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );

		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = ( i == 0 ) ? (b3Vec3){ 0.0f, 10.0f, 0.0f } : (b3Vec3){ hx, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -hx, 0.0f, 0.0f };
		jointIds[i] = b3CreateWeldJoint( worldId, &jointDef );

		parent = bodyId;
		x += hx;
	}

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float maxLinear = 0.0f;
	float maxAngular = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		maxLinear = b3MaxFloat( maxLinear, b3Joint_GetLinearSeparation( jointIds[i] ) );
		maxAngular = b3MaxFloat( maxAngular, b3Joint_GetAngularSeparation( jointIds[i] ) );
	}

	if ( maxLinear >= 0.01f || maxAngular >= 0.02f )
	{
		printf( "  hanging weld chain maxLinear=%g maxAngular=%g\n", maxLinear, maxAngular );
	}

	ENSURE( maxLinear < 0.01f );
	ENSURE( maxAngular < 0.02f );

	b3DestroyWorld( worldId );
	return 0;
}

// Overconstrained weld grid on the ground. A tree-shaped chain is full rank;
// a voxel-style lattice is not, and used to explode on the first step.
static int TestWeldLatticeOnGround( void )
{
	enum
	{
		kN = 3
	};
	const float h = 0.5f;
	const float spacing = 2.0f * h;

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3BodyId bodies[kN][kN][kN];
	b3JointId joints[3 * kN * kN * kN];
	int jointCount = 0;

	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				bodyDef.position = (b3Pos){ ( ix - 1 ) * spacing, 2.0f + iy * spacing, ( iz - 1 ) * spacing };
				bodies[ix][iy][iz] = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodies[ix][iy][iz], &shapeDef, &box.base );
			}
		}
	}

	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				if ( ix + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix + 1][iy][iz];
					jointDef.base.localFrameA.p = (b3Vec3){ h, 0.0f, 0.0f };
					jointDef.base.localFrameB.p = (b3Vec3){ -h, 0.0f, 0.0f };
					joints[jointCount++] = b3CreateWeldJoint( worldId, &jointDef );
				}
				if ( iy + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix][iy + 1][iz];
					jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, h, 0.0f };
					jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, -h, 0.0f };
					joints[jointCount++] = b3CreateWeldJoint( worldId, &jointDef );
				}
				if ( iz + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix][iy][iz + 1];
					jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, 0.0f, h };
					jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.0f, -h };
					joints[jointCount++] = b3CreateWeldJoint( worldId, &jointDef );
				}
			}
		}
	}

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float maxSpeed = 0.0f;
	float maxLinear = 0.0f;
	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				b3Vec3 v = b3Body_GetLinearVelocity( bodies[ix][iy][iz] );
				ENSURE( b3IsValidVec3( v ) );
				maxSpeed = b3MaxFloat( maxSpeed, b3Length( v ) );

				b3Pos p = b3Body_GetPosition( bodies[ix][iy][iz] );
				ENSURE( p.y > -1.0f && p.y < 20.0f );
			}
		}
	}

	for ( int i = 0; i < jointCount; ++i )
	{
		maxLinear = b3MaxFloat( maxLinear, b3Joint_GetLinearSeparation( joints[i] ) );
	}

	if ( maxSpeed >= 40.0f || maxLinear >= 0.05f )
	{
		printf( "  weld lattice maxSpeed=%g maxLinear=%g\n", maxSpeed, maxLinear );
	}

	ENSURE( maxSpeed < 40.0f );
	ENSURE( maxLinear < 0.05f );

	b3DestroyWorld( worldId );
	return 0;
}

// Hanging weld chain attached to a dynamic box that is sitting on the ground.
static int TestHangingWeldFromDynamicOnGround( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 1.0f, 0.0f };
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 4.0f;
	b3BoxHull rootBox = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
	b3CreateHullShape( boxId, &shapeDef, &rootBox.base );

	enum
	{
		kLinkCount = 8
	};
	const float hx = 0.5f;
	const float hy = 0.1f;
	b3BoxHull linkBox = b3MakeBoxHull( hx, hy, hy );
	shapeDef.density = 2.0f;

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3JointId jointIds[kLinkCount];
	b3BodyId parent = boxId;
	float x = 1.0f;

	for ( int i = 0; i < kLinkCount; ++i )
	{
		x += hx;
		bodyDef.position = (b3Pos){ x, 1.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &linkBox.base );

		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = ( i == 0 ) ? (b3Vec3){ 1.0f, 0.0f, 0.0f } : (b3Vec3){ hx, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -hx, 0.0f, 0.0f };
		jointIds[i] = b3CreateWeldJoint( worldId, &jointDef );

		parent = bodyId;
		x += hx;
	}

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Vec3 rootV = b3Body_GetLinearVelocity( boxId );
	ENSURE( b3IsValidVec3( rootV ) );
	ENSURE( b3Length( rootV ) < 20.0f );

	b3Pos rootP = b3Body_GetPosition( boxId );
	ENSURE( rootP.y > 0.0f && rootP.y < 8.0f );

	float maxLinear = 0.0f;
	float maxAngular = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		maxLinear = b3MaxFloat( maxLinear, b3Joint_GetLinearSeparation( jointIds[i] ) );
		maxAngular = b3MaxFloat( maxAngular, b3Joint_GetAngularSeparation( jointIds[i] ) );
	}

	if ( maxLinear >= 0.02f || maxAngular >= 0.04f )
	{
		printf( "  hanging weld from dynamic maxLinear=%g maxAngular=%g\n", maxLinear, maxAngular );
	}

	ENSURE( maxLinear < 0.02f );
	ENSURE( maxAngular < 0.04f );

	b3DestroyWorld( worldId );
	return 0;
}

// Lattice plus a hanging chain. Without splitting the Direct island, the
// redundant welds and the chain share one Schur system and explode.
static int TestWeldLatticeWithHangingChain( void )
{
	enum
	{
		kN = 3,
		kLinkCount = 8
	};
	const float h = 0.5f;
	const float spacing = 2.0f * h;

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3BodyId bodies[kN][kN][kN];

	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				bodyDef.position = (b3Pos){ ( ix - 1 ) * spacing, 2.0f + iy * spacing, ( iz - 1 ) * spacing };
				bodies[ix][iy][iz] = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodies[ix][iy][iz], &shapeDef, &box.base );
			}
		}
	}

	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				if ( ix + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix + 1][iy][iz];
					jointDef.base.localFrameA.p = (b3Vec3){ h, 0.0f, 0.0f };
					jointDef.base.localFrameB.p = (b3Vec3){ -h, 0.0f, 0.0f };
					b3CreateWeldJoint( worldId, &jointDef );
				}
				if ( iy + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix][iy + 1][iz];
					jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, h, 0.0f };
					jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, -h, 0.0f };
					b3CreateWeldJoint( worldId, &jointDef );
				}
				if ( iz + 1 < kN )
				{
					jointDef.base.bodyIdA = bodies[ix][iy][iz];
					jointDef.base.bodyIdB = bodies[ix][iy][iz + 1];
					jointDef.base.localFrameA.p = (b3Vec3){ 0.0f, 0.0f, h };
					jointDef.base.localFrameB.p = (b3Vec3){ 0.0f, 0.0f, -h };
					b3CreateWeldJoint( worldId, &jointDef );
				}
			}
		}
	}

	const float hx = 0.5f;
	const float hy = 0.1f;
	b3BoxHull linkBox = b3MakeBoxHull( hx, hy, hy );
	shapeDef.density = 2.0f;
	b3JointId chainIds[kLinkCount];
	b3BodyId parent = bodies[kN - 1][0][1];
	b3Pos parentPos = b3Body_GetPosition( parent );
	float x = parentPos.x + h;

	for ( int i = 0; i < kLinkCount; ++i )
	{
		x += hx;
		bodyDef.position = (b3Pos){ x, parentPos.y, parentPos.z };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, &linkBox.base );

		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = ( i == 0 ) ? (b3Vec3){ h, 0.0f, 0.0f } : (b3Vec3){ hx, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -hx, 0.0f, 0.0f };
		chainIds[i] = b3CreateWeldJoint( worldId, &jointDef );

		parent = bodyId;
		x += hx;
	}

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float maxSpeed = 0.0f;
	for ( int iz = 0; iz < kN; ++iz )
	{
		for ( int iy = 0; iy < kN; ++iy )
		{
			for ( int ix = 0; ix < kN; ++ix )
			{
				b3Vec3 v = b3Body_GetLinearVelocity( bodies[ix][iy][iz] );
				ENSURE( b3IsValidVec3( v ) );
				maxSpeed = b3MaxFloat( maxSpeed, b3Length( v ) );
				b3Pos p = b3Body_GetPosition( bodies[ix][iy][iz] );
				ENSURE( p.y > -1.0f && p.y < 20.0f );
			}
		}
	}

	float maxLinear = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		b3Vec3 v = b3Body_GetLinearVelocity( b3Joint_GetBodyB( chainIds[i] ) );
		ENSURE( b3IsValidVec3( v ) );
		maxSpeed = b3MaxFloat( maxSpeed, b3Length( v ) );
		maxLinear = b3MaxFloat( maxLinear, b3Joint_GetLinearSeparation( chainIds[i] ) );
	}

	if ( maxSpeed >= 5.0f || maxLinear >= 0.05f )
	{
		printf( "  lattice+chain maxSpeed=%g maxLinear=%g\n", maxSpeed, maxLinear );
	}

	ENSURE( maxSpeed < 5.0f );
	ENSURE( maxLinear < 0.05f );

	b3DestroyWorld( worldId );
	return 0;
}

// Elongated hull planks on the floor, welded end to end — the app scene on this
// branch, not a voxel lattice. Sleep is off so a NaN Direct solve would freeze
// the chain in place the same way the app does.
static int TestWeldPlankChainOnGround( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 40.0f, 0.5f, 40.0f );
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	enum
	{
		kLinkCount = 6
	};
	const float hx = 1.5f;
	const float hy = 0.12f;
	b3BoxHull box = b3MakeBoxHull( hx, hy, hy );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 2.0f;

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3BodyId bodies[kLinkCount];
	b3JointId jointIds[kLinkCount - 1];

	float x = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		x += hx;
		bodyDef.position = (b3Pos){ x, hy + 0.02f, 0.0f };
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
		x += hx;
	}

	for ( int i = 0; i < kLinkCount - 1; ++i )
	{
		jointDef.base.bodyIdA = bodies[i];
		jointDef.base.bodyIdB = bodies[i + 1];
		jointDef.base.localFrameA.p = (b3Vec3){ hx, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -hx, 0.0f, 0.0f };
		jointIds[i] = b3CreateWeldJoint( worldId, &jointDef );
	}

	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float maxSpeed = 0.0f;
	float maxLinear = 0.0f;
	for ( int i = 0; i < kLinkCount; ++i )
	{
		b3Vec3 v = b3Body_GetLinearVelocity( bodies[i] );
		ENSURE( b3IsValidVec3( v ) );
		maxSpeed = b3MaxFloat( maxSpeed, b3Length( v ) );
		b3Pos p = b3Body_GetPosition( bodies[i] );
		ENSURE( p.y > -1.0f && p.y < 20.0f );
	}
	for ( int i = 0; i < kLinkCount - 1; ++i )
	{
		maxLinear = b3MaxFloat( maxLinear, b3Joint_GetLinearSeparation( jointIds[i] ) );
	}

	if ( maxSpeed >= 8.0f || maxLinear >= 0.05f )
	{
		printf( "  plank chain maxSpeed=%g maxLinear=%g\n", maxSpeed, maxLinear );
	}

	ENSURE( maxSpeed < 8.0f );
	ENSURE( maxLinear < 0.05f );

	b3DestroyWorld( worldId );
	return 0;
}

// Two equal-mass boxes welded together, then one is teleported away. Both
// bodies must move toward each other and the gap must close quickly.
static int TestWeldPullApartRecovery( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	bodyDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( bodyA, &shapeDef, &box.base );

	bodyDef.position = (b3Pos){ 1.0f, 0.0f, 0.0f };
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( bodyB, &shapeDef, &box.base );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	jointDef.base.bodyIdA = bodyA;
	jointDef.base.bodyIdB = bodyB;
	jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
	jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
	b3JointId jointId = b3CreateWeldJoint( worldId, &jointDef );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Body_SetTransform( bodyB, (b3Pos){ 3.0f, 0.0f, 0.0f }, b3Quat_identity );
	b3Body_SetLinearVelocity( bodyA, b3Vec3_zero );
	b3Body_SetLinearVelocity( bodyB, b3Vec3_zero );

	float startSep = b3Joint_GetLinearSeparation( jointId );
	ENSURE( startSep > 1.5f );

	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos pA = b3Body_GetPosition( bodyA );
	b3Pos pB = b3Body_GetPosition( bodyB );
	float sep = b3Joint_GetLinearSeparation( jointId );

	if ( pA.x <= 0.05f || pB.x >= 2.95f || sep >= 0.15f )
	{
		printf( "  pull-apart pA.x=%g pB.x=%g sep=%g\n", pA.x, pB.x, sep );
	}

	ENSURE( pA.x > 0.05f );
	ENSURE( pB.x < 2.95f );
	ENSURE( sep < 0.15f );

	b3DestroyWorld( worldId );
	return 0;
}

// Five equal boxes welded in a line, plus a chord from the first to the last.
// Yanking the middle link used to leave the chord as the spanning tree, so the
// chain stayed a perfect bar and the middle body was only a stretched leftover.
static int TestWeldPullMiddleLink( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	enum
	{
		kCount = 5
	};
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3BodyId bodies[kCount];
	for ( int i = 0; i < kCount; ++i )
	{
		bodyDef.position = (b3Pos){ (float)i, 0.0f, 0.0f };
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
	}

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	b3JointId chainIds[kCount - 1];
	for ( int i = 0; i < kCount - 1; ++i )
	{
		jointDef.base.bodyIdA = bodies[i];
		jointDef.base.bodyIdB = bodies[i + 1];
		jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
		chainIds[i] = b3CreateWeldJoint( worldId, &jointDef );
	}

	jointDef.base.bodyIdA = bodies[0];
	jointDef.base.bodyIdB = bodies[kCount - 1];
	jointDef.base.localFrameA.p = (b3Vec3){ 2.0f, 0.0f, 0.0f };
	jointDef.base.localFrameB.p = (b3Vec3){ -2.0f, 0.0f, 0.0f };
	b3CreateWeldJoint( worldId, &jointDef );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Body_SetTransform( bodies[2], (b3Pos){ 2.0f, 2.0f, 0.0f }, b3Quat_identity );
	for ( int i = 0; i < kCount; ++i )
	{
		b3Body_SetLinearVelocity( bodies[i], b3Vec3_zero );
	}

	ENSURE( b3Joint_GetLinearSeparation( chainIds[1] ) > 1.5f );
	ENSURE( b3Joint_GetLinearSeparation( chainIds[2] ) > 1.5f );

	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos p1 = b3Body_GetPosition( bodies[1] );
	b3Pos p3 = b3Body_GetPosition( bodies[3] );
	float sep12 = b3Joint_GetLinearSeparation( chainIds[1] );
	float sep23 = b3Joint_GetLinearSeparation( chainIds[2] );

	if ( p1.y <= 0.15f || p3.y <= 0.15f || sep12 >= 0.25f || sep23 >= 0.25f )
	{
		printf( "  middle-link p1.y=%g p3.y=%g sep12=%g sep23=%g\n", p1.y, p3.y, sep12, sep23 );
	}

	ENSURE( p1.y > 0.15f );
	ENSURE( p3.y > 0.15f );
	ENSURE( sep12 < 0.25f );
	ENSURE( sep23 < 0.25f );

	b3DestroyWorld( worldId );
	return 0;
}

int JointTest( void )
{
	RUN_SUBTEST( TestParallelJoint );
	RUN_SUBTEST( TestDistanceJoint );
	RUN_SUBTEST( TestFilterJoint );
	RUN_SUBTEST( TestMotorJoint );
	RUN_SUBTEST( TestPrismaticJoint );
	RUN_SUBTEST( TestRevoluteJoint );
	RUN_SUBTEST( TestSphericalJoint );
	RUN_SUBTEST( TestWeldJoint );
	RUN_SUBTEST( TestWheelJoint );
	RUN_SUBTEST( TestHangingSphericalChain );
	RUN_SUBTEST( TestWeldCantilever );
	RUN_SUBTEST( TestHangingWeldChain );
	RUN_SUBTEST( TestWeldLatticeOnGround );
	RUN_SUBTEST( TestHangingWeldFromDynamicOnGround );
	RUN_SUBTEST( TestWeldLatticeWithHangingChain );
	RUN_SUBTEST( TestWeldPlankChainOnGround );
	RUN_SUBTEST( TestWeldPullApartRecovery );
	RUN_SUBTEST( TestWeldPullMiddleLink );

	return 0;
}
