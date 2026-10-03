#include "box3d/box3d.h"
#include "human.h"
#include "test_macros.h"

#include <stdio.h>

static int TestSuspensionBridge( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( worldId, &groundDef );
	enum { count = 160 };
	b3BodyId bodies[count];
	b3JointId joints[2 * ( count + 1 )];
	b3BoxHull box = b3MakeBoxHull( 0.125f, 0.125f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20.0f;
	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	jointDef.enableSpring = true;
	jointDef.hertz = 2.0f;
	jointDef.dampingRatio = 1.0f;
	b3BodyId previous = ground;
	for ( int i = 0; i <= count; ++i )
	{
		b3BodyId body = ground;
		if ( i < count )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ -20.0f + 0.125f + 0.25f * i, 20.0f, 0.0f };
			bodyDef.linearDamping = 0.1f;
			bodyDef.angularDamping = 0.1f;
			body = b3CreateBody( worldId, &bodyDef );
			bodies[i] = body;
			b3CreateHullShape( body, &shapeDef, &box.base );
		}
		for ( int side = 0; side < 2; ++side )
		{
			b3Pos pivot = { -20.0f + 0.25f * i, 20.0f, side == 0 ? -0.5f : 0.5f };
			jointDef.base.bodyIdA = previous;
			jointDef.base.bodyIdB = body;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( previous, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( body, pivot );
			joints[2 * i + side] = b3CreateSphericalJoint( worldId, &jointDef );
		}
		previous = body;
	}
	float maxSeparation = 0.0f;
	float maxSpeed = 0.0f;
	for ( int step = 0; step < 600; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		for ( int i = 0; i < 2 * ( count + 1 ); ++i )
		{
			maxSeparation = b3MaxFloat( maxSeparation, b3Joint_GetLinearSeparation( joints[i] ) );
		}
		for ( int i = 0; i < count; ++i )
		{
			b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
			ENSURE( b3IsValidVec3( velocity ) );
			if ( step >= 480 )
			{
				maxSpeed = b3MaxFloat( maxSpeed, b3Length( velocity ) );
			}
		}
	}
	printf( "  bridge maxSeparation=%g settledSpeed=%g\n", maxSeparation, maxSpeed );
	ENSURE( maxSeparation < 0.005f );
	ENSURE( maxSpeed < 3.0f );
	b3DestroyWorld( worldId );
	return 0;
}

static int RunRagdollPileSleeping( float hertz )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId ground = b3CreateBody( worldId, &groundDef );
	b3BoxHull floor = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( ground, &shapeDef, &floor.base );
	enum { count = 12 };
	Human humans[count] = { 0 };
	for ( int i = 0; i < count; ++i )
	{
		b3Pos position = { ( i % 3 - 1 ) * 0.5f, 2.0f + 2.0f * ( i / 3 ), 0.0f };
		CreateHuman( humans + i, worldId, position, 5.0f, hertz, 0.5f, i + 1, NULL, false );
	}
	int sleepStep = 0;
	for ( ; sleepStep < 1800; ++sleepStep )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		if ( b3World_GetAwakeBodyCount( worldId ) == 0 )
		{
			break;
		}
	}
	printf( "  ragdoll pile hertz=%g sleepStep=%d awake=%d\n", hertz, sleepStep, b3World_GetAwakeBodyCount( worldId ) );
	ENSURE( sleepStep < 600 );
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	}
	b3DestroyWorld( worldId );
	return 0;
}

static int TestRagdollPileSleeping( void )
{
	ENSURE( RunRagdollPileSleeping( 1.0f ) == 0 );
	ENSURE( RunRagdollPileSleeping( 0.0f ) == 0 );
	return 0;
}

static int TestPrismaticGateCollision( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId ground = b3CreateBody( worldId, &bodyDef );
	b3BoxHull floor = b3MakeBoxHull( 5.0f, 0.5f, 5.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( ground, &shapeDef, &floor.base );
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 3.0f, 0.0f };
	b3BodyId gate = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.05f, 1.0f, 1.5f );
	b3CreateHullShape( gate, &shapeDef, &box.base );
	b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
	jointDef.base.bodyIdA = ground;
	jointDef.base.bodyIdB = gate;
	jointDef.base.collideConnected = true;
	jointDef.base.localFrameA.p = b3Body_GetLocalPoint( ground, bodyDef.position );
	jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3Vec3_axisY );
	jointDef.base.localFrameB.q = jointDef.base.localFrameA.q;
	jointDef.enableMotor = true;
	jointDef.motorSpeed = -2.0f;
	jointDef.maxMotorForce = 200.0f;
	b3JointId joint = b3CreatePrismaticJoint( worldId, &jointDef );
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3Pos position = b3Body_GetPosition( gate );
		ENSURE( position.y > 0.97f );
		ENSURE_SMALL( position.x, 0.01f );
		ENSURE_SMALL( position.z, 0.01f );
		ENSURE( b3Joint_GetAngularSeparation( joint ) < 0.01f );
	}
	b3DestroyWorld( worldId );
	return 0;
}

static int RunTwinChainGate( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId ground = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull floor = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3CreateHullShape( ground, &shapeDef, &floor.base );
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -1.25f, 8.6f, 0.0f };
	b3BodyId shaft = b3CreateBody( worldId, &bodyDef );
	b3BoxHull axle = b3MakeBoxHull( 0.2f, 0.2f, 1.5f );
	shapeDef.density = 100.0f;
	b3CreateHullShape( shaft, &shapeDef, &axle.base );
	b3RevoluteJointDef hingeDef = b3DefaultRevoluteJointDef();
	hingeDef.base.bodyIdA = ground;
	hingeDef.base.bodyIdB = shaft;
	hingeDef.base.localFrameA.p = b3Body_GetLocalPoint( ground, bodyDef.position );
	hingeDef.enableMotor = true;
	hingeDef.maxMotorTorque = 30000.0f;
	hingeDef.motorSpeed = -0.3f;
	b3JointId drive = b3CreateRevoluteJoint( worldId, &hingeDef );
	bodyDef.position = (b3Pos){ 0.0f, 1.5f, 0.0f };
	b3BodyId gate = b3CreateBody( worldId, &bodyDef );
	b3BoxHull door = b3MakeBoxHull( 0.05f, 1.5f, 1.95f );
	shapeDef.density = 0.5f;
	b3CreateHullShape( gate, &shapeDef, &door.base );
	b3PrismaticJointDef slideDef = b3DefaultPrismaticJointDef();
	slideDef.base.bodyIdA = ground;
	slideDef.base.bodyIdB = gate;
	slideDef.base.collideConnected = true;
	slideDef.base.localFrameA.p = b3Body_GetLocalPoint( ground, bodyDef.position );
	slideDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisX, b3Vec3_axisY );
	slideDef.base.localFrameB.q = slideDef.base.localFrameA.q;
	slideDef.enableMotor = true;
	slideDef.maxMotorForce = 200.0f;
	b3CreatePrismaticJoint( worldId, &slideDef );
	enum { links = 40 };
	b3JointId joints[2 * ( links + 1 )];
	b3BodyId bodies[2 * links];
	b3Capsule capsule = { { 0.0f, -0.07f, 0.0f }, { 0.0f, 0.07f, 0.0f }, 0.05f };
	shapeDef.density = 1.0f;
	for ( int side = 0; side < 2; ++side )
	{
		float depth = side == 0 ? -1.5f : 1.5f;
		b3BodyId previous = shaft;
		for ( int i = 0; i <= links; ++i )
		{
			b3BodyId body = gate;
			if ( i < links )
			{
				bodyDef.position = (b3Pos){ 0.0f, 8.6f - 0.07f - 0.14f * i, depth };
				body = b3CreateBody( worldId, &bodyDef );
				bodies[side * links + i] = body;
				b3CreateCapsuleShape( body, &shapeDef, &capsule );
			}
			b3Pos pivot = { 0.0f, 8.6f - 0.14f * i, depth };
			hingeDef.base.bodyIdA = previous;
			hingeDef.base.bodyIdB = body;
			hingeDef.base.localFrameA.p = b3Body_GetLocalPoint( previous, pivot );
			hingeDef.base.localFrameB.p = b3Body_GetLocalPoint( body, pivot );
			hingeDef.motorSpeed = 0.0f;
			hingeDef.maxMotorTorque = i == links ? 50.0f : 0.05f;
			joints[side * ( links + 1 ) + i] = b3CreateRevoluteJoint( worldId, &hingeDef );
			previous = body;
		}
	}
	float maxSeparation = 0.0f;
	float settledSpeed = 0.0f;
	for ( int step = 0; step < 600; ++step )
	{
		if ( step == 120 )
		{
			b3RevoluteJoint_SetMotorSpeed( drive, 0.3f );
		}
		if ( step == 240 )
		{
			b3RevoluteJoint_SetMotorSpeed( drive, 0.0f );
		}
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3Pos position = b3Body_GetPosition( gate );
		ENSURE( position.y > 1.45f );
		ENSURE_SMALL( position.x, 0.01f );
		ENSURE_SMALL( position.z, 0.01f );
		for ( int i = 0; i < 2 * ( links + 1 ); ++i )
		{
			maxSeparation = b3MaxFloat( maxSeparation, b3Joint_GetLinearSeparation( joints[i] ) );
		}
		for ( int i = 0; i < 2 * links; ++i )
		{
			b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
			ENSURE( b3IsValidVec3( velocity ) );
			if ( step >= 480 )
			{
				settledSpeed = b3MaxFloat( settledSpeed, b3Length( velocity ) );
			}
		}
	}
	printf( "  twin chain workers=%d maxSeparation=%g settledSpeed=%g\n", workerCount, maxSeparation, settledSpeed );
	ENSURE( maxSeparation < 0.01f );
	ENSURE( settledSpeed < 0.1f );
	b3DestroyWorld( worldId );
	return 0;
}

static int TestTwinChainGate( void )
{
	ENSURE( RunTwinChainGate( 1 ) == 0 );
	ENSURE( RunTwinChainGate( 4 ) == 0 );
	return 0;
}

static int TestSharedKinematicAnchor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = (b3Pos){ 0.0f, 10.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	b3BodyId anchor = b3CreateBody( worldId, &bodyDef );
	enum { chains = 24, links = 12 };
	b3JointId joints[chains * links];
	b3Sphere sphere = { b3Vec3_zero, 0.1f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.maskBits = 0;
	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.linearVelocity = b3Vec3_zero;
	for ( int chain = 0; chain < chains; ++chain )
	{
		b3BodyId previous = anchor;
		for ( int link = 0; link < links; ++link )
		{
			bodyDef.position = (b3Pos){ 0.5f + link, 10.0f, (float)chain };
			shapeDef.density = link + 1 == links ? 1000.0f : 1.0f;
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateSphereShape( body, &shapeDef, &sphere );
			b3Pos pivot = { (float)link, 10.0f, (float)chain };
			jointDef.base.bodyIdA = previous;
			jointDef.base.bodyIdB = body;
			jointDef.base.localFrameA.p = b3Body_GetLocalPoint( previous, pivot );
			jointDef.base.localFrameB.p = b3Body_GetLocalPoint( body, pivot );
			joints[chain * links + link] = b3CreateWeldJoint( worldId, &jointDef );
			previous = body;
		}
	}
	float maxSeparation = 0.0f;
	for ( int step = 0; step < 120; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		for ( int i = 0; i < chains * links; ++i )
		{
			maxSeparation = b3MaxFloat( maxSeparation, b3Joint_GetLinearSeparation( joints[i] ) );
		}
	}
	printf( "  kinematic chains maxSeparation=%g\n", maxSeparation );
	ENSURE( maxSeparation < 0.005f );
	ENSURE_SMALL( b3Body_GetLinearVelocity( anchor ).x - 1.0f, 0.0001f );
	b3DestroyWorld( worldId );
	return 0;
}

int JointRegressionTest( void )
{
	RUN_SUBTEST( TestTwinChainGate );
	RUN_SUBTEST( TestSuspensionBridge );
	RUN_SUBTEST( TestRagdollPileSleeping );
	RUN_SUBTEST( TestPrismaticGateCollision );
	RUN_SUBTEST( TestSharedKinematicAnchor );
	return 0;
}
