#include "box3d/box3d.h"
#include "test_macros.h"

#include <stdio.h>

static int RunPullRecovery( const char* name, b3WorldId worldId, const b3BodyId* bodies, int bodyCount,
							const b3JointId* joints, int jointCount, int grabbedIndex, b3Vec3 localGrabPoint )
{
	b3BodyId target = b3_nullBodyId;
	b3JointId mouseJoint = b3_nullJointId;
	b3Pos grabPoint = b3Pos_zero;
	b3BodyId grabbedBody = bodies[grabbedIndex];
	b3MassData massData = b3Body_GetMassData( grabbedBody );
	float maxSeparation = 0.0f;
	float maxSpeed = 0.0f;
	float recoveredSeparation = 0.0f;
	float dragSeparation = 0.0f;
	float forceSeparation = 0.0f;
	int dragPeakStep = 0;
	float recoveryPeaks[3] = { 0 };
	int recoveryPeakSteps[3] = { 0 };
	double phaseTime[4] = { 0 };
	double phaseSpeed[4] = { 0 };
	int phaseSteps[4] = { 0 };
	for ( int step = 0; step < 1440; ++step )
	{
		if ( step == 120 )
		{
			grabPoint = b3Body_GetWorldPoint( grabbedBody, localGrabPoint );
			b3BodyDef targetDef = b3DefaultBodyDef();
			targetDef.type = b3_kinematicBody;
			targetDef.position = grabPoint;
			target = b3CreateBody( worldId, &targetDef );
			b3MotorJointDef mouseDef = b3DefaultMotorJointDef();
			mouseDef.base.bodyIdA = target;
			mouseDef.base.bodyIdB = grabbedBody;
			mouseDef.base.localFrameB.p = localGrabPoint;
			mouseDef.linearHertz = 7.5f;
			mouseDef.linearDampingRatio = 1.0f;
			mouseDef.maxSpringForce = 100.0f * massData.mass * 10.0f;
			float trace = massData.inertia.cx.x + massData.inertia.cy.y + massData.inertia.cz.z;
			mouseDef.maxVelocityTorque = 0.5f * sqrtf( trace / ( 3.0f * massData.mass ) ) * massData.mass * 10.0f;
			mouseJoint = b3CreateMotorJoint( worldId, &mouseDef );
		}
		if ( step >= 120 && step < 240 )
		{
			b3Vec3 offset = step < 180 ? (b3Vec3){ 20.0f, 20.0f, 15.0f } : (b3Vec3){ -20.0f, -10.0f, -15.0f };
			b3WorldTransform transform = { b3OffsetPos( grabPoint, offset ), b3Quat_identity };
			b3Body_SetTargetTransform( target, transform, 1.0f / 60.0f, true );
		}
		if ( step == 240 )
		{
			b3DestroyJoint( mouseJoint, true );
			b3DestroyBody( target );
		}
		if ( step >= 480 && step < 540 )
		{
			b3Body_ApplyForceToCenter( grabbedBody, b3MulSV( massData.mass, (b3Vec3){ 10000.0f, 5000.0f, -3000.0f } ), true );
		}
		if ( step == 840 )
		{
			b3BodyId middle = bodies[bodyCount / 2];
			b3Pos position = b3Body_GetPosition( middle );
			position.y += 8.0f;
			position.z += 6.0f;
			b3Body_SetTransform( middle, position, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 1.7f ) );
		}
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		int loadPhase = step < 120 ? 0 : step < 240 ? 1 : step >= 480 && step < 540 ? 2 : 3;
		phaseTime[loadPhase] += b3World_GetProfile( worldId ).step;
		phaseSpeed[loadPhase] += b3Length( b3Body_GetLinearVelocity( grabbedBody ) );
		phaseSteps[loadPhase] += 1;
		float separation = 0.0f;
		for ( int i = 0; i < jointCount; ++i )
		{
			float error = b3Joint_GetLinearSeparation( joints[i] );
			ENSURE( b3IsValidFloat( error ) );
			separation = b3MaxFloat( separation, error );
		}
		maxSeparation = b3MaxFloat( maxSeparation, separation );
		if ( step >= 120 && step < 240 )
		{
			if ( separation > dragSeparation )
			{
				dragSeparation = separation;
				dragPeakStep = step;
			}
		}
		if ( step >= 480 && step < 540 )
		{
			forceSeparation = b3MaxFloat( forceSeparation, separation );
		}
		for ( int i = 0; i < bodyCount; ++i )
		{
			ENSURE( b3IsValidPosition( b3Body_GetPosition( bodies[i] ) ) );
			ENSURE( b3IsValidQuat( b3Body_GetRotation( bodies[i] ) ) );
			b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
			ENSURE( b3IsValidVec3( velocity ) );
			ENSURE( b3IsValidVec3( b3Body_GetAngularVelocity( bodies[i] ) ) );
			maxSpeed = b3MaxFloat( maxSpeed, b3Length( velocity ) );
		}
		if ( ( step >= 420 && step < 480 ) || ( step >= 780 && step < 840 ) || step >= 1380 )
		{
			recoveredSeparation = b3MaxFloat( recoveredSeparation, separation );
			int phase = step < 480 ? 0 : step < 840 ? 1 : 2;
			if ( separation > recoveryPeaks[phase] )
			{
				recoveryPeaks[phase] = separation;
				recoveryPeakSteps[phase] = step;
			}
		}
		if ( step == 119 || step == 239 || step == 479 || step == 539 || step == 839 || step == 1439 )
		{
			printf( "  %s step=%d separation=%g peakSpeed=%g\n", name, step, separation, maxSpeed );
			fflush( stdout );
		}
	}
	printf( "  %s maxSeparation=%g recoveredSeparation=%g\n", name, maxSeparation, recoveredSeparation );
	printf( "  %s dragSeparation=%g step=%d forceSeparation=%g\n", name, dragSeparation, dragPeakStep, forceSeparation );
	for ( int i = 0; i < 3; ++i )
	{
		printf( "  %s recovery phase=%d peak=%g step=%d\n", name, i, recoveryPeaks[i], recoveryPeakSteps[i] );
	}
	for ( int i = 0; i < 4; ++i )
	{
		printf( "  %s load phase=%d meanStepMs=%g meanPayloadSpeed=%g\n", name, i,
				phaseTime[i] / phaseSteps[i], phaseSpeed[i] / phaseSteps[i] );
	}
	ENSURE( recoveredSeparation < 0.005f );
	ENSURE( dragSeparation < 0.01f );
	ENSURE( forceSeparation < 0.02f );
	b3DestroyWorld( worldId );
	return 0;
}

static int RunBallAndChainPullRecovery( bool friction )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( worldId, &bodyDef );
	enum { linkCount = 32 };
	b3BodyId bodies[linkCount + 1];
	b3JointId joints[linkCount + 1];
	b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.125f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	jointDef.enableMotor = friction;
	jointDef.maxMotorTorque = 10.0f;
	jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
	bodyDef.type = b3_dynamicBody;
	b3BodyId parent = ground;
	for ( int i = 0; i < linkCount; ++i )
	{
		bodyDef.position = (b3Pos){ 0.5f + i, 0.0f, 0.0f };
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateCapsuleShape( bodies[i], &shapeDef, &capsule );
		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = bodies[i];
		joints[i] = b3CreateSphericalJoint( worldId, &jointDef );
		jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		parent = bodies[i];
	}
	b3Sphere sphere = { b3Vec3_zero, 2.0f };
	bodyDef.position = (b3Pos){ 34.0f, 0.0f, 0.0f };
	bodies[linkCount] = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( bodies[linkCount], &shapeDef, &sphere );
	jointDef.base.bodyIdA = parent;
	jointDef.base.bodyIdB = bodies[linkCount];
	jointDef.base.localFrameB.p = (b3Vec3){ -2.0f, 0.0f, 0.0f };
	joints[linkCount] = b3CreateSphericalJoint( worldId, &jointDef );
	return RunPullRecovery( friction ? "ball chain" : "frictionless ball chain", worldId, bodies, linkCount + 1,
							joints, linkCount + 1, linkCount,
							(b3Vec3){ 0.0f, 1.5f, 0.0f } );
}

static int TestBallAndChainPullRecovery( void )
{
	ENSURE( RunBallAndChainPullRecovery( true ) == 0 );
	ENSURE( RunBallAndChainPullRecovery( false ) == 0 );
	return 0;
}

static int RunBridgeImpactRecovery( b3WorldId worldId, const b3BodyId* bodies, int bodyCount,
								   const b3JointId* joints, int jointCount )
{
	enum { shotCount = 3 };
	b3ShapeId projectiles[shotCount] = { 0 };
	bool hits[shotCount] = { false };
	float maxSeparation[shotCount] = { 0 };
	float recoveredSeparation[shotCount] = { 0 };
	float maxSpeed = 0.0f;
	for ( int step = 0; step < 1200; ++step )
	{
		if ( step == 120 || step == 300 || step == 480 )
		{
			int shot = ( step - 120 ) / 180;
			b3BodyId plank = bodies[bodyCount / 2 + 10 * ( shot - 1 )];
			b3Pos position = b3Body_GetPosition( plank );
			float direction = shot == 1 ? 1.0f : -1.0f;
			position.z -= 3.0f * direction;
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = position;
			float speed = shot == 0 ? 20.0f : shot == 1 ? 100.0f : 400.0f;
			bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, speed * direction };
			bodyDef.isBullet = true;
			b3BodyId projectile = b3CreateBody( worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 4.0f;
			shapeDef.enableHitEvents = true;
			b3Sphere sphere = { b3Vec3_zero, shot == 0 ? 0.25f : 2.0f };
			projectiles[shot] = b3CreateSphereShape( projectile, &shapeDef, &sphere );
		}
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3ContactEvents events = b3World_GetContactEvents( worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			for ( int shot = 0; shot < shotCount; ++shot )
			{
				b3ContactHitEvent* hit = events.hitEvents + i;
				b3ShapeId other = b3_nullShapeId;
				if ( B3_ID_EQUALS( projectiles[shot], hit->shapeIdA ) )
				{
					other = hit->shapeIdB;
				}
				else if ( B3_ID_EQUALS( projectiles[shot], hit->shapeIdB ) )
				{
					other = hit->shapeIdA;
				}
				if ( B3_IS_NON_NULL( other ) )
				{
					b3BodyId hitBody = b3Shape_GetBody( other );
					for ( int k = 0; k < bodyCount; ++k )
					{
						hits[shot] |= B3_ID_EQUALS( hitBody, bodies[k] );
					}
				}
			}
		}
		float separation = 0.0f;
		for ( int i = 0; i < jointCount; ++i )
		{
			float error = b3Joint_GetLinearSeparation( joints[i] );
			ENSURE( b3IsValidFloat( error ) );
			separation = b3MaxFloat( separation, error );
		}
		if ( step >= 120 )
		{
			int shot = b3MinInt( ( step - 120 ) / 180, shotCount - 1 );
			maxSeparation[shot] = b3MaxFloat( maxSeparation[shot], separation );
			if ( step >= 1140 || ( step >= 240 && step < 300 ) || ( step >= 420 && step < 480 ) )
			{
				recoveredSeparation[shot] = b3MaxFloat( recoveredSeparation[shot], separation );
			}
		}
		for ( int i = 0; i < bodyCount; ++i )
		{
			ENSURE( b3IsValidPosition( b3Body_GetPosition( bodies[i] ) ) );
			ENSURE( b3IsValidQuat( b3Body_GetRotation( bodies[i] ) ) );
			b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
			ENSURE( b3IsValidVec3( velocity ) );
			ENSURE( b3IsValidVec3( b3Body_GetAngularVelocity( bodies[i] ) ) );
			maxSpeed = b3MaxFloat( maxSpeed, b3Length( velocity ) );
		}
	}
	const float separationLimits[shotCount] = { 0.005f, 0.01f, 0.01f };
	for ( int shot = 0; shot < shotCount; ++shot )
	{
		printf( "  bridge impact=%d hit=%d maxSeparation=%g recoveredSeparation=%g peakSpeed=%g\n",
				shot, hits[shot], maxSeparation[shot], recoveredSeparation[shot], maxSpeed );
		ENSURE( hits[shot] );
		ENSURE( maxSeparation[shot] < separationLimits[shot] );
		ENSURE( recoveredSeparation[shot] < 0.005f );
	}
	b3DestroyWorld( worldId );
	return 0;
}

static int RunBridgeRecovery( bool impacts )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	worldDef.workerCount = 4;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( worldId, &groundDef );
	enum { count = 150 };
	b3BodyId bodies[count];
	b3JointId joints[2 * ( count + 1 )];
	b3BoxHull box = b3MakeBoxHull( 0.125f, 0.125f, 0.5f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20.0f;
	groundDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId floor = b3CreateBody( worldId, &groundDef );
	b3BoxHull floorHull = b3MakeBoxHull( 60.0f, 1.0f, 60.0f );
	b3CreateHullShape( floor, &shapeDef, &floorHull.base );
	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	jointDef.base.constraintHertz = 1000.0f;
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
	if ( impacts )
	{
		return RunBridgeImpactRecovery( worldId, bodies, count, joints, 2 * ( count + 1 ) );
	}
	return RunPullRecovery( "bridge", worldId, bodies, count, joints, 2 * ( count + 1 ), count / 2,
							(b3Vec3){ 0.1f, 0.0f, 0.4f } );
}

static int TestBridgeImpactRecovery( void )
{
	return RunBridgeRecovery( true );
}

static int TestBridgePullRecovery( void )
{
	return RunBridgeRecovery( false );
}

static int TestRecoveryMotionLocks( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.motionLocks = (b3MotionLocks){ true, true, true, true, true, true };
	b3BodyId locked = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeCubeHull( 0.5f );
	b3CreateHullShape( locked, &shapeDef, &box.base );
	bodyDef.motionLocks = (b3MotionLocks){ 0 };
	bodyDef.position = (b3Pos){ 2.0f, 2.0f, 2.0f };
	b3BodyId freeBody = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( freeBody, &shapeDef, &box.base );
	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	jointDef.base.bodyIdA = locked;
	jointDef.base.bodyIdB = freeBody;
	jointDef.base.localFrameB.p = (b3Vec3){ -2.0f, -2.0f, -2.0f };
	b3JointId joint = b3CreateWeldJoint( worldId, &jointDef );
	b3Body_SetTransform( freeBody, (b3Pos){ 3.0f, 3.0f, 3.0f }, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f ) );
	b3Body_SetLinearVelocity( freeBody, (b3Vec3){ 1000.0f, 500.0f, -300.0f } );
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3Pos position = b3Body_GetPosition( locked );
		b3Quat rotation = b3Body_GetRotation( locked );
		ENSURE( position.x == 0.0f && position.y == 0.0f && position.z == 0.0f );
		ENSURE( rotation.v.x == 0.0f && rotation.v.y == 0.0f && rotation.v.z == 0.0f && rotation.s == 1.0f );
		ENSURE( b3IsValidPosition( b3Body_GetPosition( freeBody ) ) );
	}
	ENSURE( b3Joint_GetLinearSeparation( joint ) < 0.005f );
	ENSURE( b3Joint_GetAngularSeparation( joint ) < 0.005f );
	b3DestroyWorld( worldId );
	return 0;
}

static int RunChainSpeedLimitTranslation( int linkCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	enum { maximumLinks = 400 };
	ENSURE( linkCount <= maximumLinks );
	b3BodyId bodies[maximumLinks + 1];
	b3JointId joints[maximumLinks];
	float masses[maximumLinks + 1];
	float totalMass = 0.0f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, 100.0f, 0.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Capsule capsule = { { -0.5f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, 0.125f };
	for ( int i = 0; i <= linkCount; ++i )
	{
		bodyDef.position = (b3Pos){ i == linkCount ? linkCount + 2.0f : i + 0.5f, 0.0f, 0.0f };
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		if ( i == linkCount )
		{
			b3Sphere sphere = { b3Vec3_zero, 2.0f };
			b3CreateSphereShape( bodies[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodies[i], &shapeDef, &capsule );
		}
		masses[i] = b3Body_GetMass( bodies[i] );
		totalMass += masses[i];
		if ( i > 0 )
		{
			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			jointDef.base.bodyIdA = bodies[i - 1];
			jointDef.base.bodyIdB = bodies[i];
			jointDef.base.localFrameA.p = (b3Vec3){ 0.5f, 0.0f, 0.0f };
			jointDef.base.localFrameB.p = (b3Vec3){ i == linkCount ? -2.0f : -0.5f, 0.0f, 0.0f };
			joints[i - 1] = b3CreateSphericalJoint( worldId, &jointDef );
		}
	}
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	b3BodyId target = b3CreateBody( worldId, &bodyDef );
	b3MotorJointDef motorDef = b3DefaultMotorJointDef();
	motorDef.base.bodyIdA = target;
	motorDef.base.bodyIdB = bodies[linkCount / 2];
	motorDef.maxVelocityForce = 0.0f;
	motorDef.maxSpringForce = 0.0f;
	motorDef.maxSpringTorque = 0.0f;
	motorDef.maxVelocityTorque = 10000.0f;
	motorDef.angularVelocity = (b3Vec3){ 800.0f, 0.0f, 0.0f };
	b3CreateMotorJoint( worldId, &motorDef );
	float minimumSpeed = 100.0f;
	float separation = 0.0f;
	for ( int step = 0; step < 60; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		float momentumY = 0.0f;
		for ( int i = 0; i <= linkCount; ++i )
		{
			b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
			ENSURE( b3IsValidVec3( velocity ) );
			momentumY += masses[i] * velocity.y;
		}
		minimumSpeed = b3MinFloat( minimumSpeed, momentumY / totalMass );
		for ( int i = 0; i < linkCount; ++i )
		{
			separation = b3MaxFloat( separation, b3Joint_GetLinearSeparation( joints[i] ) );
		}
	}
	printf( "  chain speed limit links=%d minimumTranslation=%g separation=%g\n", linkCount, minimumSpeed, separation );
	b3DestroyWorld( worldId );
	ENSURE( minimumSpeed > 99.9f );
	ENSURE( separation < 0.005f );
	return 0;
}

static int TestChainSpeedLimitPreservesTranslation( void )
{
	ENSURE( RunChainSpeedLimitTranslation( 8 ) == 0 );
	ENSURE( RunChainSpeedLimitTranslation( 400 ) == 0 );
	return 0;
}

int JointRecoveryTest( void )
{
	RUN_SUBTEST( TestChainSpeedLimitPreservesTranslation );
	RUN_SUBTEST( TestRecoveryMotionLocks );
	RUN_SUBTEST( TestBallAndChainPullRecovery );
	RUN_SUBTEST( TestBridgeImpactRecovery );
	RUN_SUBTEST( TestBridgePullRecovery );
	return 0;
}
