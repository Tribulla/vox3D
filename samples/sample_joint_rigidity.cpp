#include "gfx/draw.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <vector>

class JointRigidity : public Sample
{
public:
	enum Scene
	{
		sphericalChain,
		weldCantilever,
		weldChain,
		weldLattice,
		fixedLadder,
		cantilever,
		heavyPendulum,
		hingeChain,
		offsetSlider,
		motorLoad,
		massRatioChains,
		closedHingeLoop,
		bridgeImpact,
		sceneCount
	};

	JointRigidity( SampleContext* context, Scene scene )
		: Sample( context )
		, m_scene( scene )
	{
		bool labPort = scene >= fixedLadder && scene <= motorLoad;
		m_groundBody = AddGroundBox( labPort ? 30.0f : 50.0f );
		if ( labPort )
		{
			b3ShapeId groundShape;
			int shapeCount = b3Body_GetShapes( m_groundBody, &groundShape, 1 );
			B3_ASSERT( shapeCount == 1 );
			if ( shapeCount == 1 )
			{
				b3Shape_SetFriction( groundShape, 0.7f );
			}
		}
		switch ( scene )
		{
			case sphericalChain:
			case weldCantilever:
			case weldChain:
				CreateChain();
				break;
			case weldLattice:
				CreateLattice();
				break;
			case fixedLadder:
				CreateFixedLadder();
				break;
			case cantilever:
				CreateCantilever();
				break;
			case heavyPendulum:
				CreateHeavyPendulum();
				break;
			case hingeChain:
				CreateHingeChain();
				break;
			case offsetSlider:
				CreateOffsetSlider();
				break;
			case motorLoad:
				CreateMotorLoad();
				break;
			case massRatioChains:
				CreateMassRatioChains();
				break;
			case closedHingeLoop:
				CreateClosedHingeLoop();
				break;
			case bridgeImpact:
				CreateBridgeImpact();
				break;
			default:
				B3_ASSERT( false );
				break;
		}
		m_referenceLoadPosition = b3Body_GetPosition( m_loadBody );
		MeasureErrors();
		if ( context->restart == false )
		{
			if ( scene == sphericalChain )
			{
				m_camera->SetView( 30.0f, 15.0f, 38.0f, { 8.0f, 17.0f, 0.0f } );
			}
			else if ( scene == weldLattice )
			{
				m_camera->SetView( 40.0f, 25.0f, 9.0f, { 0.0f, 2.0f, 0.0f } );
			}
			else if ( scene == fixedLadder )
			{
				m_camera->SetView( 25.0f, 15.0f, 35.0f, { 0.0f, 11.0f, 0.0f } );
			}
			else if ( scene == cantilever )
			{
				m_camera->SetView( 8.0f, 10.0f, 26.0f, { 5.0f, 7.0f, 0.0f } );
			}
			else if ( scene == bridgeImpact )
			{
				m_camera->SetView( 40.0f, 18.0f, 48.0f, { 0.0f, 10.0f, 0.0f } );
			}
			else if ( scene >= fixedLadder )
			{
				m_camera->SetView( 20.0f, 11.0f, 28.0f, { 0.0f, 7.0f, 0.0f } );
			}
			else
			{
				m_camera->SetView( 30.0f, 20.0f, 22.0f, { 6.0f, 7.0f, 0.0f } );
			}
		}
	}

	b3BodyId AddBox( b3BodyType type, b3Pos position, b3Vec3 halfExtents, float density, float friction = 0.5f )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = type;
		bodyDef.position = position;
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = density;
		shapeDef.baseMaterial.friction = friction;
		b3BoxHull box = b3MakeBoxHull( halfExtents.x, halfExtents.y, halfExtents.z );
		b3CreateHullShape( body, &shapeDef, &box.base );
		return body;
	}

	b3BodyId AddSphere( b3BodyType type, b3Pos position, float radius, float density, float friction = 0.5f )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = type;
		bodyDef.position = position;
		b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = density;
		shapeDef.baseMaterial.friction = friction;
		b3Sphere sphere = { b3Vec3_zero, radius };
		b3CreateSphereShape( body, &shapeDef, &sphere );
		return body;
	}

	b3JointId ConnectLocal( b3BodyId a, b3BodyId b, b3JointType type, b3Vec3 anchorA, b3Vec3 anchorB,
							b3Quat rotation = b3Quat_identity )
	{
		b3JointDef base = b3DefaultSphericalJointDef().base;
		base.bodyIdA = a;
		base.bodyIdB = b;
		base.localFrameA = { anchorA, rotation };
		base.localFrameB = { anchorB, rotation };
		b3JointId joint = b3_nullJointId;
		switch ( type )
		{
			case b3_revoluteJoint:
			{
				b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
				def.base = base;
				joint = b3CreateRevoluteJoint( m_worldId, &def );
				break;
			}
			case b3_prismaticJoint:
			{
				b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
				def.base = base;
				joint = b3CreatePrismaticJoint( m_worldId, &def );
				break;
			}
			case b3_weldJoint:
			{
				b3WeldJointDef def = b3DefaultWeldJointDef();
				def.base = base;
				joint = b3CreateWeldJoint( m_worldId, &def );
				break;
			}
			case b3_sphericalJoint:
			{
				b3SphericalJointDef def = b3DefaultSphericalJointDef();
				def.base = base;
				joint = b3CreateSphericalJoint( m_worldId, &def );
				break;
			}
			default:
				B3_ASSERT( false );
				break;
		}
		m_joints.push_back( joint );
		return joint;
	}

	void Connect( b3BodyId a, b3BodyId b, b3Pos pivot )
	{
		b3JointDef base = b3DefaultSphericalJointDef().base;
		base.bodyIdA = a;
		base.bodyIdB = b;
		base.localFrameA.p = b3Body_GetLocalPoint( a, pivot );
		base.localFrameB.p = b3Body_GetLocalPoint( b, pivot );
		if ( m_scene == sphericalChain )
		{
			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			jointDef.base = base;
			m_joints.push_back( b3CreateSphericalJoint( m_worldId, &jointDef ) );
		}
		else
		{
			b3WeldJointDef jointDef = b3DefaultWeldJointDef();
			jointDef.base = base;
			m_joints.push_back( b3CreateWeldJoint( m_worldId, &jointDef ) );
		}
	}

	void CreateChain()
	{
		bool spherical = m_scene == sphericalChain;
		int count = spherical ? 24 : m_scene == weldCantilever ? 1 : 16;
		float halfLength = m_scene == weldCantilever ? 2.0f : 0.5f;
		float height = spherical ? 30.0f : 8.0f;
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = { -0.5f, height, 0.0f };
		b3BodyId parent = b3CreateBody( m_worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull anchorBox = b3MakeCubeHull( 0.5f );
		b3CreateHullShape( parent, &shapeDef, &anchorBox.base );
		bodyDef.type = b3_dynamicBody;
		shapeDef.density = 5.0f;
		b3BoxHull box = b3MakeBoxHull( halfLength, 0.125f, 0.125f );
		b3Capsule capsule = { { -halfLength, 0.0f, 0.0f }, { halfLength, 0.0f, 0.0f }, 0.125f };
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = { ( 2.0f * i + 1.0f ) * halfLength, height, 0.0f };
			b3BodyId body = b3CreateBody( m_worldId, &bodyDef );
			if ( spherical )
			{
				b3CreateCapsuleShape( body, &shapeDef, &capsule );
			}
			else
			{
				b3CreateHullShape( body, &shapeDef, &box.base );
			}
			Connect( parent, body, { 2.0f * i * halfLength, height, 0.0f } );
			parent = body;
		}
		float end = 2.0f * count * halfLength;
		bodyDef.position = { end + 0.75f, height, 0.0f };
		m_loadBody = b3CreateBody( m_worldId, &bodyDef );
		shapeDef.density = 50.0f;
		if ( spherical )
		{
			b3Sphere sphere = { b3Vec3_zero, 0.75f };
			b3CreateSphereShape( m_loadBody, &shapeDef, &sphere );
		}
		else
		{
			b3BoxHull payload = b3MakeCubeHull( 0.75f );
			b3CreateHullShape( m_loadBody, &shapeDef, &payload.base );
		}
		Connect( parent, m_loadBody, { end, height, 0.0f } );
	}

	void CreateLattice()
	{
		constexpr int count = 3;
		b3BodyId bodies[count][count][count];
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 10.0f;
		b3BoxHull box = b3MakeCubeHull( 0.45f );
		for ( int z = 0; z < count; ++z )
		{
			for ( int y = 0; y < count; ++y )
			{
				for ( int x = 0; x < count; ++x )
				{
					bodyDef.position = { x - 1.0f, 2.0f + y, z - 1.0f };
					bodies[x][y][z] = b3CreateBody( m_worldId, &bodyDef );
					b3CreateHullShape( bodies[x][y][z], &shapeDef, &box.base );
				}
			}
		}
		for ( int z = 0; z < count; ++z )
		{
			for ( int y = 0; y < count; ++y )
			{
				for ( int x = 0; x < count; ++x )
				{
					b3BodyId a = bodies[x][y][z];
					if ( x + 1 < count )
						Connect( a, bodies[x + 1][y][z], { x - 0.5f, 2.0f + y, z - 1.0f } );
					if ( y + 1 < count )
						Connect( a, bodies[x][y + 1][z], { x - 1.0f, 2.5f + y, z - 1.0f } );
					if ( z + 1 < count )
						Connect( a, bodies[x][y][z + 1], { x - 1.0f, 2.0f + y, z - 0.5f } );
				}
			}
		}
		m_loadBody = bodies[2][2][2];
	}

	void CreateFixedLadder()
	{
		constexpr int count = 12;
		constexpr float halfWidth = 4.0f;
		constexpr float firstY = 1.6f;
		constexpr float spacing = 1.8f;
		b3BodyId bodies[count];
		for ( int i = 0; i < count; ++i )
		{
			bodies[i] = AddBox( b3_dynamicBody, { 0.0f, firstY + spacing * i, 0.0f }, { halfWidth, 0.2f, 0.35f }, 1.0f, 0.7f );
		}
		b3Quat upAxis = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
		b3Pos pivot = { -halfWidth, firstY - 0.5f * spacing, 0.0f };
		ConnectLocal( m_groundBody, bodies[0], b3_revoluteJoint, b3Body_GetLocalPoint( m_groundBody, pivot ),
					  { -halfWidth, -0.5f * spacing, 0.0f }, upAxis );
		for ( int i = 0; i + 1 < count; ++i )
		{
			float side = ( i & 1 ) == 0 ? halfWidth : -halfWidth;
			ConnectLocal( bodies[i], bodies[i + 1], b3_revoluteJoint, { side, 0.5f * spacing, 0.0f },
						  { side, -0.5f * spacing, 0.0f }, upAxis );
		}
		m_loadBody = bodies[count - 1];
	}

	void CreateCantilever()
	{
		constexpr float hx = 0.35f;
		b3BodyId previous = AddBox( b3_staticBody, { 0.0f, 8.0f, 0.0f }, { 0.4f, 1.2f, 1.0f }, 0.0f );
		for ( int i = 0; i < 24; ++i )
		{
			b3BodyId beam =
				AddBox( b3_dynamicBody, { 0.4f + ( 2.0f * i + 1.0f ) * hx, 8.0f, 0.0f }, { hx, 0.14f, 0.18f }, 1.0f, 0.4f );
			ConnectLocal( previous, beam, b3_weldJoint, { i == 0 ? 0.4f : hx, 0.0f, 0.0f }, { -hx, 0.0f, 0.0f } );
			previous = beam;
		}
		m_loadBody = previous;
	}

	void CreateHeavyPendulum()
	{
		b3BodyId anchor = AddSphere( b3_staticBody, { 0.0f, 12.0f, 0.0f }, 0.3f, 0.0f );
		b3BodyId rod = AddBox( b3_dynamicBody, { 0.0f, 8.7f, 0.0f }, { 0.12f, 3.0f, 0.12f }, 0.15f );
		ConnectLocal( anchor, rod, b3_revoluteJoint, { 0.0f, -0.3f, 0.0f }, { 0.0f, 3.0f, 0.0f } );
		m_loadBody = AddSphere( b3_dynamicBody, { 0.0f, 4.85f, 0.0f }, 0.85f, 80.0f, 0.4f );
		ConnectLocal( rod, m_loadBody, b3_weldJoint, { 0.0f, -3.0f, 0.0f }, { 0.0f, 0.85f, 0.0f } );
		b3Body_SetLinearVelocity( m_loadBody, { 2.5f, 0.0f, 0.0f } );
	}

	void CreateHingeChain()
	{
		b3BodyId previous = AddBox( b3_staticBody, { 0.0f, 18.0f, 0.0f }, { 0.4f, 0.4f, 0.4f }, 0.0f );
		for ( int i = 0; i < 24; ++i )
		{
			b3BodyId body = AddBox( b3_dynamicBody, { 0.0f, 17.2f - 0.75f * i, 0.0f }, { 0.25f, 0.35f, 0.25f }, 1.0f );
			ConnectLocal( previous, body, b3_revoluteJoint, { 0.0f, i == 0 ? -0.4f : -0.35f, 0.0f }, { 0.0f, 0.35f, 0.0f } );
			previous = body;
		}
		m_loadBody = previous;
		b3Body_SetLinearVelocity( m_loadBody, { 3.0f, 0.0f, 0.0f } );
	}

	void CreateOffsetSlider()
	{
		b3BodyId rail = AddBox( b3_staticBody, { 0.0f, 7.0f, 0.0f }, { 8.0f, 0.1f, 0.1f }, 0.0f );
		m_loadBody = AddBox( b3_dynamicBody, { 0.0f, 7.0f, 0.0f }, { 0.7f, 0.2f, 0.7f }, 1.0f, 0.4f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 45.0f;
		shapeDef.baseMaterial.friction = 0.4f;
		b3BoxHull mass = b3MakeOffsetBoxHull( 0.65f, 0.65f, 0.65f, { 0.0f, -1.1f, 1.8f } );
		b3CreateHullShape( m_loadBody, &shapeDef, &mass.base );
		b3Body_SetLinearVelocity( m_loadBody, { 4.0f, 0.0f, 0.0f } );
		m_sliderJoint = ConnectLocal( rail, m_loadBody, b3_prismaticJoint, b3Vec3_zero, b3Vec3_zero );
		b3PrismaticJoint_SetLimits( m_sliderJoint, -7.0f, 7.0f );
		b3PrismaticJoint_EnableLimit( m_sliderJoint, true );
	}

	void CreateMotorLoad()
	{
		b3BodyId pivot = AddBox( b3_staticBody, { 0.0f, 8.0f, 0.0f }, { 0.35f, 0.35f, 0.35f }, 0.0f );
		b3BodyId arm = AddBox( b3_dynamicBody, { 3.2f, 8.0f, 0.0f }, { 3.2f, 0.15f, 0.15f }, 0.35f );
		m_motorJoint = ConnectLocal( pivot, arm, b3_revoluteJoint, { 0.35f, 0.0f, 0.0f }, { -3.2f, 0.0f, 0.0f } );
		b3RevoluteJoint_EnableMotor( m_motorJoint, m_enableMotor );
		b3RevoluteJoint_SetMaxMotorTorque( m_motorJoint, m_motorTorque );
		b3RevoluteJoint_SetMotorSpeed( m_motorJoint, m_motorSpeed );
		m_loadBody = AddSphere( b3_dynamicBody, { 6.4f, 8.0f, 0.0f }, 0.7f, 40.0f );
		ConnectLocal( arm, m_loadBody, b3_weldJoint, { 3.2f, 0.0f, 0.0f }, b3Vec3_zero );
	}

	void CreateMassRatioChains()
	{
		const float ratios[] = { 10.0f, 100.0f, 1000.0f };
		for ( int lane = 0; lane < 3; ++lane )
		{
			float z = 5.0f - 5.0f * lane;
			b3BodyId previous = AddBox( b3_staticBody, { 0.0f, 16.4f, z }, { 0.4f, 0.4f, 0.4f }, 0.0f );
			for ( int i = 0; i < 12; ++i )
			{
				b3BodyId body = AddBox( b3_dynamicBody, { 0.0f, 15.65f - 0.7f * i, z }, { 0.25f, 0.35f, 0.25f }, 1.0f );
				ConnectLocal( previous, body, b3_sphericalJoint, { 0.0f, i == 0 ? -0.4f : -0.35f, 0.0f }, { 0.0f, 0.35f, 0.0f } );
				previous = body;
			}
			// The one-cubic-meter payload weighs exactly ratio times a link (0.175 kg).
			b3BodyId payload = AddBox( b3_dynamicBody, { 0.0f, 7.1f, z }, { 0.5f, 0.5f, 0.5f }, 0.175f * ratios[lane] );
			ConnectLocal( previous, payload, b3_sphericalJoint, { 0.0f, -0.35f, 0.0f }, { 0.0f, 0.5f, 0.0f } );
			m_comparisonLoads.push_back( payload );
			m_loadBody = payload;
		}
	}

	void CreateClosedHingeLoop()
	{
		b3BodyId base = AddBox( b3_staticBody, { 0.0f, 8.0f, 0.0f }, { 2.0f, 0.15f, 0.35f }, 0.0f );
		b3BodyId left = AddBox( b3_dynamicBody, { -2.0f, 6.0f, 0.0f }, { 0.15f, 2.0f, 0.35f }, 2.0f );
		b3BodyId right = AddBox( b3_dynamicBody, { 2.0f, 6.0f, 0.0f }, { 0.15f, 2.0f, 0.35f }, 2.0f );
		m_loadBody = AddBox( b3_dynamicBody, { 0.0f, 4.0f, 0.0f }, { 2.0f, 0.15f, 0.35f }, 20.0f );
		ConnectLocal( base, left, b3_revoluteJoint, { -2.0f, 0.0f, 0.0f }, { 0.0f, 2.0f, 0.0f } );
		ConnectLocal( left, m_loadBody, b3_revoluteJoint, { 0.0f, -2.0f, 0.0f }, { -2.0f, 0.0f, 0.0f } );
		ConnectLocal( m_loadBody, right, b3_revoluteJoint, { 2.0f, 0.0f, 0.0f }, { 0.0f, -2.0f, 0.0f } );
		ConnectLocal( right, base, b3_revoluteJoint, { 0.0f, 2.0f, 0.0f }, { 2.0f, 0.0f, 0.0f } );
	}

	void CreateBridgeImpact()
	{
		constexpr int count = 64;
		constexpr float hx = 0.3f;
		constexpr float start = -count * hx;
		b3BodyId previous = AddBox( b3_staticBody, { start - hx, 12.0f, 0.0f }, { hx, 0.5f, 1.0f }, 0.0f );
		for ( int i = 0; i <= count; ++i )
		{
			b3BodyId body = AddBox( i == count ? b3_staticBody : b3_dynamicBody,
									{ start + ( 2.0f * i + 1.0f ) * hx, 12.0f, 0.0f }, { hx, 0.15f, 1.0f }, 10.0f, 0.6f );
			for ( float z : { -0.8f, 0.8f } )
			{
				ConnectLocal( previous, body, b3_sphericalJoint, { hx, 0.0f, z }, { -hx, 0.0f, z } );
			}
			if ( i == count / 2 )
			{
				m_loadBody = body;
			}
			previous = body;
		}
	}

	void LaunchImpact()
	{
		if ( B3_IS_NON_NULL( m_projectile ) )
		{
			b3DestroyBody( m_projectile );
		}
		b3Pos target = b3Body_GetPosition( m_loadBody );
		m_projectile = AddSphere( b3_dynamicBody, b3OffsetPos( target, { 0.0f, 6.0f, 0.0f } ), m_impactRadius, 30.0f, 0.4f );
		b3Body_SetLinearVelocity( m_projectile, { 0.0f, -m_impactSpeed, 0.0f } );
	}

	static float RotationError( b3Quat q )
	{
		return 2.0f * atan2f( b3Length( q.v ), b3AbsFloat( q.s ) );
	}

	void MeasureErrors()
	{
		m_linearError = 0.0f;
		m_angularError = 0.0f;
		for ( b3JointId joint : m_joints )
		{
			b3BodyId a = b3Joint_GetBodyA( joint );
			b3BodyId b = b3Joint_GetBodyB( joint );
			b3Transform localA = b3Joint_GetLocalFrameA( joint );
			b3Transform localB = b3Joint_GetLocalFrameB( joint );
			b3Pos pA = b3Body_GetWorldPoint( a, localA.p );
			b3Pos pB = b3Body_GetWorldPoint( b, localB.p );
			b3Quat qA = b3MulQuat( b3Body_GetRotation( a ), localA.q );
			b3Quat qB = b3MulQuat( b3Body_GetRotation( b ), localB.q );
			b3Vec3 dp = b3SubPos( pB, pA );
			float linear = b3Length( dp );
			float angular = 0.0f;
			switch ( b3Joint_GetType( joint ) )
			{
				case b3_revoluteJoint:
				{
					b3Vec3 axisA = b3RotateVector( qA, b3Vec3_axisZ );
					b3Vec3 axisB = b3RotateVector( qB, b3Vec3_axisZ );
					angular = atan2f( b3Length( b3Cross( axisA, axisB ) ), b3Dot( axisA, axisB ) );
					break;
				}
				case b3_prismaticJoint:
				{
					b3Vec3 axis = b3RotateVector( qA, b3Vec3_axisX );
					float translation = b3Dot( dp, axis );
					b3Vec3 perpendicular = b3MulAdd( dp, -translation, axis );
					float limitError = 0.0f;
					if ( b3PrismaticJoint_IsLimitEnabled( joint ) )
					{
						limitError = b3MaxFloat( 0.0f, b3PrismaticJoint_GetLowerLimit( joint ) - translation );
						limitError = b3MaxFloat( limitError, translation - b3PrismaticJoint_GetUpperLimit( joint ) );
					}
					linear = sqrtf( b3LengthSquared( perpendicular ) + limitError * limitError );
					angular = RotationError( b3InvMulQuat( qA, qB ) );
					break;
				}
				case b3_weldJoint:
					angular = RotationError( b3InvMulQuat( qA, qB ) );
					break;
				default:
					break;
			}
			m_linearError = b3MaxFloat( m_linearError, linear );
			m_angularError = b3MaxFloat( m_angularError, angular );
		}
	}

	b3Vec3 LoadDirection() const
	{
		return m_direction == 0	  ? b3Vec3{ 0.0f, -1.0f, 0.0f }
			   : m_direction == 1 ? b3Vec3{ 0.0f, 1.0f, 0.0f }
								  : b3Vec3{ 0.0f, 0.0f, 1.0f };
	}

	void Step() override
	{
		if ( m_holdLoad && m_context->hertz > 0.0f && ( m_context->pause == false || m_context->singleStep > 0 ) )
		{
			if ( m_comparisonLoads.empty() )
			{
				ApplyLoad( m_loadBody );
			}
			else
			{
				for ( b3BodyId body : m_comparisonLoads )
				{
					ApplyLoad( body );
				}
			}
		}
		Sample::Step();
		if ( m_didStep )
		{
			MeasureErrors();
			m_peakLinearError = b3MaxFloat( m_peakLinearError, m_linearError );
			m_peakAngularError = b3MaxFloat( m_peakAngularError, m_angularError );
		}
	}

	void ApplyLoad( b3BodyId body )
	{
		b3Body_ApplyForceToCenter( body, b3MulSV( b3Body_GetMass( body ) * m_loadAcceleration, LoadDirection() ), true );
	}

	void ApplyImpulse( b3BodyId body )
	{
		b3Pos point = b3Body_GetWorldPoint( body, { 0.0f, 0.5f, 0.0f } );
		b3Body_ApplyLinearImpulse( body, b3MulSV( b3Body_GetMass( body ) * m_impulseSpeed, LoadDirection() ), point, true );
	}

	void Render() override
	{
		if ( m_showPins == false )
		{
			return;
		}
		Vec4 color = MakeColor( b3_colorRed );
		for ( b3JointId joint : m_joints )
		{
			b3Pos a = b3Body_GetWorldPoint( b3Joint_GetBodyA( joint ), b3Joint_GetLocalFrameA( joint ).p );
			b3Pos b = b3Body_GetWorldPoint( b3Joint_GetBodyB( joint ), b3Joint_GetLocalFrameB( joint ).p );
			DrawPoint( a, 9.0f, color );
			if ( b3Joint_GetType( joint ) != b3_prismaticJoint && b3LengthSquared( b3SubPos( b, a ) ) > 1.0e-6f )
			{
				DrawPoint( b, 9.0f, color );
				DrawLine( a, b, color );
			}
		}
	}

	bool DrawControls() override
	{
		static const char* descriptions[] = {
			"A long spherical chain carrying a heavy end mass.",
			"A short fixed beam carrying a heavy end mass.",
			"A segmented fixed beam carrying a heavy end mass.",
			"A cube with redundant weld constraints resting on the ground.",
			"Lab port: twelve wide beams joined by alternating vertical-axis hinges.",
			"Lab port: twenty-four light welded segments extending from a wall.",
			"Lab port: a very light hinged rod carries a dense pendulum bob.",
			"Lab port: twenty-four hinged links with an initial sideways kick.",
			"Lab port: a slider with a heavy offset mass and travel limits at +/-7 m.",
			"Lab port: a motor resists a heavy bob on a long arm. Rotation under motor overload is expected.",
			"Identical chains with payload-to-link mass ratios of 10:1, 100:1 and 1000:1, from front to back. Loads affect all "
			"three.",
			"A four-bar hinge loop with a heavy bottom beam. Pull across its plane to test axis alignment and loop closure.",
			"A sixty-four-plank bridge supported along both edges. Launch repeatable impacts into its center." };
		ImGui::TextWrapped( "%s", descriptions[m_scene] );
		ImGui::TextWrapped( "Ctrl-drag to pull. Shift-click to launch an object." );
		ImGui::Checkbox( "Show joint pins", &m_showPins );
		ImGui::SliderFloat( "Gravity scale", &m_gravityScale, 0.0f, 5.0f, "%.1f" );
		b3World_SetGravity( m_worldId, { 0.0f, -10.0f * m_gravityScale, 0.0f } );
		ImGui::Combo( "Load direction", &m_direction, "Down\0Up\0Sideways\0" );
		ImGui::SliderFloat( "Load (m/s^2)", &m_loadAcceleration, 0.0f, 1000.0f, "%.0f" );
		ImGui::Checkbox( "Hold load", &m_holdLoad );
		ImGui::SliderFloat( "Impulse (m/s)", &m_impulseSpeed, 0.0f, 100.0f, "%.0f" );
		if ( ImGui::Button( "Apply impulse" ) )
		{
			if ( m_comparisonLoads.empty() )
			{
				ApplyImpulse( m_loadBody );
			}
			else
			{
				for ( b3BodyId body : m_comparisonLoads )
				{
					ApplyImpulse( body );
				}
			}
		}
		if ( m_scene == motorLoad )
		{
			ImGui::Separator();
			if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
			{
				b3RevoluteJoint_EnableMotor( m_motorJoint, m_enableMotor );
			}
			if ( ImGui::SliderFloat( "Max torque (Nm)", &m_motorTorque, 0.0f, 5000.0f, "%.0f" ) )
			{
				b3RevoluteJoint_SetMaxMotorTorque( m_motorJoint, m_motorTorque );
			}
			if ( ImGui::SliderFloat( "Motor speed (rad/s)", &m_motorSpeed, -10.0f, 10.0f, "%.1f" ) )
			{
				b3RevoluteJoint_SetMotorSpeed( m_motorJoint, m_motorSpeed );
			}
			ImGui::Text( "Motor torque: %.1f Nm", b3RevoluteJoint_GetMotorTorque( m_motorJoint ) );
		}
		if ( m_scene == offsetSlider )
		{
			ImGui::Text( "Slider travel: %.3f m", b3PrismaticJoint_GetTranslation( m_sliderJoint ) );
		}
		if ( m_scene == bridgeImpact )
		{
			ImGui::Separator();
			ImGui::SliderFloat( "Impact radius (m)", &m_impactRadius, 0.2f, 2.0f, "%.1f" );
			ImGui::SliderFloat( "Impact speed (m/s)", &m_impactSpeed, 10.0f, 200.0f, "%.0f" );
			if ( ImGui::Button( "Launch impact" ) )
			{
				LaunchImpact();
			}
		}
		ImGui::Separator();
		ImGui::Text( "Linear constraint error: %.3f mm", 1000.0f * m_linearError );
		ImGui::Text( "Peak: %.3f mm", 1000.0f * m_peakLinearError );
		if ( m_scene != sphericalChain && m_scene != massRatioChains && m_scene != bridgeImpact )
		{
			ImGui::Text( "Angle error: %.3f deg", B3_RAD_TO_DEG * m_angularError );
			ImGui::Text( "Peak: %.3f deg", B3_RAD_TO_DEG * m_peakAngularError );
		}
		ImGui::Text( "Load speed: %.2f m/s", b3Length( b3Body_GetLinearVelocity( m_loadBody ) ) );
		if ( m_scene == cantilever || m_scene == weldCantilever || m_scene == weldChain )
		{
			ImGui::Text( "Tip drop: %.3f mm", 1000.0f * ( m_referenceLoadPosition.y - b3Body_GetPosition( m_loadBody ).y ) );
		}
		if ( ImGui::Button( "Reset peaks" ) )
		{
			m_peakLinearError = m_linearError;
			m_peakAngularError = m_angularError;
		}
		return true;
	}

	static Sample* CreateSphericalChain( SampleContext* context )
	{
		return new JointRigidity( context, sphericalChain );
	}
	static Sample* CreateWeldCantilever( SampleContext* context )
	{
		return new JointRigidity( context, weldCantilever );
	}
	static Sample* CreateWeldChain( SampleContext* context )
	{
		return new JointRigidity( context, weldChain );
	}
	static Sample* CreateWeldLattice( SampleContext* context )
	{
		return new JointRigidity( context, weldLattice );
	}
	template <Scene scene> static Sample* Create( SampleContext* context )
	{
		return new JointRigidity( context, scene );
	}

	Scene m_scene;
	b3BodyId m_groundBody = b3_nullBodyId;
	b3BodyId m_loadBody = b3_nullBodyId;
	b3BodyId m_projectile = b3_nullBodyId;
	b3JointId m_sliderJoint = b3_nullJointId;
	b3JointId m_motorJoint = b3_nullJointId;
	b3Pos m_referenceLoadPosition = b3Pos_zero;
	std::vector<b3JointId> m_joints;
	std::vector<b3BodyId> m_comparisonLoads;
	float m_loadAcceleration = 100.0f;
	float m_impulseSpeed = 20.0f;
	float m_gravityScale = 1.0f;
	float m_linearError = 0.0f;
	float m_angularError = 0.0f;
	float m_peakLinearError = 0.0f;
	float m_peakAngularError = 0.0f;
	float m_motorTorque = 250.0f;
	float m_motorSpeed = 0.0f;
	float m_impactRadius = 0.6f;
	float m_impactSpeed = 60.0f;
	int m_direction = 2;
	bool m_holdLoad = false;
	bool m_enableMotor = true;
	bool m_showPins = true;
};

static int sphericalChain = RegisterSample( "Rigidity", "Spherical Chain", JointRigidity::CreateSphericalChain );
static int weldCantilever = RegisterSample( "Rigidity", "Weld Cantilever", JointRigidity::CreateWeldCantilever );
static int weldChain = RegisterSample( "Rigidity", "Weld Chain", JointRigidity::CreateWeldChain );
static int weldLattice = RegisterSample( "Rigidity", "Weld Lattice", JointRigidity::CreateWeldLattice );
static int fixedLadder = RegisterSample( "Rigidity", "Fixed Ladder", JointRigidity::Create<JointRigidity::fixedLadder> );
static int cantilever = RegisterSample( "Rigidity", "Cantilever", JointRigidity::Create<JointRigidity::cantilever> );
static int heavyPendulum = RegisterSample( "Rigidity", "Heavy Pendulum", JointRigidity::Create<JointRigidity::heavyPendulum> );
static int hingeChain = RegisterSample( "Rigidity", "Hinge Chain", JointRigidity::Create<JointRigidity::hingeChain> );
static int offsetSlider = RegisterSample( "Rigidity", "Offset Slider", JointRigidity::Create<JointRigidity::offsetSlider> );
static int motorLoad = RegisterSample( "Rigidity", "Motor Load", JointRigidity::Create<JointRigidity::motorLoad> );
static int massRatioChains =
	RegisterSample( "Rigidity", "Mass Ratio Chains", JointRigidity::Create<JointRigidity::massRatioChains> );
static int closedHingeLoop =
	RegisterSample( "Rigidity", "Closed Hinge Loop", JointRigidity::Create<JointRigidity::closedHingeLoop> );
static int bridgeImpact = RegisterSample( "Rigidity", "Bridge Impact", JointRigidity::Create<JointRigidity::bridgeImpact> );
