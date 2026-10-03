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
		weldLattice
	};

	JointRigidity( SampleContext* context, Scene scene )
		: Sample( context )
		, m_scene( scene )
	{
		AddGroundBox( 50.0f );
		if ( scene == weldLattice )
		{
			CreateLattice();
		}
		else
		{
			CreateChain();
		}
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
			else
			{
				m_camera->SetView( 30.0f, 20.0f, 22.0f, { 6.0f, 7.0f, 0.0f } );
			}
		}
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
			float mass = b3Body_GetMass( m_loadBody );
			b3Body_ApplyForceToCenter( m_loadBody, b3MulSV( mass * m_loadAcceleration, LoadDirection() ), true );
		}
		Sample::Step();
		if ( m_didStep )
		{
			m_linearError = 0.0f;
			m_angularError = 0.0f;
			for ( b3JointId joint : m_joints )
			{
				m_linearError = b3MaxFloat( m_linearError, b3Joint_GetLinearSeparation( joint ) );
				if ( m_scene != sphericalChain )
				{
					m_angularError = b3MaxFloat( m_angularError, b3Joint_GetAngularSeparation( joint ) );
				}
			}
			m_peakLinearError = b3MaxFloat( m_peakLinearError, m_linearError );
			m_peakAngularError = b3MaxFloat( m_peakAngularError, m_angularError );
		}
	}

	bool DrawControls() override
	{
		ImGui::TextWrapped( "Ctrl-drag to pull. Shift-click to launch an object." );
		ImGui::SliderFloat( "Gravity scale", &m_gravityScale, 0.0f, 5.0f, "%.1f" );
		b3World_SetGravity( m_worldId, { 0.0f, -10.0f * m_gravityScale, 0.0f } );
		ImGui::Combo( "Load direction", &m_direction, "Down\0Up\0Sideways\0" );
		ImGui::SliderFloat( "Load (m/s^2)", &m_loadAcceleration, 0.0f, 1000.0f, "%.0f" );
		ImGui::Checkbox( "Hold load", &m_holdLoad );
		ImGui::SliderFloat( "Impulse (m/s)", &m_impulseSpeed, 0.0f, 100.0f, "%.0f" );
		if ( ImGui::Button( "Apply impulse" ) )
		{
			b3Pos point = b3Body_GetWorldPoint( m_loadBody, { 0.0f, 0.5f, 0.0f } );
			b3Body_ApplyLinearImpulse( m_loadBody, b3MulSV( b3Body_GetMass( m_loadBody ) * m_impulseSpeed, LoadDirection() ),
									   point, true );
		}
		ImGui::Separator();
		ImGui::Text( "Anchor error: %.3f mm", 1000.0f * m_linearError );
		ImGui::Text( "Peak: %.3f mm", 1000.0f * m_peakLinearError );
		if ( m_scene != sphericalChain )
		{
			ImGui::Text( "Angle error: %.3f deg", B3_RAD_TO_DEG * m_angularError );
			ImGui::Text( "Peak: %.3f deg", B3_RAD_TO_DEG * m_peakAngularError );
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

	Scene m_scene;
	b3BodyId m_loadBody = b3_nullBodyId;
	std::vector<b3JointId> m_joints;
	float m_loadAcceleration = 100.0f;
	float m_impulseSpeed = 20.0f;
	float m_gravityScale = 1.0f;
	float m_linearError = 0.0f;
	float m_angularError = 0.0f;
	float m_peakLinearError = 0.0f;
	float m_peakAngularError = 0.0f;
	int m_direction = 2;
	bool m_holdLoad = false;
};

static int sphericalChain = RegisterSample( "Rigidity", "Spherical Chain", JointRigidity::CreateSphericalChain );
static int weldCantilever = RegisterSample( "Rigidity", "Weld Cantilever", JointRigidity::CreateWeldCantilever );
static int weldChain = RegisterSample( "Rigidity", "Weld Chain", JointRigidity::CreateWeldChain );
static int weldLattice = RegisterSample( "Rigidity", "Weld Lattice", JointRigidity::CreateWeldLattice );
