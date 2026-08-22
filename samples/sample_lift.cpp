#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <vector>

// Helper to create an axis-aligned box hull at an arbitrary local center offset
static b3HullData* CreateBoxHullOffset( b3Vec3 halfExtents, b3Vec3 center )
{
	b3Vec3 pts[8];
	int idx = 0;
	for ( float sx : { -1.0f, 1.0f } )
	{
		for ( float sy : { -1.0f, 1.0f } )
		{
			for ( float sz : { -1.0f, 1.0f } )
			{
				pts[idx++] = { center.x + sx * halfExtents.x, center.y + sy * halfExtents.y,
							   center.z + sz * halfExtents.z };
			}
		}
	}
	return b3CreateHull( pts, 8, 8 );
}

// Helper to create an airplane wing hull with span along X, chord along Z (nose at -Z, tail at +Z),
// thickness and camber along Y, and dihedral angle (wingtips elevated).
static b3HullData* CreateGliderWingHull( float span, float chord, float thicknessRatio, float camberRatio,
										float dihedralDeg, b3Vec3 center )
{
	enum
	{
		kProfilePoints = 10,
		kSpanStations = 6
	};

	float dihedralRad = dihedralDeg * B3_PI / 180.0f;
	std::vector<b3Vec3> allPoints;
	allPoints.reserve( kProfilePoints * 2 * kSpanStations );

	for ( int s = 0; s < kSpanStations; ++s )
	{
		float u = (float)s / (float)( kSpanStations - 1 );
		float x = ( u - 0.5f ) * span;
		float yDihedral = fabsf( x ) * sinf( dihedralRad );

		for ( int i = 0; i < kProfilePoints; ++i )
		{
			float t = (float)i / (float)( kProfilePoints - 1 );
			// Cosine spacing from leading edge (t=0, z=-chord/2) to trailing edge (t=1, z=+chord/2)
			float zFrac = 0.5f * ( 1.0f - cosf( t * B3_PI ) );
			float zLocal = ( zFrac - 0.5f ) * chord;

			float xNorm = zFrac;
			// NACA thickness profile
			float yt = 5.0f * thicknessRatio * chord *
					   ( 0.2969f * sqrtf( xNorm ) - 0.1260f * xNorm - 0.3516f * xNorm * xNorm +
						 0.2843f * xNorm * xNorm * xNorm - 0.1015f * xNorm * xNorm * xNorm * xNorm );
			// Camber line
			float yc = 4.0f * camberRatio * chord * xNorm * ( 1.0f - xNorm );

			float yUpper = center.y + yDihedral + yc + yt;
			float yLower = center.y + yDihedral + yc - yt;
			float zPos = center.z + zLocal;
			float xPos = center.x + x;

			allPoints.push_back( { xPos, yUpper, zPos } );
			allPoints.push_back( { xPos, yLower, zPos } );
		}
	}

	return b3CreateHull( allPoints.data(), (int)allPoints.size(), 32 );
}

// Helper to create a swept-back vertical stabilizer fin
static b3HullData* CreateSweptVerticalFinHull( float baseLength, float tipLength, float height, float sweepBack,
											  float thickness, b3Vec3 rootCenter )
{
	b3Vec3 pts[8];
	float halfT = 0.5f * thickness;

	float zBaseFront = rootCenter.z - 0.5f * baseLength;
	float zBaseRear = rootCenter.z + 0.5f * baseLength;
	float zTipFront = zBaseFront + sweepBack;
	float zTipRear = zTipFront + tipLength;
	float yBase = rootCenter.y;
	float yTip = rootCenter.y + height;

	int idx = 0;
	for ( float signX : { -1.0f, 1.0f } )
	{
		float x = rootCenter.x + signX * halfT;
		pts[idx++] = { x, yBase, zBaseFront };
		pts[idx++] = { x, yBase, zBaseRear };
		pts[idx++] = { x, yTip, zTipFront };
		pts[idx++] = { x, yTip, zTipRear };
	}

	return b3CreateHull( pts, 8, 8 );
}

// Helper to create swept delta fins for darts at the tail end
static b3HullData* CreateDartDeltaFins( float xFront, float xRear, float finSpan, float thickness, bool vertical )
{
	b3Vec3 pts[12];
	float halfT = 0.5f * thickness;
	int idx = 0;

	if ( vertical )
	{
		// Top fin (+Y) and bottom fin (-Y)
		for ( float signZ : { -1.0f, 1.0f } )
		{
			float z = signZ * halfT;
			// Top fin
			pts[idx++] = { xFront, 0.08f, z };
			pts[idx++] = { xRear, 0.08f, z };
			pts[idx++] = { xRear, 0.08f + finSpan, z };
			// Bottom fin
			pts[idx++] = { xFront, -0.08f, z };
			pts[idx++] = { xRear, -0.08f, z };
			pts[idx++] = { xRear, -0.08f - finSpan, z };
		}
	}
	else
	{
		// Right fin (+Z) and left fin (-Z)
		for ( float signY : { -1.0f, 1.0f } )
		{
			float y = signY * halfT;
			// Right fin
			pts[idx++] = { xFront, y, 0.08f };
			pts[idx++] = { xRear, y, 0.08f };
			pts[idx++] = { xRear, y, 0.08f + finSpan };
			// Left fin
			pts[idx++] = { xFront, y, -0.08f };
			pts[idx++] = { xRear, y, -0.08f };
			pts[idx++] = { xRear, y, -0.08f - finSpan };
		}
	}

	return b3CreateHull( pts, idx, 16 );
}

// Helper to create a single radial pitched rotor blade in the hub local space
static b3HullData* CreateRadialRotorBladeHull( float innerRadius, float outerRadius, float chord, float pitchDeg,
											  float azimuthRad )
{
	enum
	{
		kRadialStations = 5,
		kChordPoints = 6
	};

	float pitchRad = pitchDeg * B3_PI / 180.0f;
	float sinA = sinf( azimuthRad );
	float cosA = cosf( azimuthRad );

	// Radial direction
	b3Vec3 uR = { sinA, 0.0f, cosA };
	// Chord / tangent direction (in direction of rotation around +Y)
	b3Vec3 uC = { cosA, 0.0f, -sinA };
	// Up direction
	b3Vec3 uY = { 0.0f, 1.0f, 0.0f };

	// Pitched chord and thickness directions
	b3Vec3 vChord = { cosf( pitchRad ) * uC.x, sinf( pitchRad ), cosf( pitchRad ) * uC.z };
	b3Vec3 vThick = { -sinf( pitchRad ) * uC.x, cosf( pitchRad ), -sinf( pitchRad ) * uC.z };

	std::vector<b3Vec3> pts;
	pts.reserve( kRadialStations * kChordPoints * 2 );

	for ( int rs = 0; rs < kRadialStations; ++rs )
	{
		float u = (float)rs / (float)( kRadialStations - 1 );
		float r = innerRadius + u * ( outerRadius - innerRadius );
		b3Vec3 rootP = { r * uR.x, 0.0f, r * uR.z };

		for ( int cp = 0; cp < kChordPoints; ++cp )
		{
			float t = (float)cp / (float)( kChordPoints - 1 );
			float s = ( t - 0.5f ) * chord;
			float xNorm = t;

			// Thickness
			float yt = 5.0f * 0.10f * chord *
					   ( 0.2969f * sqrtf( xNorm ) - 0.1260f * xNorm - 0.3516f * xNorm * xNorm +
						 0.2843f * xNorm * xNorm * xNorm - 0.1015f * xNorm * xNorm * xNorm * xNorm );
			float yc = 4.0f * 0.02f * chord * xNorm * ( 1.0f - xNorm );

			float upper = yc + yt;
			float lower = yc - yt;

			b3Vec3 pUpper = { rootP.x + s * vChord.x + upper * vThick.x,
							  rootP.y + s * vChord.y + upper * vThick.y,
							  rootP.z + s * vChord.z + upper * vThick.z };
			b3Vec3 pLower = { rootP.x + s * vChord.x + lower * vThick.x,
							  rootP.y + s * vChord.y + lower * vThick.y,
							  rootP.z + s * vChord.z + lower * vThick.z };

			pts.push_back( pUpper );
			pts.push_back( pLower );
		}
	}

	return b3CreateHull( pts.data(), (int)pts.size(), 32 );
}

class GliderFlight : public Sample
{
public:
	explicit GliderFlight( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -35.0f, 15.0f, 60.0f, { 0.0f, 12.0f, 0.0f } );
		}

		AddGroundBox( 100.0f );

		m_launchSpeed = 22.0f;
		m_launchPitchDeg = 5.0f;
		m_launchAltitude = 18.0f;

		SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
	}

	void SpawnGlider( b3Pos position )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = position;

		float pitchRad = m_launchPitchDeg * B3_PI / 180.0f;
		bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );

		// Initial forward velocity (toward -Z)
		float vz = -m_launchSpeed * cosf( pitchRad );
		float vy = m_launchSpeed * sinf( pitchRad );
		bodyDef.linearVelocity = { 0.0f, vy, vz };

		b3BodyId gliderId = b3CreateBody( m_worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 0.5f;

		// 1. Fuselage: streamlined capsule along Z axis (nose at -Z, tail at +Z)
		b3Capsule fuselage = { { 0.0f, 0.0f, -2.4f }, { 0.0f, 0.15f, 2.4f }, 0.22f };
		b3CreateCapsuleShape( gliderId, &shapeDef, &fuselage );

		// 2. Main Wings: wide cambered airfoil wings spanning across X (from -3.6m to +3.6m)
		// located near aerodynamic center (z = -0.3m, y = 0.22m)
		b3HullData* wingHull = CreateGliderWingHull( 7.2f, 1.3f, 0.12f, 0.035f, 3.5f, { 0.0f, 0.22f, -0.3f } );
		if ( wingHull != nullptr )
		{
			b3ShapeDef wingShape = shapeDef;
			wingShape.density = 0.4f;
			b3CreateHullShape( gliderId, &wingShape, wingHull );
			b3DestroyHull( wingHull );
		}

		// 3. Horizontal Stabilizer / Elevator: rear wing across X at the tail (z = +2.2m)
		b3HullData* hStabHull = CreateBoxHullOffset( { 1.1f, 0.02f, 0.35f }, { 0.0f, 0.25f, 2.1f } );
		if ( hStabHull != nullptr )
		{
			b3ShapeDef tailDef = shapeDef;
			tailDef.density = 0.2f;
			b3CreateHullShape( gliderId, &tailDef, hStabHull );
			b3DestroyHull( hStabHull );
		}

		// 4. Vertical Stabilizer / Rudder Fin: swept vertical fin at the tail (z = +2.0m, y pointing up)
		b3HullData* vFinHull = CreateSweptVerticalFinHull( 0.7f, 0.4f, 0.85f, 0.25f, 0.04f, { 0.0f, 0.22f, 2.0f } );
		if ( vFinHull != nullptr )
		{
			b3ShapeDef finDef = shapeDef;
			finDef.density = 0.2f;
			b3CreateHullShape( gliderId, &finDef, vFinHull );
			b3DestroyHull( vFinHull );
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift Demo: Glider Flight" );
		DrawTextLine( "Press Space to launch a new glider." );
		DrawTextLine( "Note: Aerodynamic forces will be driven by shape geometry once lift is enabled." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
		}
	}

	bool DrawControls() override
	{
		bool changed = false;
		if ( ImGui::Button( "Launch Glider" ) )
		{
			SpawnGlider( { 0.0f, m_launchAltitude, 35.0f } );
			changed = true;
		}
		ImGui::SliderFloat( "Launch Speed (m/s)", &m_launchSpeed, 5.0f, 50.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Pitch (deg)", &m_launchPitchDeg, -15.0f, 30.0f, "%.1f" );
		ImGui::SliderFloat( "Launch Altitude (m)", &m_launchAltitude, 5.0f, 50.0f, "%.1f" );
		return changed;
	}

	static Sample* Create( SampleContext* context )
	{
		return new GliderFlight( context );
	}

private:
	float m_launchSpeed;
	float m_launchPitchDeg;
	float m_launchAltitude;
};

static int sampleGliderFlight = RegisterSample( "Lift", "Glider Flight", GliderFlight::Create );

class WindTunnelAirfoil : public Sample
{
public:
	explicit WindTunnelAirfoil( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -15.0f, 10.0f, 22.0f, { 0.0f, 4.0f, 0.0f } );
		}

		AddGroundBox( 20.0f );

		m_windSpeed = 25.0f;
		m_angleOfAttackDeg = 8.0f;

		// Mount 3 test airfoils on pivot joints:
		// 1. Cambered Clark-Y style airfoil
		// 2. Symmetric NACA 0012 airfoil
		// 3. Flat plate
		SpawnAirfoilStation( { -5.0f, 4.0f, 0.0f }, "Cambered Airfoil", 0.12f, 0.05f );
		SpawnAirfoilStation( { 0.0f, 4.0f, 0.0f }, "Symmetric 0012", 0.12f, 0.00f );
		SpawnAirfoilStation( { 5.0f, 4.0f, 0.0f }, "Flat Plate", 0.02f, 0.00f );
	}

	void SpawnAirfoilStation( b3Pos pos, const char* label, float thickness, float camber )
	{
		(void)label;

		// Static mount post
		b3BodyDef postDef = b3DefaultBodyDef();
		postDef.position = { pos.x, pos.y * 0.5f, pos.z };
		b3BodyId postId = b3CreateBody( m_worldId, &postDef );
		b3Capsule postCap = { { 0.0f, -pos.y * 0.5f, 0.0f }, { 0.0f, pos.y * 0.5f, 0.0f }, 0.08f };
		b3ShapeDef postShape = b3DefaultShapeDef();
		b3CreateCapsuleShape( postId, &postShape, &postCap );

		// Dynamic airfoil wing section (span along Z, chord along X)
		b3BodyDef wingDef = b3DefaultBodyDef();
		wingDef.type = b3_dynamicBody;
		wingDef.position = pos;
		b3BodyId wingId = b3CreateBody( m_worldId, &wingDef );

		// Generate airfoil with chord along X and span along Z
		enum
		{
			kPts = 12
		};
		float chord = 1.6f;
		float span = 2.5f;
		std::vector<b3Vec3> pts;
		pts.reserve( kPts * 4 );

		for ( int i = 0; i < kPts; ++i )
		{
			float t = (float)i / (float)( kPts - 1 );
			float xFrac = 0.5f * ( 1.0f - cosf( t * B3_PI ) );
			float xLocal = ( xFrac - 0.5f ) * chord;
			float yt = 5.0f * thickness * chord *
					   ( 0.2969f * sqrtf( xFrac ) - 0.1260f * xFrac - 0.3516f * xFrac * xFrac +
						 0.2843f * xFrac * xFrac * xFrac - 0.1015f * xFrac * xFrac * xFrac * xFrac );
			float yc = 4.0f * camber * chord * xFrac * ( 1.0f - xFrac );

			pts.push_back( { xLocal, yc + yt, -0.5f * span } );
			pts.push_back( { xLocal, yc - yt, -0.5f * span } );
			pts.push_back( { xLocal, yc + yt, 0.5f * span } );
			pts.push_back( { xLocal, yc - yt, 0.5f * span } );
		}

		b3HullData* hull = b3CreateHull( pts.data(), (int)pts.size(), 32 );
		if ( hull != nullptr )
		{
			b3ShapeDef wingShape = b3DefaultShapeDef();
			wingShape.density = 1.0f;
			b3CreateHullShape( wingId, &wingShape, hull );
			b3DestroyHull( hull );
		}

		// Pivot revolute joint along Z axis with angular spring to hold pitch
		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = postId;
		jointDef.base.bodyIdB = wingId;
		jointDef.base.localFrameA.p = { 0.0f, pos.y * 0.5f, 0.0f };
		jointDef.base.localFrameB.p = b3Vec3_zero;
		jointDef.base.localFrameA.q = b3Quat_identity;
		jointDef.base.localFrameB.q = b3Quat_identity;
		jointDef.enableSpring = true;
		jointDef.hertz = 3.0f;
		jointDef.dampingRatio = 0.7f;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.5f * B3_PI;
		jointDef.upperAngle = 0.5f * B3_PI;
		b3CreateRevoluteJoint( m_worldId, &jointDef );
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Wind Tunnel Test Section" );
		DrawTextLine( "Simulated Wind Speed: %.1f m/s", m_windSpeed );
		DrawTextLine( "Three test shapes: Cambered Airfoil, Symmetric 0012, Flat Plate" );
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Wind Speed (m/s)", &m_windSpeed, 0.0f, 60.0f, "%.1f" );
		ImGui::SliderFloat( "Target AoA (deg)", &m_angleOfAttackDeg, -30.0f, 30.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new WindTunnelAirfoil( context );
	}

private:
	float m_windSpeed;
	float m_angleOfAttackDeg;
};

static int sampleWindTunnel = RegisterSample( "Lift", "Wind Tunnel Airfoil", WindTunnelAirfoil::Create );

class SpinningRotor : public Sample
{
public:
	explicit SpinningRotor( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -25.0f, 15.0f, 35.0f, { 0.0f, 10.0f, 0.0f } );
		}

		AddGroundBox( 40.0f );

		m_bladeCount = 3;
		m_initialRPM = 400.0f;
		m_dropHeight = 25.0f;

		SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
	}

	void SpawnRotorAssembly( b3Pos pos )
	{
		// 1. Central Payload / Mast body
		b3BodyDef mastDef = b3DefaultBodyDef();
		mastDef.type = b3_dynamicBody;
		mastDef.position = pos;
		b3BodyId mastId = b3CreateBody( m_worldId, &mastDef );

		b3Capsule payloadCap = { { 0.0f, -1.2f, 0.0f }, { 0.0f, 0.2f, 0.0f }, 0.35f };
		b3ShapeDef mastShape = b3DefaultShapeDef();
		mastShape.density = 2.0f;
		b3CreateCapsuleShape( mastId, &mastShape, &payloadCap );

		// 2. Rotating Rotor Hub (revolute joint along vertical Y axis)
		b3BodyDef hubDef = b3DefaultBodyDef();
		hubDef.type = b3_dynamicBody;
		hubDef.position = { pos.x, pos.y + 0.45f, pos.z };

		// Initial angular spin around Y
		float omegaY = m_initialRPM * ( 2.0f * B3_PI / 60.0f );
		hubDef.angularVelocity = { 0.0f, omegaY, 0.0f };
		b3BodyId hubId = b3CreateBody( m_worldId, &hubDef );

		// Hub central cylinder
		b3ShapeDef hubShape = b3DefaultShapeDef();
		hubShape.density = 1.0f;
		b3HullData* hubCylinder = b3CreateCylinder( 0.12f, 0.35f, 0.0f, 16 );
		if ( hubCylinder != nullptr )
		{
			b3CreateHullShape( hubId, &hubShape, hubCylinder );
			b3DestroyHull( hubCylinder );
		}

		// Attach radial pitched rotor blades directly as compound shapes to the Hub
		float innerR = 0.35f;
		float outerR = 3.8f;
		float bladeChord = 0.42f;
		float pitchAngleDeg = 6.5f; // positive collective pitch

		b3ShapeDef bladeShape = b3DefaultShapeDef();
		bladeShape.density = 0.35f;

		for ( int i = 0; i < m_bladeCount; ++i )
		{
			float azimuthRad = (float)i * ( 2.0f * B3_PI / (float)m_bladeCount );
			b3HullData* bladeHull =
				CreateRadialRotorBladeHull( innerR, outerR, bladeChord, pitchAngleDeg, azimuthRad );
			if ( bladeHull != nullptr )
			{
				b3CreateHullShape( hubId, &bladeShape, bladeHull );
				b3DestroyHull( bladeHull );
			}
		}

		// Connect hub to mast via low-friction Revolute Joint around vertical Y axis
		b3RevoluteJointDef mastJoint = b3DefaultRevoluteJointDef();
		mastJoint.base.bodyIdA = mastId;
		mastJoint.base.bodyIdB = hubId;
		mastJoint.base.localFrameA.p = { 0.0f, 0.45f, 0.0f };
		mastJoint.base.localFrameB.p = b3Vec3_zero;
		// Revolute joint default axis is local Z, rotate to Y
		mastJoint.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
		mastJoint.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI );
		b3CreateRevoluteJoint( m_worldId, &mastJoint );
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Spinning Rotor / Autogyro" );
		DrawTextLine( "Blades are pitched and attached radially to the rotating hub." );
		DrawTextLine( "Press Space to spawn another rotor assembly." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Spawn Rotor" ) )
		{
			SpawnRotorAssembly( { 0.0f, m_dropHeight, 0.0f } );
			return true;
		}
		ImGui::SliderFloat( "Initial RPM", &m_initialRPM, 0.0f, 800.0f, "%.0f" );
		ImGui::SliderFloat( "Drop Height (m)", &m_dropHeight, 5.0f, 50.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpinningRotor( context );
	}

private:
	int m_bladeCount;
	float m_initialRPM;
	float m_dropHeight;
};

static int sampleSpinningRotor = RegisterSample( "Lift", "Spinning Rotor", SpinningRotor::Create );

class FallingAirfoils : public Sample
{
public:
	explicit FallingAirfoils( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -20.0f, 15.0f, 40.0f, { 0.0f, 8.0f, 0.0f } );
		}

		AddGroundBox( 30.0f );

		SpawnShapes();
	}

	void SpawnShapes()
	{
		for ( int i = -3; i <= 3; ++i )
		{
			float x = (float)i * 3.5f;

			// 1. Cambered Airfoil (span across X, chord along Z)
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x, 16.0f, -4.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, (float)i * 0.15f );
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3HullData* hull =
					CreateGliderWingHull( 2.5f, 1.4f, 0.12f, 0.04f, 0.0f, { 0.0f, 0.0f, 0.0f } );
				if ( hull != nullptr )
				{
					b3ShapeDef shapeDef = b3DefaultShapeDef();
					shapeDef.density = 0.5f;
					b3CreateHullShape( bodyId, &shapeDef, hull );
					b3DestroyHull( hull );
				}
			}

			// 2. Flat Plate
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x, 14.0f, 0.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, (float)i * 0.2f );
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3BoxHull plate = b3MakeBoxHull( 1.0f, 0.015f, 0.8f );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				shapeDef.density = 0.3f;
				b3CreateHullShape( bodyId, &shapeDef, &plate.base );
			}

			// 3. Disc / Frisbee
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = { x, 18.0f, 4.0f };
				bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.1f * (float)i );
				bodyDef.angularVelocity = { 0.0f, 40.0f, 0.0f };
				b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );

				b3HullData* discHull = b3CreateCylinder( 0.04f, 0.7f, 0.0f, 16 );
				if ( discHull != nullptr )
				{
					b3ShapeDef shapeDef = b3DefaultShapeDef();
					shapeDef.density = 0.6f;
					b3CreateHullShape( bodyId, &shapeDef, discHull );
					b3DestroyHull( discHull );
				}
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Falling Airfoils, Plates & Discs" );
		DrawTextLine( "Showcases diverse geometry ready for aerodynamic drag & lift forces." );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FallingAirfoils( context );
	}
};

static int sampleFallingAirfoils = RegisterSample( "Lift", "Falling Airfoils", FallingAirfoils::Create );

class FinStabilizedDarts : public Sample
{
public:
	explicit FinStabilizedDarts( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -40.0f, 12.0f, 50.0f, { 0.0f, 6.0f, 0.0f } );
		}

		AddGroundBox( 50.0f );

		m_launchSpeed = 40.0f;

		for ( int i = 0; i < 5; ++i )
		{
			LaunchDart( { -25.0f, 4.0f + (float)i * 2.0f, (float)( i - 2 ) * 3.0f } );
		}
	}

	void LaunchDart( b3Pos pos )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = pos;
		bodyDef.linearVelocity = { m_launchSpeed, 2.0f, 0.0f }; // high speed along +X

		b3BodyId dartId = b3CreateBody( m_worldId, &bodyDef );

		// 1. Slender body shaft (capsule along X from tail at x=-1.4 to front at x=+0.8)
		b3Capsule shaft = { { -1.4f, 0.0f, 0.0f }, { 0.8f, 0.0f, 0.0f }, 0.08f };
		b3ShapeDef shaftShape = b3DefaultShapeDef();
		shaftShape.density = 2.0f;
		b3CreateCapsuleShape( dartId, &shaftShape, &shaft );

		// 2. Heavy nose tip at the front (+X) for forward CG
		b3Sphere nose = { { 0.88f, 0.0f, 0.0f }, 0.14f };
		b3ShapeDef noseShape = b3DefaultShapeDef();
		noseShape.density = 12.0f;
		b3CreateSphereShape( dartId, &noseShape, &nose );

		// 3. Four stabilizing delta fins positioned at the TAIL end (x in [-1.4f, -0.8f])
		b3ShapeDef finShape = b3DefaultShapeDef();
		finShape.density = 0.3f;

		// Horizontal fin pair (extending left/right in +/-Z) at the tail
		b3HullData* horizFins = CreateDartDeltaFins( -0.8f, -1.4f, 0.42f, 0.02f, false );
		if ( horizFins != nullptr )
		{
			b3CreateHullShape( dartId, &finShape, horizFins );
			b3DestroyHull( horizFins );
		}

		// Vertical fin pair (extending up/down in +/-Y) at the tail
		b3HullData* vertFins = CreateDartDeltaFins( -0.8f, -1.4f, 0.42f, 0.02f, true );
		if ( vertFins != nullptr )
		{
			b3CreateHullShape( dartId, &finShape, vertFins );
			b3DestroyHull( vertFins );
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine( "Shape-Based Lift: Fin-Stabilized Darts" );
		DrawTextLine( "Swept stabilizing fins are at the rear tail end." );
		DrawTextLine( "Press Space to launch more darts." );
	}

	void Keyboard( int key, int action, int modifiers ) override
	{
		(void)modifiers;
		if ( action == ACTION_PRESS && key == KEY_SPACE )
		{
			LaunchDart( { -25.0f, 6.0f, (float)( ( m_stepCount % 5 ) - 2 ) * 2.5f } );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Button( "Launch Dart" ) )
		{
			LaunchDart( { -25.0f, 6.0f, 0.0f } );
			return true;
		}
		ImGui::SliderFloat( "Dart Speed (m/s)", &m_launchSpeed, 10.0f, 80.0f, "%.1f" );
		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new FinStabilizedDarts( context );
	}

private:
	float m_launchSpeed;
};

static int sampleDarts = RegisterSample( "Lift", "Fin Stabilized Darts", FinStabilizedDarts::Create );
