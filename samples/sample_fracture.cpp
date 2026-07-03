#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <cstring>
#include <vector>

static void AppendBox( std::vector<b3Vec3i>& cells, int nx, int ny, int nz, int ox, int oy, int oz )
{
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
				cells.push_back( b3Vec3i{ ox + x, oy + y, oz + z } );
}

static bool AnchorLowX( b3Vec3i cell, void* context )
{
	return cell.x < *(int*)context;
}

static void AddConvex( b3WorldId worldId, b3HullData* hull, b3Vec3 pos, float baseY, int chunks,
					   b3FractureMaterial mat )
{
	b3Transform identity = { b3Vec3_zero, b3Quat_identity };
	b3AABB aabb = b3ComputeHullAABB( hull, identity );
	b3WorldTransform xf = { { pos.x, baseY - aabb.lowerBound.y, pos.z }, b3Quat_identity };
	b3World_CreateFractureConvex( worldId, hull, xf, chunks, mat, nullptr );
	b3DestroyHull( hull );
}

static b3BodyId FireBall( b3WorldId worldId, b3Vec3 center, b3Vec3 velocity, float radius, uint32_t color )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = b3ToPos( center );
	bd.linearVelocity = velocity;
	bd.isBullet = true;
	b3BodyId body = b3CreateBody( worldId, &bd );

	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 8.0f;
	sd.baseMaterial.friction = 0.5f;
	sd.baseMaterial.customColor = color;
	sd.enableContactEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
	b3CreateSphereShape( body, &sd, &sphere );
	return body;
}

class FractureSample : public Sample
{
public:
	FractureSample( SampleContext* context, const char* demo )
		: Sample( context ), m_demo( demo )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -35.0f, 18.0f, 52.0f, { 0.0f, 6.0f, 0.0f } );
		}

		b3World_SetGravity( m_worldId, { 0.0f, -9.81f, 0.0f } );
		AddGroundBox( 60.0f );

		b3World_EnableFracture( m_worldId, 1.0f, 0.0f );

		BuildScene();
		b3World_ApplyFractureColors( m_worldId, m_colorMode );
	}

	void BuildScene()
	{
		std::vector<b3Vec3i> cells;
		b3FractureMaterial concrete = b3GetFractureMaterial( b3_fractureConcrete );
		b3FractureMaterial brick = b3GetFractureMaterial( b3_fractureBrick );
		b3FractureMaterial stone = b3GetFractureMaterial( b3_fractureStone );

		if ( strcmp( m_demo, "voxel_wall" ) == 0 )
		{
			AppendBox( cells, 22, 10, 2, -11, 0, -1 );
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), brick, nullptr );
		}
		else if ( strcmp( m_demo, "voxel_tower" ) == 0 )
		{
			AppendBox( cells, 4, 20, 4, -2, 0, -2 );
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), concrete, nullptr );
		}
		else if ( strcmp( m_demo, "voxel_pyramid" ) == 0 )
		{
			int base = 13, y = 0;
			for ( int size = base; size >= 1; size -= 2, ++y )
			{
				int off = ( base - size ) / 2;
				for ( int x = 0; x < size; ++x )
					for ( int z = 0; z < size; ++z )
						cells.push_back( b3Vec3i{ -6 + off + x, y, -6 + off + z } );
			}
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), stone, nullptr );
		}
		else if ( strcmp( m_demo, "voxel_cantilever" ) == 0 )
		{
			AppendBox( cells, 26, 4, 4, -2, 14, -2 );
			int thresh = 0; // anchor the two columns above origin x = -2
			b3FractureDef def = b3DefaultFractureDef();
			def.anchor = AnchorLowX;
			def.anchorContext = &thresh;
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), concrete, &def );
		}
		else if ( strcmp( m_demo, "beam_snap" ) == 0 )
		{
			AppendBox( cells, 24, 3, 3, -3, 12, -1 );
			int thresh = -1;
			b3FractureDef def = b3DefaultFractureDef();
			def.merge = true; // a solid ("normal") rigid body
			def.anchor = AnchorLowX;
			def.anchorContext = &thresh;
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), concrete, &def );
		}
		else if ( strcmp( m_demo, "crate_smash" ) == 0 )
		{
			b3World_CreateFractureBox( m_worldId, { 0.0f, 3.0f, 0.0f }, { 3.0f, 3.0f, 3.0f }, stone, nullptr );
		}
		else if ( strcmp( m_demo, "bridge" ) == 0 )
		{
			b3World_CreateFractureBox( m_worldId, { -8.0f, 4.0f, 0.0f }, { 2.0f, 4.0f, 4.0f }, stone, nullptr );
			b3World_CreateFractureBox( m_worldId, { 8.0f, 4.0f, 0.0f }, { 2.0f, 4.0f, 4.0f }, stone, nullptr );
			b3World_CreateFractureBox( m_worldId, { 0.0f, 10.5f, 0.0f }, { 10.0f, 2.5f, 4.0f }, concrete, nullptr );
		}
		else if ( strcmp( m_demo, "crush" ) == 0 )
		{
			b3FractureMaterial metal = b3GetFractureMaterial( b3_fractureMetal );
			b3World_CreateFractureBox( m_worldId, { -5.0f, 5.0f, 0.0f }, { 1.0f, 5.0f, 2.5f }, concrete, nullptr );
			b3World_CreateFractureBox( m_worldId, { 5.0f, 5.0f, 0.0f }, { 1.0f, 5.0f, 2.5f }, concrete, nullptr );
			b3World_CreateFractureBox( m_worldId, { 0.0f, 11.0f, 0.0f }, { 8.0f, 1.0f, 2.5f }, concrete, nullptr );
			b3World_CreateFractureBox( m_worldId, { 0.0f, 15.0f, 0.0f }, { 7.0f, 3.0f, 2.5f }, metal, nullptr );
		}
		else if ( strcmp( m_demo, "column" ) == 0 )
		{
			// a round stone column (a real convex mesh) pre-fractured into chunks
			AddConvex( m_worldId, b3CreateCylinder( 12.0f, 2.0f, 0.0f, 20 ), { 0.0f, 0.0f, 0.0f }, 0.0f, 30,
					   b3GetFractureMaterial( b3_fractureStone ) );
		}
		else if ( strcmp( m_demo, "boulder" ) == 0 )
		{
			// an irregular rock hull, shattered along Voronoi seams
			AddConvex( m_worldId, b3CreateRock( 4.0f ), { 0.0f, 0.0f, 0.0f }, 0.0f, 24,
					   b3GetFractureMaterial( b3_fractureStone ) );
		}
		else if ( strcmp( m_demo, "gem" ) == 0 )
		{
			// a faceted crystal spike (a tapered cone) made of glass, resting on its base
			AddConvex( m_worldId, b3CreateCone( 7.0f, 2.2f, 0.3f, 8 ), { 0.0f, 0.0f, 0.0f }, 0.0f, 18,
					   b3GetFractureMaterial( b3_fractureGlass ) );
		}
		else // "impact" (self-running)
		{
			AppendBox( cells, 22, 10, 2, -11, 0, -1 );
			b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), brick, nullptr );
			FireBall( m_worldId, { 0.0f, 5.0f, 24.0f }, { 0.0f, 0.0f, -46.0f }, 2.5f, 0xC0C0C8 );
		}
	}

	void Step() override
	{
		Sample::Step();

		int bodies = b3World_GetFractureBodyCount( m_worldId );
		if ( m_colorMode == b3_fractureColorStress || m_colorMode != m_lastColorMode || bodies != m_lastBodyCount )
		{
			b3World_ApplyFractureColors( m_worldId, m_colorMode );
		}
		m_lastColorMode = m_colorMode;
		m_lastBodyCount = bodies;

		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		ResetText();
		DrawTextLine( "%s", m_demo );
		DrawTextLine( "pieces %d   voxels %d   peak stress %.2f", b3World_GetFractureBodyCount( m_worldId ),
					  b3World_GetFractureVoxelCount( m_worldId ), b3World_GetFractureMaxStress( m_worldId ) );
		static const char* modeName[] = { "material", "stress", "fragment" };
		DrawTextLine( "Shift+Left: shoot    V: colour (%s)    B: fracture (%s)", modeName[(int)m_colorMode],
					  t.fractureEnabled ? "on" : "off" );
	}

	void Keyboard( int key, int action, int ) override
	{
		if ( action != ACTION_PRESS )
		{
			return;
		}
		if ( key == KEY_V )
		{
			m_colorMode = (b3FractureColorMode)( ( (int)m_colorMode + 1 ) % 3 );
		}
		else if ( key == KEY_B )
		{
			b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
			t.fractureEnabled = !t.fractureEnabled;
			b3World_SetFractureTuning( m_worldId, t );
		}
	}

	bool DrawControls() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		bool changed = false;

		ImGui::Text( "Demo: %s", m_demo );
		changed |= ImGui::Checkbox( "Fracture enabled", &t.fractureEnabled );
		changed |= ImGui::Checkbox( "Contact stress", &t.contactStress );
		changed |= ImGui::SliderFloat( "Strength scale", &t.strengthScale, 0.1f, 5.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Crack roughness", &t.fractureRoughness, 0.0f, 4.0f, "%.1f" );
		if ( changed )
		{
			b3World_SetFractureTuning( m_worldId, t );
		}

		int mode = (int)m_colorMode;
		const char* modes[] = { "Material", "Stress", "Fragment" };
		if ( ImGui::Combo( "Colour", &mode, modes, 3 ) )
		{
			m_colorMode = (b3FractureColorMode)mode;
		}

		ImGui::TextDisabled( "Shift + Left click to shoot" );
		return true;
	}

	const char* m_demo;
	b3FractureColorMode m_colorMode = b3_fractureColorMaterial;
	int m_lastColorMode = -1;
	int m_lastBodyCount = -1;
};

FRACTURE_SAMPLE( CreateVoxelWall, "voxel_wall", "Voxel Wall" );
FRACTURE_SAMPLE( CreateVoxelTower, "voxel_tower", "Voxel Tower" );
FRACTURE_SAMPLE( CreateVoxelPyramid, "voxel_pyramid", "Voxel Pyramid" );
FRACTURE_SAMPLE( CreateVoxelCantilever, "voxel_cantilever", "Voxel Cantilever" );
FRACTURE_SAMPLE( CreateBeamSnap, "beam_snap", "Beam Snap (solid)" );
FRACTURE_SAMPLE( CreateCrateSmash, "crate_smash", "Crate Smash (solid)" );
FRACTURE_SAMPLE( CreateBridge, "bridge", "Bridge (solid span)" );
FRACTURE_SAMPLE( CreateCrush, "crush", "Crush (solid pillars)" );
FRACTURE_SAMPLE( CreateColumn, "column", "Column (mesh chunks)" );
FRACTURE_SAMPLE( CreateBoulder, "boulder", "Boulder (mesh chunks)" );
FRACTURE_SAMPLE( CreateGem, "gem", "Gem (mesh chunks)" );
FRACTURE_SAMPLE( CreateImpact, "impact", "Impact (self-running)" );
