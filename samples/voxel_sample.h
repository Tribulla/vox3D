#pragma once

#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

static inline void AppendBox( std::vector<b3Vec3i>& cells, int nx, int ny, int nz, int ox, int oy, int oz )
{
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
				cells.push_back( b3Vec3i{ ox + x, oy + y, oz + z } );
}

static inline void AppendBoxShell( std::vector<b3Vec3i>& cells, int nx, int ny, int nz, int ox, int oy, int oz,
								   int thickness )
{
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
			{
				bool shell = x < thickness || x >= nx - thickness || y < thickness || y >= ny - thickness ||
							 z < thickness || z >= nz - thickness;
				if ( shell )
					cells.push_back( b3Vec3i{ ox + x, oy + y, oz + z } );
			}
}

static inline void AppendVoxelSphere( std::vector<b3Vec3i>& cells, int cx, int cy, int cz, int r )
{
	float rr = ( r + 0.25f ) * ( r + 0.25f );
	for ( int dx = -r; dx <= r; ++dx )
		for ( int dy = -r; dy <= r; ++dy )
			for ( int dz = -r; dz <= r; ++dz )
				if ( (float)( dx * dx + dy * dy + dz * dz ) <= rr )
					cells.push_back( b3Vec3i{ cx + dx, cy + dy, cz + dz } );
}

static inline void AppendVoxelCylinderY( std::vector<b3Vec3i>& cells, int cx, int oy, int cz, int r, int h )
{
	float rr = ( r + 0.25f ) * ( r + 0.25f );
	for ( int dx = -r; dx <= r; ++dx )
		for ( int dz = -r; dz <= r; ++dz )
			if ( (float)( dx * dx + dz * dz ) <= rr )
				for ( int y = 0; y < h; ++y )
					cells.push_back( b3Vec3i{ cx + dx, oy + y, cz + dz } );
}

static inline void AppendVoxelPipe( std::vector<b3Vec3i>& cells, int cx, int oy, int cz, int r, int t, int h )
{
	float ro2 = ( r + 0.25f ) * ( r + 0.25f );
	float ri = (float)( r - t );
	float ri2 = ( ri + 0.25f ) * ( ri + 0.25f );
	for ( int dx = -r; dx <= r; ++dx )
		for ( int dz = -r; dz <= r; ++dz )
		{
			float d2 = (float)( dx * dx + dz * dz );
			if ( d2 <= ro2 && d2 > ri2 )
				for ( int y = 0; y < h; ++y )
					cells.push_back( b3Vec3i{ cx + dx, oy + y, cz + dz } );
		}
}

static inline void AppendVoxelTorus( std::vector<b3Vec3i>& cells, int cx, int cy, int cz, int R, int r )
{
	int ext = R + r;
	for ( int dx = -ext; dx <= ext; ++dx )
		for ( int dz = -ext; dz <= ext; ++dz )
		{
			float q = sqrtf( (float)( dx * dx + dz * dz ) ) - (float)R;
			for ( int dy = -r; dy <= r; ++dy )
				if ( q * q + (float)( dy * dy ) <= ( r + 0.25f ) * ( r + 0.25f ) )
					cells.push_back( b3Vec3i{ cx + dx, cy + dy, cz + dz } );
		}
}

static inline void AppendVoxelPyramid( std::vector<b3Vec3i>& cells, int base, int ox, int oy, int oz )
{
	int y = 0;
	for ( int size = base; size >= 1; size -= 2, ++y )
	{
		int off = ( base - size ) / 2;
		for ( int x = 0; x < size; ++x )
			for ( int z = 0; z < size; ++z )
				cells.push_back( b3Vec3i{ ox + off + x, oy + y, oz + off + z } );
	}
}

static inline void AppendVoxelArch( std::vector<b3Vec3i>& cells, int ox, int oy, int oz, int span, int rise, int thick,
								   int depth )
{
	float cxf = ox + 0.5f * ( span - 1 );
	float a = 0.5f * ( span - 1 ); // horizontal semi-axis
	float b = (float)rise;		   // vertical semi-axis
	for ( int x = 0; x < span; ++x )
		for ( int y = 0; y <= rise; ++y )
		{
			float nx = ( ( ox + x ) - cxf ) / a;
			float ny = (float)y / b;
			float rad = sqrtf( nx * nx + ny * ny );
			float inner = 1.0f - (float)thick / b;
			if ( rad <= 1.0f && rad >= inner )
				for ( int d = 0; d < depth; ++d )
					cells.push_back( b3Vec3i{ ox + x, oy + y, oz + d } );
		}
}

static inline const char* const* VoxGlyph( char c )
{
	static const char* V[7] = { "#...#", "#...#", "#...#", "#...#", ".#.#.", ".#.#.", "..#.." };
	static const char* O[7] = { ".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###." };
	static const char* X[7] = { "#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#" };
	static const char* T3[7] = { "####.", "....#", "....#", ".###.", "....#", "....#", "####." };
	static const char* D[7] = { "###..", "#..#.", "#...#", "#...#", "#...#", "#..#.", "###.." };
	static const char* SP[7] = { ".....", ".....", ".....", ".....", ".....", ".....", "....." };
	switch ( c )
	{
		case 'V':
			return V;
		case 'O':
			return O;
		case 'X':
			return X;
		case '3':
			return T3;
		case 'D':
			return D;
		default:
			return SP;
	}
}

static inline void AppendText( std::vector<b3Vec3i>& cells, const char* text, int ox, int oy, int oz, int depth )
{
	int penX = ox;
	for ( const char* p = text; *p; ++p )
	{
		const char* const* g = VoxGlyph( *p );
		for ( int row = 0; row < 7; ++row )
			for ( int col = 0; col < 5; ++col )
				if ( g[row][col] == '#' )
					for ( int d = 0; d < depth; ++d )
						cells.push_back( b3Vec3i{ penX + col, oy + ( 6 - row ), oz + d } );
		penX += 6; // 5 wide + 1 space
	}
}

static inline bool VoxAnchorLowX( b3Vec3i cell, void* context )
{
	return cell.x < *(int*)context;
}


class VoxelSample : public Sample
{
public:
	explicit VoxelSample( SampleContext* context )
		: Sample( context )
	{
	}

	void Boot()
	{
		if ( m_context->restart == false )
		{
			SetupCamera();
		}

		b3World_SetGravity( m_worldId, { 0.0f, -9.81f, 0.0f } );
		AddGroundBox( m_groundExtent );
		b3World_EnableFracture( m_worldId, m_voxel, m_groundY );

		if ( m_stressDefault )
		{
			m_colorMode = b3_fractureColorStress;
		}

		BuildScene();
		b3World_ApplyFractureColors( m_worldId, m_colorMode );
	}

	virtual void SetupCamera()
	{
		m_camera->SetView( -35.0f, 18.0f, 52.0f, { 0.0f, 6.0f, 0.0f } );
	}

	virtual void BuildScene() = 0;

	virtual void OnStep()
	{
	}
	virtual void ExtraHud()
	{
	}
	virtual bool ExtraControls()
	{
		return false;
	}
	virtual void OnKey( int /*key*/ )
	{
	}

	void Step() override
	{
		Sample::Step();
		OnStep();

		int bodies = b3World_GetFractureBodyCount( m_worldId );
		if ( m_colorMode == b3_fractureColorStress || m_colorMode != m_lastColorMode || bodies != m_lastBodyCount )
		{
			b3World_ApplyFractureColors( m_worldId, m_colorMode );
		}
		m_lastColorMode = m_colorMode;
		m_lastBodyCount = bodies;

		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		static const char* modeName[] = { "material", "stress", "fragment" };

		ResetText();
		DrawTextLine( "pieces %d   voxels %d   peak stress %.2f", bodies, b3World_GetFractureVoxelCount( m_worldId ),
					  b3World_GetFractureMaxStress( m_worldId ) );
		ExtraHud();
		DrawTextLine( "V: colour (%s)    B: fracture (%s)    Shift+Left: shoot", modeName[(int)m_colorMode],
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
		else
		{
			OnKey( key );
		}
	}

	bool DrawControls() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		bool changed = false;

		if ( ImGui::CollapsingHeader( "Fracture", ImGuiTreeNodeFlags_DefaultOpen ) )
		{
			changed |= ImGui::Checkbox( "Fracture enabled", &t.fractureEnabled );
			changed |= ImGui::Checkbox( "Contact stress", &t.contactStress );
			changed |= ImGui::Checkbox( "Parallel analysis (needs workers > 1)", &t.parallelAnalysis );
			changed |= ImGui::SliderFloat( "Strength scale", &t.strengthScale, 0.1f, 5.0f, "%.2f" );
			changed |= ImGui::SliderFloat( "Crack roughness", &t.fractureRoughness, 0.0f, 4.0f, "%.1f" );
			changed |= ImGui::SliderFloat( "Contact smoothing", &t.contactSmoothing, 0.02f, 1.0f, "%.2f" );
			changed |= ImGui::SliderInt( "Analysis stride", &t.analysisStride, 1, 8 );
			changed |= ImGui::SliderInt( "Hold frames", &t.fractureHoldFrames, 1, 30 );
			changed |= ImGui::SliderInt( "Max debris (0=unlimited)", &t.maxDebris, 0, 5000 );
		}

		if ( ImGui::CollapsingHeader( "Impact" ) )
		{
			changed |= ImGui::SliderFloat( "Settle speed", &t.settleSpeed, 0.5f, 10.0f, "%.1f" );
			changed |= ImGui::SliderFloat( "Impact speed", &t.impactSpeed, 1.0f, 20.0f, "%.1f" );
			changed |= ImGui::SliderFloat( "Impact bearing", &t.impactBearing, 0.25f, 3.0f, "%.2f" );
			changed |= ImGui::SliderInt( "Impact radius", &t.impactRadius, 1, 6 );
		}

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

		ExtraControls();
		ImGui::TextDisabled( "Shift+Left click to shoot" );
		return true;
	}

	int AddVoxels( const std::vector<b3Vec3i>& cells, b3FractureMaterial material, const b3FractureDef* def = nullptr )
	{
		if ( cells.empty() )
			return -1;
		return b3World_CreateFractureVoxels( m_worldId, cells.data(), (int)cells.size(), material, def );
	}

	b3BodyId FireBall( b3Vec3 center, b3Vec3 velocity, float radius, uint32_t color, float density = 8.0f )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = b3ToPos( center );
		bd.linearVelocity = velocity;
		bd.isBullet = true;
		b3BodyId body = b3CreateBody( m_worldId, &bd );

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = density;
		sd.baseMaterial.friction = 0.5f;
		sd.baseMaterial.customColor = color;
		sd.enableContactEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
		b3CreateSphereShape( body, &sd, &sphere );
		return body;
	}

	b3FractureMaterial Material( b3FractureMaterialType type ) const
	{
		return b3GetFractureMaterial( type );
	}

	void HudProfile()
	{
		b3Profile p = b3World_GetProfile( m_worldId );
		b3Counters c = b3World_GetCounters( m_worldId );
		DrawTextLine( "step %.2f ms | fracture %.2f (gather %.2f  analyze %.2f  sever %.2f  debris %.2f) ms", p.step,
					  p.fracture, p.fractureGather, p.fractureAnalyze, p.fractureSever, p.fractureDebris );
		DrawTextLine( "bodies %d  shapes %d  contacts %d | workers %d", c.bodyCount, c.shapeCount, c.contactCount,
					  m_context->workerCount );
	}

	void Rebuild()
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );

		b3Capacity capacity = {};
		CreateWorld( &capacity );
		ResetProfile();

		b3World_SetGravity( m_worldId, { 0.0f, -9.81f, 0.0f } );
		AddGroundBox( m_groundExtent );
		b3World_EnableFracture( m_worldId, m_voxel, m_groundY );
		b3World_SetFractureTuning( m_worldId, t );

		BuildScene();
		b3World_ApplyFractureColors( m_worldId, m_colorMode );

		m_lastColorMode = -1;
		m_lastBodyCount = -1;
	}

protected:
	float m_voxel = 1.0f;
	float m_groundY = 0.0f;
	float m_groundExtent = 60.0f;
	bool m_stressDefault = false; // start in stress-heatmap colour mode

	b3FractureColorMode m_colorMode = b3_fractureColorMaterial;
	int m_lastColorMode = -1;
	int m_lastBodyCount = -1;
};
