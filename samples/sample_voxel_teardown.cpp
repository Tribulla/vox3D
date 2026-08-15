#include "utils.h"
#include "voxel_sample.h"

#include <algorithm>

static bool AnchorFoundation( b3Vec3i cell, void* )
{
	return cell.y <= 0;
}

class VoxelTeardown : public VoxelSample
{
public:
	explicit VoxelTeardown( SampleContext* context )
		: VoxelSample( context )
	{
		m_vpm = m_isDebug ? 2 : 4;
		if ( const char* e = getenv( "VOX_VPM" ) ) // headless perf runs at a chosen resolution
			m_vpm = b3ClampInt( atoi( e ), 2, 8 );
		m_voxel = 1.0f / (float)m_vpm;
		m_groundExtent = 42.0f;
		m_groundThick = 1; // a thin sheet keeps the static ground at ~100k cells, not ~400k
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -38.0f, 20.0f, 95.0f, { 0.0f, 8.0f, 0.0f } );
	}

	int C( float meters ) const
	{
		return (int)lroundf( meters * (float)m_vpm );
	}

	void AddAnchored( const std::vector<b3Vec3i>& cells, b3FractureMaterialType type )
	{
		b3FractureDef def = b3DefaultFractureDef();
		def.anchor = AnchorFoundation;
		AddVoxels( cells, Material( type ), &def );
	}

	void AppendOffice( std::vector<b3Vec3i>& cells, int ox, int oz, float wM, float dM, int floors )
	{
		int w = C( wM ), d = C( dM );
		int floorH = C( 3.0f ), t = C( 0.5f );
		int sill = C( 1.0f ), head = C( 0.6f );
		int winW = C( 1.4f ), pierW = C( 0.9f ), cornerW = C( 1.0f );
		int h = floors * floorH;

		for ( int x = 0; x < w; ++x )
			for ( int z = 0; z < d; ++z )
			{
				bool wallX = x < t || x >= w - t; // faces where windows run along z
				bool wallZ = z < t || z >= d - t;
				if ( !wallX && !wallZ )
					continue;
				int along = wallX ? z : x;
				int extent = wallX ? d : w;
				bool inWinColumn = false;
				if ( along >= cornerW && along < extent - cornerW )
					inWinColumn = ( ( along - cornerW ) % ( winW + pierW ) ) < winW;
				for ( int y = 0; y < h; ++y )
				{
					int fy = y % floorH;
					if ( inWinColumn && fy >= sill && fy < floorH - head )
						continue;
					cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
				}
			}

		int slabT = C( 0.3f ) > 0 ? C( 0.3f ) : 1;
		for ( int f = 1; f <= floors; ++f )
			AppendBox( cells, w - 2 * t, slabT, d - 2 * t, ox + t, f * floorH - slabT, oz + t );
		for ( int x = 0; x < w; ++x )
			for ( int z = 0; z < d; ++z )
				if ( x < t || x >= w - t || z < t || z >= d - t )
					for ( int y = h; y < h + C( 0.8f ); ++y )
						cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
	}

	void AppendHouse( std::vector<b3Vec3i>& cells, int ox, int oz, float wM, float dM, float wallM, float ridgeM )
	{
		int w = C( wM ), d = C( dM );
		int wallH = C( wallM ), ridge = C( ridgeM ), t = C( 0.5f );
		int doorW = C( 1.0f ), doorH = C( 2.2f );
		int winW = C( 1.2f ), pierW = C( 1.2f ), sill = C( 1.0f ), head = C( 0.6f );
		float halfSpan = 0.5f * (float)( w - 1 );

		for ( int x = 0; x < w; ++x )
		{
			int ry = wallH + (int)lroundf( (float)ridge * ( 1.0f - fabsf( (float)x - halfSpan ) / halfSpan ) );
			for ( int z = 0; z < d; ++z )
			{
				bool wallX = x < t || x >= w - t;
				bool wallZ = z < t || z >= d - t; // gable ends rise to the roofline
				int top = wallZ ? ry : wallH;
				if ( wallX || wallZ )
				{
					int along = wallX ? z : x;
					int extent = wallX ? d : w;
					bool inWinColumn = along >= t + winW / 2 && along < extent - t - winW / 2 &&
									   ( ( along - t ) % ( winW + pierW ) ) < winW;
					bool inDoor = z < t && x >= w / 2 - doorW && x <= w / 2 + doorW;
					for ( int y = 0; y < top; ++y )
					{
						if ( inDoor && y < doorH )
							continue;
						if ( inWinColumn && y >= sill && y < sill + ( wallH - sill - head ) && y < wallH )
							continue;
						cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
					}
				}
				for ( int y = ry; y < ry + 2; ++y )
					cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
			}
		}
	}

	void AppendKeep( std::vector<b3Vec3i>& cells, int cx, int cz, float radiusM, float heightM )
	{
		int r = C( radiusM ), t = C( 0.75f ), h = C( heightM );
		int doorH = C( 2.5f ), merlonH = C( 0.8f );
		float ro2 = ( r + 0.25f ) * ( r + 0.25f );
		float ri = (float)( r - t );
		float ri2 = ( ri + 0.25f ) * ( ri + 0.25f );

		for ( int dx = -r; dx <= r; ++dx )
			for ( int dz = -r; dz <= r; ++dz )
			{
				float d2 = (float)( dx * dx + dz * dz );
				if ( d2 > ro2 || d2 <= ri2 )
					continue;
				float theta = atan2f( (float)dz, (float)dx );
				bool door = fabsf( fabsf( theta ) - B3_PI ) < 0.35f;
				for ( int y = 0; y < h; ++y )
				{
					if ( door && y < doorH )
						continue;
					cells.push_back( b3Vec3i{ cx + dx, y, cz + dz } );
				}
				int bucket = (int)( ( theta + B3_PI ) * 16.0f / ( 2.0f * B3_PI ) );
				if ( ( bucket & 1 ) == 0 )
					for ( int y = h; y < h + merlonH; ++y )
						cells.push_back( b3Vec3i{ cx + dx, y, cz + dz } );
			}
	}

	void AppendAqueduct( std::vector<b3Vec3i>& cells, int ox, int oz, float lenM, float heightM, int arches )
	{
		int len = C( lenM ), h = C( heightM ), t = C( 1.0f );
		float pitch = (float)len / (float)arches;
		float spanHalf = 0.38f * pitch;
		float rise = (float)h - (float)C( 1.2f );

		for ( int x = 0; x < len; ++x )
		{
			bool hole = false;
			float fx = (float)x;
			for ( int k = 0; k < arches && !hole; ++k )
			{
				float xc = ( (float)k + 0.5f ) * pitch;
				float nx = ( fx - xc ) / spanHalf;
				if ( nx * nx < 1.0f )
				{
					float opening = rise * sqrtf( 1.0f - nx * nx );
					hole = true;
					for ( int y = 0; y < h; ++y )
						if ( (float)y >= opening )
							for ( int z = 0; z < t; ++z )
								cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
				}
			}
			if ( !hole )
				for ( int y = 0; y < h; ++y )
					for ( int z = 0; z < t; ++z )
						cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
		}
		AppendBox( cells, len, C( 0.5f ) > 0 ? C( 0.5f ) : 1, t + 2, ox, h, oz - 1 );
	}

	void AppendWarehouse( std::vector<b3Vec3i>& cells, int ox, int oz, float wM, float dM, float hM )
	{
		int w = C( wM ), d = C( dM ), h = C( hM ), t = C( 0.5f );
		int doorW = C( 2.5f ), doorH = C( 3.5f );
		for ( int x = 0; x < w; ++x )
			for ( int z = 0; z < d; ++z )
			{
				bool wall = x < t || x >= w - t || z < t || z >= d - t;
				if ( !wall )
					continue;
				bool inDoor = z < t && x >= w / 2 - doorW / 2 && x < w / 2 + doorW / 2;
				for ( int y = 0; y < h; ++y )
				{
					if ( inDoor && y < doorH )
						continue;
					cells.push_back( b3Vec3i{ ox + x, y, oz + z } );
				}
			}
		AppendBox( cells, w, 2, d, ox, h, oz );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.strengthScale = 0.5f;
		t.impactRadius = m_vpm > 2 ? m_vpm : 2; // a solid hit carves out roughly a meter
		t.minFragment = m_vpm / 2 > 1 ? m_vpm / 2 : 1; // cull slivers the camera can barely see
		t.maxDebris = m_isDebug ? 1500 : 4000;
		t.analysisStride = 2;
		t.warmupFrames = 30;
		b3World_SetFractureTuning( m_worldId, t );

		m_targets.clear();
		m_shots.clear(); // Rebuild destroyed the world, so tracked bodies are gone
		std::vector<b3Vec3i> cells;

		AppendOffice( cells, C( -32.0f ), C( -20.0f ), 16.0f, 12.0f, m_isDebug ? 3 : 5 );
		AddAnchored( cells, b3_fractureConcrete );
		m_targets.push_back( { -24.0f, -14.0f } );

		cells.clear();
		AppendHouse( cells, C( 3.0f ), C( -22.0f ), 10.0f, 8.0f, 4.0f, 3.0f );
		AddAnchored( cells, b3_fractureBrick );
		m_targets.push_back( { 8.0f, -18.0f } );

		cells.clear();
		AppendKeep( cells, C( 22.0f ), C( 10.0f ), 3.5f, 12.0f );
		AddAnchored( cells, b3_fractureStone );
		m_targets.push_back( { 22.0f, 10.0f } );

		cells.clear();
		int stackT = C( 0.5f ) > 1 ? C( 0.5f ) : 2;
		AppendVoxelPipe( cells, C( -8.0f ), 0, C( 16.0f ), C( 1.25f ), stackT, C( 18.0f ) );
		AddAnchored( cells, b3_fractureBrick );
		m_targets.push_back( { -8.0f, 16.0f } );

		cells.clear();
		AppendAqueduct( cells, C( -20.0f ), C( 28.0f ), 40.0f, 8.0f, 5 );
		AddAnchored( cells, b3_fractureStone );
		m_targets.push_back( { 0.0f, 30.0f } );

		cells.clear();
		AppendWarehouse( cells, C( -30.0f ), C( 4.0f ), 14.0f, 9.0f, 5.0f );
		AddAnchored( cells, b3_fractureWood );
		m_targets.push_back( { -23.0f, 8.5f } );

		cells.clear();
		int mx = C( 14.0f ) - 10, mz = C( 20.0f ) - 9;
		AppendBox( cells, 26, 2, 24, mx - 3, 0, mz - 3 );
		AppendVoxelMoai( cells, mx, 2, mz );
		AddAnchored( cells, b3_fractureStone );
		m_targets.push_back( { 14.0f, 20.0f } );

		int gw = C( 8.0f ), gd = C( 5.0f ), gh = C( 3.5f ), post = C( 0.4f ) > 0 ? C( 0.4f ) : 1;
		int gx = C( -12.0f ), gz = C( -34.0f );
		cells.clear();
		for ( int corner = 0; corner < 4; ++corner )
		{
			int px = gx + ( corner & 1 ? gw - post : 0 );
			int pz = gz + ( corner & 2 ? gd - post : 0 );
			AppendBox( cells, post, gh, post, px, 0, pz );
		}
		AppendBox( cells, gw, post, gd, gx, gh, gz ); // roof frame slab
		AddAnchored( cells, b3_fractureWood );
		cells.clear();
		for ( int x = 0; x < gw; ++x )
			for ( int z = 0; z < gd; ++z )
			{
				bool wall = x < 1 || x >= gw - 1 || z < 1 || z >= gd - 1;
				bool corner = ( x < post || x >= gw - post ) && ( z < post || z >= gd - post );
				if ( wall && !corner )
					for ( int y = 0; y < gh; ++y )
						cells.push_back( b3Vec3i{ gx + x, y, gz + z } );
			}
		AddAnchored( cells, b3_fractureGlass );
		m_targets.push_back( { -8.0f, -31.5f } );
	}

	void TrackShot( b3BodyId body )
	{
		m_shots.push_back( { body, m_ownedVoxelData.back() } );
		if ( (int)m_shots.size() > 24 )
		{
			Shot old = m_shots.front();
			m_shots.erase( m_shots.begin() );
			b3DestroyBody( old.body );
			auto it = std::find( m_ownedVoxelData.begin(), m_ownedVoxelData.end(), old.data );
			if ( it != m_ownedVoxelData.end() )
			{
				b3DestroyVoxelData( *it );
				m_ownedVoxelData.erase( it );
			}
		}
	}

	void Meteor()
	{
		if ( m_targets.empty() )
			return;
		b3Vec2 target = m_targets[(uint32_t)RandomIntRange( 0, (int)m_targets.size() - 1 )];
		float x = target.x + RandomFloatRange( -3.0f, 3.0f );
		float z = target.y + RandomFloatRange( -3.0f, 3.0f );
		b3Vec3 start = { x + RandomFloatRange( -8.0f, 8.0f ), 55.0f, z + RandomFloatRange( -8.0f, 8.0f ) };
		b3Vec3 dir = b3Normalize( b3Vec3{ x, 2.0f, z } - start );
		TrackShot( FireBall( start, 75.0f * dir, 1.8f, 0x805040u, 25.0f ) );
	}

	void Cannonball()
	{
		if ( m_targets.empty() )
			return;
		b3Vec2 target = m_targets[(uint32_t)RandomIntRange( 0, (int)m_targets.size() - 1 )];
		float angle = RandomFloatRange( 0.0f, 2.0f * B3_PI );
		b3Vec3 start = { target.x + 45.0f * cosf( angle ), RandomFloatRange( 8.0f, 16.0f ),
						 target.y + 45.0f * sinf( angle ) };
		b3Vec3 aim = { target.x, RandomFloatRange( 2.0f, 8.0f ), target.y };
		b3Vec3 dir = b3Normalize( aim - start );
		TrackShot( FireBall( start, 85.0f * dir, 0.9f, 0x9090A0u, 25.0f ) );
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			Meteor();
		else if ( key == KEY_T )
			m_bombard = !m_bombard;
	}

	void OnStep() override
	{
		if ( m_bombard && m_stepCount > 30 && ( m_stepCount % 14 ) == 0 )
			Cannonball();
	}

	bool ExtraControls() override
	{
		bool changed = false;
		if ( ImGui::SliderInt( "Voxels per meter", &m_vpm, 2, 6, "%d", ImGuiSliderFlags_AlwaysClamp ) )
		{
			m_voxel = 1.0f / (float)m_vpm;
			Rebuild();
			changed = true;
		}
		changed |= ImGui::Checkbox( "Bombardment", &m_bombard );
		return changed;
	}

	void ExtraHud() override
	{
		HudProfile();
		DrawTextLine( "voxel %.2f m (%d per meter)    Space: meteor    T: bombardment (%s)", m_voxel, m_vpm,
					  m_bombard ? "on" : "off" );
	}

	struct Shot
	{
		b3BodyId body;
		b3VoxelData* data;
	};

	int m_vpm = 4; // voxels per meter
	bool m_bombard = getenv( "VOX_BOMBARD" ) != nullptr;
	std::vector<b3Vec2> m_targets; // building centers (x, z) in meters
	std::vector<Shot> m_shots;	   // live projectiles, oldest first
};

#define VOXEL_SAMPLE( Class, name )                                                                                    \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Voxel", name, Create##Class )

VOXEL_SAMPLE( VoxelTeardown, "Teardown Town" );
