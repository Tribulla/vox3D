#include "voxel_sample.h"

struct PanelEnds
{
	int lo, hi;
};
static bool VoxAnchorPanelEnds( b3Vec3i cell, void* context )
{
	PanelEnds* e = (PanelEnds*)context;
	return cell.x < e->lo || cell.x >= e->hi;
}

class ImpactWallSmash : public VoxelSample
{
public:
	explicit ImpactWallSmash( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -20.0f, 12.0f, 44.0f, { 0.0f, 5.0f, 8.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 24, 11, 2, -12, 0, -1 );
		AddVoxels( cells, Material( b3_fractureBrick ) );
	}

	void Fire()
	{
		FireBall( { 0.0f, 5.0f, 26.0f }, { 0.0f, 0.0f, -46.0f }, 2.5f, 0xC0C0C8 );
	}

	void OnStep() override
	{
		if ( m_selfRun && m_stepCount > 0 && ( m_stepCount % 150 ) == 0 )
			Fire();
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			Fire();
	}

	bool ExtraControls() override
	{
		ImGui::Checkbox( "Self-running", &m_selfRun );
		return true;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: fire a ball    Self-running: %s", m_selfRun ? "on" : "off" );
	}

	bool m_selfRun = true;
};

class ImpactBulletHoles : public VoxelSample
{
public:
	explicit ImpactBulletHoles( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( 0.0f, 4.0f, 40.0f, { 0.0f, 7.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 22, 14, 3, -11, 0, -1 );
		m_ends = { -9, 9 };
		b3FractureDef def = b3DefaultFractureDef();
		def.anchor = VoxAnchorPanelEnds;
		def.anchorContext = &m_ends;
		AddVoxels( cells, Material( b3_fractureConcrete ), &def );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		float x = -6.0f + 3.0f * ( m_shot % 5 );
		float y = 4.0f + 2.0f * ( ( m_shot / 5 ) % 4 );
		FireBall( { x, y, 30.0f }, { 0.0f, 0.0f, -70.0f }, 0.8f, 0xE0D060, 12.0f );
		m_shot++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: fire fast rounds (%d)   tune Impact radius / bearing", m_shot );
	}

	PanelEnds m_ends = { 0, 0 };
	int m_shot = 0;
};

class ImpactDemolition : public VoxelSample
{
public:
	explicit ImpactDemolition( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 14.0f, 62.0f, { 0.0f, 12.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBoxShell( cells, 14, 26, 10, -7, 0, -5, 2 );
		for ( int y = 6; y < 26; y += 6 )
			AppendBox( cells, 14, 1, 10, -7, y, -5 );
		AddVoxels( cells, Material( b3_fractureBrick ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		float y = 4.0f + 3.0f * ( m_swing % 6 );
		FireBall( { 0.0f, y, 34.0f }, { 0.0f, 0.0f, -40.0f }, 3.0f, 0x9090A0, 15.0f );
		m_swing++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: swing the wrecking ball (%d)", m_swing );
	}

	int m_swing = 0;
};

class ImpactCrateSmash : public VoxelSample
{
public:
	explicit ImpactCrateSmash( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 16.0f, 40.0f, { 0.0f, 4.0f, 0.0f } );
	}

	void DropCrate( int x )
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 6, 6, 6, x - 3, 18, -3 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true; // solid crate
		AddVoxels( cells, Material( b3_fractureStone ), &def );
	}

	void BuildScene() override
	{
		DropCrate( 0 );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		DropCrate( -8 + 4 * ( m_drops % 5 ) );
		m_drops++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: drop another crate (%d)", m_drops );
	}

	int m_drops = 0;
};

class ImpactCrush : public VoxelSample
{
public:
	explicit ImpactCrush( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 12.0f, 48.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> pillar;
		AppendBox( pillar, 2, 10, 5, -6, 0, -2 );
		AddVoxels( pillar, Material( b3_fractureConcrete ) );
		pillar.clear();
		AppendBox( pillar, 2, 10, 5, 4, 0, -2 );
		AddVoxels( pillar, Material( b3_fractureConcrete ) );

		std::vector<b3Vec3i> beam;
		AppendBox( beam, 16, 2, 5, -8, 10, -2 );
		AddVoxels( beam, Material( b3_fractureConcrete ) );

		std::vector<b3Vec3i> cap;
		AppendBox( cap, 14, 6, 5, -7, 12, -2 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true; // one heavy solid block
		AddVoxels( cap, Material( b3_fractureMetal ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "the metal cap's weight overloads and crushes the pillars" );
	}
};

class ImpactDominoes : public VoxelSample
{
public:
	explicit ImpactDominoes( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -68.0f, 18.0f, 76.0f, { 0.0f, 4.0f, 0.0f } );
	}

	void BuildScene() override
	{
		m_count = m_isDebug ? 9 : 16;
		m_firstZ = -( m_count - 1 ) * 5 / 2; // centre the row so it stays on the ground
		for ( int i = 0; i < m_count; ++i )
		{
			std::vector<b3Vec3i> cells;
			AppendBox( cells, 5, 12, 1, -2, 0, m_firstZ + i * 5 );
			AddVoxels( cells, Material( i % 2 ? b3_fractureBrick : b3_fractureConcrete ) );
		}
	}

	void Topple()
	{
		FireBall( { 0.0f, 9.0f, (float)m_firstZ - 12.0f }, { 0.0f, -1.0f, 26.0f }, 1.6f, 0xE0D060, 6.0f );
	}

	void OnStep() override
	{
		if ( m_selfRun && m_stepCount == 40 )
			Topple();
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			Topple();
	}

	bool ExtraControls() override
	{
		ImGui::Checkbox( "Self-running", &m_selfRun );
		return true;
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d dominoes    Space: topple    Self-running: %s", m_count, m_selfRun ? "on" : "off" );
	}

	int m_count = 16;
	int m_firstZ = 0;
	bool m_selfRun = true;
};

class ImpactColonnade : public VoxelSample
{
public:
	explicit ImpactColonnade( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -20.0f, 12.0f, 62.0f, { 0.0f, 9.0f, 0.0f } );
	}

	void BuildScene() override
	{
		for ( int i = 0; i < 5; ++i )
		{
			std::vector<b3Vec3i> col;
			AppendBox( col, 3, 14, 3, -12 + i * 6, 0, -1 );
			AddVoxels( col, Material( b3_fractureStone ) );
		}
		std::vector<b3Vec3i> roof;
		AppendBox( roof, 30, 6, 5, -15, 14, -2 );
		AddVoxels( roof, Material( b3_fractureConcrete ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		float x = -12.0f + 6.0f * ( m_shot % 5 );
		FireBall( { x, 5.0f, 26.0f }, { 0.0f, 0.0f, -55.0f }, 2.0f, 0x9090A0, 12.0f );
		m_shot++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: shoot out a column (%d)", m_shot );
	}

	int m_shot = 0;
};

class ImpactCratering : public VoxelSample
{
public:
	explicit ImpactCratering( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -28.0f, 34.0f, 56.0f, { 0.0f, 2.0f, 0.0f } );
	}

	void BuildScene() override
	{
		int w = m_isDebug ? 20 : 30, d = m_isDebug ? 16 : 24;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, 5, d, -w / 2, 0, -d / 2 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		float x = -10.0f + 5.0f * ( m_shot % 5 ), z = -8.0f + 4.0f * ( ( m_shot / 5 ) % 5 );
		FireBall( { x, 22.0f, z }, { 0.0f, -60.0f, 0.0f }, 1.8f, 0xE0D060, 14.0f );
		m_shot++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: bombard the slab (%d)    Shift+Left also shoots", m_shot );
	}

	int m_shot = 0;
};

#define IMPACT_SAMPLE( Class, name )                                                                                   \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Impact", name, Create##Class )

IMPACT_SAMPLE( ImpactWallSmash, "Wall Smash" );
IMPACT_SAMPLE( ImpactBulletHoles, "Bullet Holes" );
IMPACT_SAMPLE( ImpactDemolition, "Demolition" );
IMPACT_SAMPLE( ImpactCrateSmash, "Crate Smash (solid)" );
IMPACT_SAMPLE( ImpactCrush, "Crush" );
IMPACT_SAMPLE( ImpactDominoes, "Dominoes" );
IMPACT_SAMPLE( ImpactColonnade, "Colonnade" );
IMPACT_SAMPLE( ImpactCratering, "Cratering" );
