#include "voxel_sample.h"

struct EndsContext
{
	int lo, hi;
};
static bool VoxAnchorEndsX( b3Vec3i cell, void* context )
{
	EndsContext* e = (EndsContext*)context;
	return cell.x < e->lo || cell.x >= e->hi;
}

class StressCantilever : public VoxelSample
{
public:
	explicit StressCantilever( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -35.0f, 10.0f, 54.0f, { 4.0f, 14.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, m_length, 4, 4, -6, 14, -2 );
		m_anchorThreshold = -2; // the two columns inside the "wall"
		b3FractureDef def = b3DefaultFractureDef();
		def.anchor = VoxAnchorLowX;
		def.anchorContext = &m_anchorThreshold;
		AddVoxels( cells, Material( b3_fractureConcrete ), &def );
	}

	bool ExtraControls() override
	{
		if ( ImGui::SliderInt( "Length", &m_length, 8, 40 ) )
		{
			Rebuild();
			return true;
		}
		return false;
	}

	void ExtraHud() override
	{
		DrawTextLine( "cantilever length %d - lower Strength scale to snap it at the root", m_length );
	}

	int m_length = 20;
	int m_anchorThreshold = 0;
};

class StressTower : public VoxelSample
{
public:
	explicit StressTower( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 8.0f, 60.0f, { 0.0f, 16.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 4, m_height, 4, -2, 0, -2 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );
	}

	bool ExtraControls() override
	{
		if ( ImGui::SliderInt( "Height", &m_height, 10, 44 ) )
		{
			Rebuild();
			return true;
		}
		return false;
	}

	void ExtraHud() override
	{
		DrawTextLine( "tower height %d voxels - base carries the whole column's weight", m_height );
	}

	int m_height = 28;
};

class StressBridge : public VoxelSample
{
public:
	explicit StressBridge( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 14.0f, 56.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> pier;
		AppendBox( pier, 3, 8, 4, -12, 0, -2 );
		AddVoxels( pier, Material( b3_fractureStone ) );
		pier.clear();
		AppendBox( pier, 3, 8, 4, 9, 0, -2 );
		AddVoxels( pier, Material( b3_fractureStone ) );

		std::vector<b3Vec3i> deck;
		AppendBox( deck, 24, 2, 4, -12, 8, -2 );
		AddVoxels( deck, Material( b3_fractureConcrete ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		std::vector<b3Vec3i> block;
		AppendBox( block, 3, 3, 3, -1, 16, -1 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true;
		AddVoxels( block, Material( b3_fractureMetal ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: drop a weight on mid-span" );
	}
};

class StressArch : public VoxelSample
{
public:
	explicit StressArch( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 10.0f, 50.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendVoxelArch( cells, -13, 0, -3, 27, 14, 3, 6 );
		AddVoxels( cells, Material( b3_fractureStone ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		std::vector<b3Vec3i> block;
		AppendBox( block, 4, 4, 4, -2, 20, -2 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true;
		AddVoxels( block, Material( b3_fractureMetal ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: load the crown - watch thrust build toward the feet" );
	}
};

class StressBeamSnap : public VoxelSample
{
public:
	explicit StressBeamSnap( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 12.0f, 52.0f, { 0.0f, 12.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 28, 3, 3, -14, 12, -1 );
		m_ends = { -12, 12 }; // pin the outermost two columns at each end
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true; // a solid rigid body
		def.anchor = VoxAnchorEndsX;
		def.anchorContext = &m_ends;
		AddVoxels( cells, Material( b3_fractureConcrete ), &def );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;
		std::vector<b3Vec3i> block;
		AppendBox( block, 4, 4, 4, -2, 18, -2 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true;
		AddVoxels( block, Material( b3_fractureMetal ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "solid beam, both ends fixed - Space drops a load on the centre" );
	}

	EndsContext m_ends = { 0, 0 };
};

class StressOverload : public VoxelSample
{
public:
	explicit StressOverload( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -28.0f, 12.0f, 50.0f, { 0.0f, 9.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> col;
		AppendBox( col, 3, 10, 4, -11, 0, -2 );
		AddVoxels( col, Material( b3_fractureStone ) );
		col.clear();
		AppendBox( col, 3, 10, 4, 8, 0, -2 );
		AddVoxels( col, Material( b3_fractureStone ) );

		std::vector<b3Vec3i> lintel;
		AppendBox( lintel, 22, 3, 4, -11, 10, -2 );
		AddVoxels( lintel, Material( b3_fractureConcrete ) );

		std::vector<b3Vec3i> load;
		AppendBox( load, 6, 5, 4, -3, 13, -2 );
		b3FractureDef def = b3DefaultFractureDef();
		def.merge = true;
		AddVoxels( load, Material( b3_fractureMetal ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "preloaded lintel - sweep Strength scale to move the stress field and fail it" );
	}
};

class StressJenga : public VoxelSample
{
public:
	explicit StressJenga( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 14.0f, 52.0f, { 0.0f, 14.0f, 0.0f } );
	}

	void BuildScene() override
	{
		int layers = m_isDebug ? 9 : 15;
		for ( int L = 0; L < layers; ++L )
		{
			int y = L * 2;
			for ( int b = 0; b < 3; ++b )
			{
				std::vector<b3Vec3i> cells;
				if ( L % 2 == 0 )
					AppendBox( cells, 9, 2, 3, -4, y, -4 + b * 3 );
				else
					AppendBox( cells, 3, 2, 9, -4 + b * 3, y, -4 );
				AddVoxels( cells, Material( b3_fractureWood ) );
			}
		}
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 6.0f, 20.0f }, { 0.0f, 0.0f, -45.0f }, 1.2f, 0x9090A0, 10.0f );
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: knock a block out of the stack" );
	}
};

class StressSilo : public VoxelSample
{
public:
	explicit StressSilo( SampleContext* context )
		: VoxelSample( context )
	{
		m_stressDefault = true;
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 10.0f, 56.0f, { 0.0f, 14.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendVoxelPipe( cells, 0, 0, 0, 5, 2, m_isDebug ? 18 : 30 );
		AddVoxels( cells, Material( b3_fractureBrick ) );
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 4.0f, 26.0f }, { 0.0f, 0.0f, -55.0f }, 2.5f, 0x9090A0, 14.0f );
	}

	void ExtraHud() override
	{
		DrawTextLine( "hollow chimney in compression - Space: blast the base and fell it" );
	}
};

#define STRESS_SAMPLE( Class, name )                                                                                   \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Stress", name, Create##Class )

STRESS_SAMPLE( StressCantilever, "Cantilever" );
STRESS_SAMPLE( StressTower, "Tower" );
STRESS_SAMPLE( StressBridge, "Bridge Span" );
STRESS_SAMPLE( StressArch, "Arch" );
STRESS_SAMPLE( StressBeamSnap, "Beam Snap (solid)" );
STRESS_SAMPLE( StressOverload, "Overload" );
STRESS_SAMPLE( StressJenga, "Jenga" );
STRESS_SAMPLE( StressSilo, "Silo" );
