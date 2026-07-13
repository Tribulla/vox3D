#include "voxel_sample.h"

class BenchVoxelWall : public VoxelSample
{
public:
	explicit BenchVoxelWall( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -18.0f, 16.0f, 90.0f, { 0.0f, 12.0f, 0.0f } );
	}

	void BuildScene() override
	{
		int w = ( m_isDebug ? 6 : 8 ) * m_scale;
		int h = ( m_isDebug ? 3 : 4 ) * m_scale;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, h, 3, -w / 2, 0, -1 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );
	}

	bool ExtraControls() override
	{
		if ( ImGui::SliderInt( "Wall scale", &m_scale, 2, 6 ) )
		{
			Rebuild();
			return true;
		}
		return false;
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 12.0f, 40.0f }, { 0.0f, 0.0f, -55.0f }, 3.0f, 0x9090A0, 15.0f );
	}

	void ExtraHud() override
	{
		HudProfile();
		DrawTextLine( "Space/Shift+Left: smash    (raise Wall scale for more voxels)" );
	}

	int m_scale = 5;
};

class BenchVoxelRain : public VoxelSample
{
public:
	explicit BenchVoxelRain( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -20.0f, 22.0f, 70.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
	}

	void OnStep() override
	{
		int period = m_isDebug ? 12 : 6;
		int maxBlocks = m_isDebug ? 50 : 110; // keep total voxels under the render cap
		if ( m_spawn < maxBlocks && m_stepCount > 0 && ( m_stepCount % period ) == 0 )
		{
			int x = -18 + 4 * ( m_spawn % 10 );
			int z = -12 + 4 * ( ( m_spawn / 10 ) % 7 );
			std::vector<b3Vec3i> cells;
			AppendBox( cells, 3, 3, 3, x, 34, z );
			static const b3FractureMaterialType mats[] = { b3_fractureConcrete, b3_fractureBrick, b3_fractureStone };
			AddVoxels( cells, Material( mats[m_spawn % 3] ) );
			m_spawn++;
		}
	}

	void ExtraHud() override
	{
		HudProfile();
		DrawTextLine( "raining blocks: %d    (use Max debris to cap the rubble)", m_spawn );
	}

	int m_spawn = 0;
};

class BenchCollapseStorm : public VoxelSample
{
public:
	explicit BenchCollapseStorm( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -16.0f, 18.0f, 96.0f, { 0.0f, 14.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.strengthScale = 0.15f; // brittle: cannot hold its own weight
		t.warmupFrames = 20;
		t.maxDebris = 2000;
		b3World_SetFractureTuning( m_worldId, t );

		int w = m_isDebug ? 28 : 44;
		int h = m_isDebug ? 16 : 26;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, h, 3, -w / 2, 0, -1 );
		AddVoxels( cells, Material( b3_fractureGlass ) );
	}

	void ExtraHud() override
	{
		HudProfile();
		DrawTextLine( "brittle wall self-collapsing - watch sever/debris ms and the piece count" );
	}
};

static void BuildBusyField( VoxelSample* s, bool debug )
{
	int nx = debug ? 5 : 9;
	int nz = debug ? 5 : 9;
	int layers = debug ? 4 : 6;
	b3FractureDef def = b3DefaultFractureDef();
	def.merge = true;
	for ( int i = 0; i < nx; ++i )
		for ( int k = 0; k < nz; ++k )
			for ( int L = 0; L < layers; ++L )
			{
				std::vector<b3Vec3i> cells;
				// Offset alternate layers so the stack is unstable and keeps moving.
				int ox = -nx * 2 + i * 4 + ( L & 1 );
				int oz = -nz * 2 + k * 4;
				AppendBox( cells, 3, 3, 3, ox, 1 + L * 4, oz );
				s->AddVoxels( cells, s->Material( b3_fractureStone ), &def );
			}
}

class BenchStrideSweep : public VoxelSample
{
public:
	explicit BenchStrideSweep( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 24.0f, 64.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		BuildBusyField( this, m_isDebug );
	}

	void ExtraHud() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		HudProfile();
		DrawTextLine( "Analysis stride = %d  ->  watch the analyze ms scale (set Sleep off for a steady read)",
					  t.analysisStride );
	}
};

class BenchParallelAnalysis : public VoxelSample
{
public:
	explicit BenchParallelAnalysis( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 24.0f, 64.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		BuildBusyField( this, m_isDebug );
	}

	void ExtraHud() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		HudProfile();
		DrawTextLine( "Parallel analysis: %s   (needs Solver > Workers > 1, currently %d)",
					  t.parallelAnalysis ? "on" : "off", m_context->workerCount );
	}
};

class BenchDebrisStorm : public VoxelSample
{
public:
	explicit BenchDebrisStorm( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -18.0f, 16.0f, 80.0f, { 0.0f, 10.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.minFragment = 1;	 // let it go all the way to single-voxel dust
		t.maxDebris = 1200;	 // ... but cap the live count
		t.strengthScale = 0.3f;
		b3World_SetFractureTuning( m_worldId, t );

		int w = m_isDebug ? 26 : 46;
		int h = m_isDebug ? 14 : 22;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, h, 2, -w / 2, 0, -1 );
		AddVoxels( cells, Material( b3_fractureBrick ) );
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 10.0f, 40.0f }, { 0.0f, 0.0f, -60.0f }, 3.5f, 0x9090A0, 18.0f );
	}

	void ExtraHud() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		HudProfile();
		DrawTextLine( "Space: shatter it    Max debris = %d caps the live rubble", t.maxDebris );
	}
};

#define BENCH_SAMPLE( Class, name )                                                                                    \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Benchmark", name, Create##Class )

BENCH_SAMPLE( BenchVoxelWall, "Voxel Wall (Large)" );
BENCH_SAMPLE( BenchVoxelRain, "Voxel Rain" );
BENCH_SAMPLE( BenchCollapseStorm, "Collapse Storm" );
BENCH_SAMPLE( BenchStrideSweep, "Stride Sweep" );
BENCH_SAMPLE( BenchParallelAnalysis, "Parallel Analysis" );
BENCH_SAMPLE( BenchDebrisStorm, "Debris Storm" );
