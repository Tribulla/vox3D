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
		m_camera->SetView( -16.0f, 22.0f, 140.0f, { 0.0f, 26.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.strengthScale = 0.35f; // concrete is strong; soften so the smash volley chews it up
		t.maxDebris = 5000;
		b3World_SetFractureTuning( m_worldId, t );

		int w = ( m_isDebug ? 8 : 12 ) * m_scale;
		int h = ( m_isDebug ? 4 : 8 ) * m_scale;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, h, 4, -w / 2, 0, -1 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );
	}

	bool ExtraControls() override
	{
		if ( ImGui::SliderInt( "Wall scale", &m_scale, 2, 9 ) )
		{
			Rebuild();
			return true;
		}
		return false;
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 16.0f, 44.0f }, { 0.0f, 0.0f, -60.0f }, 4.0f, 0x9090A0, 15.0f );
	}

	void OnStep() override
	{
		if ( m_stepCount >= 40 && m_shots < 10 && ( m_stepCount % 11 ) == 0 )
		{
			float x = -32.0f + 7.0f * (float)m_shots;
			FireBall( { x, 24.0f, 44.0f }, { 0.0f, 0.0f, -80.0f }, 2.0f, 0x9090A0, 30.0f );
			m_shots++;
		}
	}

	void ExtraHud() override
	{
		HudProfile();
		DrawTextLine( "Space/Shift+Left: smash    (raise Wall scale for more voxels)" );
	}

	int m_scale = 7;
	int m_shots = 0;
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
		m_camera->SetView( -20.0f, 26.0f, 110.0f, { 0.0f, 10.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.maxDebris = 8000; // blocks shatter on the long fall; cap the churning dust pile
		b3World_SetFractureTuning( m_worldId, t );
	}

	void OnStep() override
	{
		int period = m_isDebug ? 6 : 2;
		int maxBlocks = m_isDebug ? 150 : 900;
		int perTick = m_isDebug ? 1 : 3;
		if ( m_stepCount == 0 || ( m_stepCount % period ) != 0 )
			return;
		for ( int n = 0; n < perTick && m_spawn < maxBlocks; ++n )
		{
			const int cols = 11;
			int layer = m_spawn / ( cols * cols );
			int x = -25 + 5 * ( m_spawn % cols );
			int z = -25 + 5 * ( ( m_spawn / cols ) % cols );
			std::vector<b3Vec3i> cells;
			AppendBox( cells, 3, 3, 3, x, 44 + 3 * layer, z );
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
		m_camera->SetView( -15.0f, 24.0f, 140.0f, { 0.0f, 22.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.strengthScale = 0.15f; // brittle: cannot hold its own weight
		t.warmupFrames = 20;
		t.maxDebris = 6000;
		b3World_SetFractureTuning( m_worldId, t );

		int w = m_isDebug ? 40 : 90;
		int h = m_isDebug ? 20 : 50;
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
	int nx = debug ? 8 : 16;
	int nz = debug ? 8 : 16;
	int layers = debug ? 5 : 8;
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
		m_camera->SetView( -25.0f, 30.0f, 100.0f, { 0.0f, 10.0f, 0.0f } );
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
		m_camera->SetView( -25.0f, 30.0f, 100.0f, { 0.0f, 10.0f, 0.0f } );
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
		m_camera->SetView( -16.0f, 22.0f, 130.0f, { 0.0f, 18.0f, 0.0f } );
	}

	void BuildScene() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		t.minFragment = 1;	 // let it go all the way to single-voxel dust
		t.maxDebris = 5000;	 // ... but cap the live count
		t.strengthScale = 0.12f;
		b3World_SetFractureTuning( m_worldId, t );

		// 100 x 44 x 2 ~= 8.8k voxels -> shatters into thousands of single-voxel fragments.
		int w = m_isDebug ? 44 : 100;
		int h = m_isDebug ? 18 : 44;
		std::vector<b3Vec3i> cells;
		AppendBox( cells, w, h, 2, -w / 2, 0, -1 );
		AddVoxels( cells, Material( b3_fractureBrick ) );
	}

	void OnKey( int key ) override
	{
		if ( key == KEY_SPACE )
			FireBall( { 0.0f, 14.0f, 44.0f }, { 0.0f, 0.0f, -65.0f }, 4.0f, 0x9090A0, 18.0f );
	}

	void OnStep() override
	{
		if ( m_stepCount >= 40 && m_shots < 12 && ( m_stepCount % 10 ) == 0 )
		{
			float x = -40.0f + 7.5f * (float)m_shots;
			FireBall( { x, 20.0f, 44.0f }, { 0.0f, 0.0f, -85.0f }, 2.0f, 0x9090A0, 30.0f );
			m_shots++;
		}
	}

	void ExtraHud() override
	{
		b3FractureTuning t = b3World_GetFractureTuning( m_worldId );
		HudProfile();
		DrawTextLine( "Space: shatter it    Max debris = %d caps the live rubble", t.maxDebris );
	}

	int m_shots = 0;
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
