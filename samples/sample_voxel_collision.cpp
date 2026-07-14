#include "voxel_sample.h" // pulls in Sample + the Append* voxel cell builders

#include <cstdlib>
#include <cstring>
#include <vector>

enum VoxelBuildMode
{
	VOX_MODE_VOXELSHAPE = 0, // one b3_voxelShape per body (the ported collider)
	VOX_MODE_PERVOXELBOX = 1 // one b3_hullShape box per solid cell (legacy baseline)
};

static inline void AppendPlus( std::vector<b3Vec3i>& cells, int ox, int oy, int oz, int arm, int height )
{
	for ( int y = 0; y < height; ++y )
		for ( int a = -arm; a <= arm; ++a )
		{
			cells.push_back( b3Vec3i{ ox + a, oy + y, oz } ); // bar along X
			if ( a != 0 )
				cells.push_back( b3Vec3i{ ox, oy + y, oz + a } ); // bar along Z
		}
}

static inline bool PlusFootprint( int x, int z, int arm )
{
	return ( z == 0 && x >= -arm && x <= arm ) || ( x == 0 && z >= -arm && z <= arm );
}

class VoxelCollisionSample : public Sample
{
public:
	explicit VoxelCollisionSample( SampleContext* context )
		: Sample( context )
	{
		const char* mode = getenv( "VOX_MODE" );
		if ( mode != nullptr && ( strcmp( mode, "perbox" ) == 0 || strcmp( mode, "box" ) == 0 ) )
			m_mode = VOX_MODE_PERVOXELBOX;

		m_measure = getenv( "VOX_MEASURE" ) != nullptr;
		m_noSleep = getenv( "VOX_NOSLEEP" ) != nullptr;

		if ( m_noSleep )
			b3World_EnableSleeping( m_worldId, false );

		b3World_SetGravity( m_worldId, { 0.0f, -9.81f, 0.0f } );
	}

	~VoxelCollisionSample() override
	{
		FinishRecording();
		if ( B3_IS_NON_NULL( m_worldId ) )
		{
			b3DestroyWorld( m_worldId );
			m_worldId = b3_nullWorldId;
		}
		FreeVoxelData();
	}

	virtual void BuildScene() = 0;
	virtual void ExtraHud()
	{
	}

	void Boot()
	{
		if ( m_context->restart == false )
			SetupCamera();
		m_context->subStepCount = 4;
		if ( const char* ss = getenv( "VOX_SUBSTEPS" ) )
			m_context->subStepCount = atoi( ss );
		if ( m_heightLabel != nullptr )
		{
			const char* h = getenv( "VOX_HEIGHT" );
			if ( h != nullptr )
			{
				int v = atoi( h );
				m_height = v < m_heightMin ? m_heightMin : ( v > m_heightMax ? m_heightMax : v );
			}
		}
		BuildScene();
	}

	virtual void SetupCamera()
	{
		m_camera->SetView( -32.0f, 16.0f, 46.0f, { 0.0f, 6.0f, 0.0f } );
	}

	const char* ModeName() const
	{
		return m_mode == VOX_MODE_VOXELSHAPE ? "b3_voxelShape" : "per-voxel box";
	}

	void AddFlatGround( int halfX, int halfZ, int thick )
	{
		if ( m_mode == VOX_MODE_VOXELSHAPE )
		{
			std::vector<b3Vec3i> cells;
			for ( int x = -halfX; x <= halfX; ++x )
				for ( int z = -halfZ; z <= halfZ; ++z )
					for ( int y = -( thick - 1 ); y <= 0; ++y )
						cells.push_back( b3Vec3i{ x, y, z } );
			AddVoxelBody( cells, { 0.0f, -0.5f, 0.0f }, false, 0u );
		}
		else
		{
			b3BodyDef bd = b3DefaultBodyDef();
			bd.position = b3ToPos( { 0.0f, -0.5f * (float)thick, 0.0f } );
			b3BodyId g = b3CreateBody( m_worldId, &bd );

			b3ShapeDef sd = b3DefaultShapeDef();
			b3BoxHull hull = b3MakeBoxHull( (float)halfX + 0.5f, 0.5f * (float)thick, (float)halfZ + 0.5f );
			b3CreateHullShape( g, &sd, &hull.base );
		}
	}

	b3BodyId AddVoxelBody( const std::vector<b3Vec3i>& cells, b3Vec3 pos, bool dynamic, uint32_t color )
	{
		if ( cells.empty() )
			return b3_nullBodyId;

		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = dynamic ? b3_dynamicBody : b3_staticBody;
		bd.position = b3ToPos( pos );
		b3BodyId body = b3CreateBody( m_worldId, &bd );
		if ( dynamic )
		{
			m_dynamicBodies.push_back( body );
			if ( const char* k = getenv( "VOX_KICK" ) )
			{
				float kv = (float)atof( k );
				b3Body_SetAngularVelocity( body, { kv, 0.0f, kv } );
			}
		}

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.baseMaterial.friction = 0.6f;
		sd.baseMaterial.customColor = color;

		if ( m_mode == VOX_MODE_VOXELSHAPE )
		{
			b3VoxelData* vd = b3CreateVoxelData( cells.data(), (int)cells.size(), m_voxel );
			m_voxelData.push_back( vd ); // keep alive for the shape's lifetime
			b3CreateVoxelShape( body, &sd, vd );
		}
		else
		{
			const float h = 0.5f * m_voxel;
			for ( const b3Vec3i& c : cells )
			{
				b3Vec3 center = { c.x * m_voxel, c.y * m_voxel, c.z * m_voxel };
				b3BoxHull hull = b3MakeOffsetBoxHull( h, h, h, center );
				b3CreateHullShape( body, &sd, &hull.base );
			}
		}
		return body;
	}

	b3BodyId AddConvexBody( int kind, b3Vec3 pos, float r, uint32_t color )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = b3ToPos( pos );
		b3BodyId body = b3CreateBody( m_worldId, &bd );
		m_dynamicBodies.push_back( body );

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.baseMaterial.friction = 0.6f;
		sd.baseMaterial.customColor = color;

		if ( kind == 0 )
		{
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, r };
			b3CreateSphereShape( body, &sd, &sphere );
		}
		else if ( kind == 1 )
		{
			b3BoxHull hull = b3MakeBoxHull( r, r, r );
			b3CreateHullShape( body, &sd, &hull.base );
		}
		else
		{
			b3Capsule cap = { { -r, 0.0f, 0.0f }, { r, 0.0f, 0.0f }, r };
			b3CreateCapsuleShape( body, &sd, &cap );
		}
		return body;
	}

	void Rebuild()
	{
		b3Capacity capacity = {};
		CreateWorld( &capacity );
		FreeVoxelData();
		m_dynamicBodies.clear();
		m_peakStepMs = 0.0f;
		m_peakSpeed = 0.0f;
		if ( m_noSleep )
			b3World_EnableSleeping( m_worldId, false );
		b3World_SetGravity( m_worldId, { 0.0f, -9.81f, 0.0f } );
		BuildScene();
	}

	void Step() override
	{
		Sample::Step();

		b3Profile p = b3World_GetProfile( m_worldId );
		b3Counters c = b3World_GetCounters( m_worldId );

		if ( m_didStep && p.step > m_peakStepMs )
			m_peakStepMs = p.step; // worst-case per-step cost, independent of sleep state

		if ( m_didStep )
		{
			float sp = 0.0f;
			for ( b3BodyId id : m_dynamicBodies )
			{
				float s = b3Length( b3Body_GetLinearVelocity( id ) );
				if ( s > sp )
					sp = s;
			}
			if ( sp > m_peakSpeed )
				m_peakSpeed = sp;
		}

		ResetText();
		DrawTextLine( "Collision mode: %s    sub-steps %d", ModeName(), m_context->subStepCount );
		DrawTextLine( "step %.2f ms | bodies %d  shapes %d  contacts %d | workers %d", p.step, c.bodyCount, c.shapeCount,
					  c.contactCount, m_context->workerCount );
		ExtraHud();
		DrawTextLine( "V: toggle collision mode   (env VOX_MODE=perbox, VOX_MEASURE=1)" );

		if ( m_measure && m_didStep && m_stepCount > 0 && ( m_stepCount % 30 ) == 0 )
		{
			float maxSpeed = 0.0f;
			float maxAng = 0.0f;
			float topY = -1.0e9f;
			for ( b3BodyId id : m_dynamicBodies )
			{
				float sp = b3Length( b3Body_GetLinearVelocity( id ) );
				if ( sp > maxSpeed )
					maxSpeed = sp;
				float w = b3Length( b3Body_GetAngularVelocity( id ) ); // rocking shows here, not in linear speed
				if ( w > maxAng )
					maxAng = w;
				float y = (float)b3Body_GetPosition( id ).y;
				if ( y > topY )
					topY = y;
			}
			int mfEst = 0, heavy7 = 0;
			for ( int b = 0; b < 8; ++b )
				mfEst += ( b + 1 ) * c.manifoldCounts[b];
			heavy7 = c.manifoldCounts[6];
			printf( "[vox-measure] scene=%s mode=%s step=%d stepMs=%.3f peakMs=%.3f contacts=%d mfEst=%d heavy7=%d "
					"awake=%d shapes=%d maxSpeed=%.4f maxAng=%.4f peakSpeed=%.3f topY=%.3f substeps=%d\n",
					m_sceneName, ModeName(), m_stepCount, p.step, m_peakStepMs, c.contactCount, mfEst, heavy7,
					c.awakeContactCount, c.shapeCount, maxSpeed, maxAng, m_peakSpeed, topY, m_context->subStepCount );
			fflush( stdout );
		}
	}

	void Keyboard( int key, int action, int ) override
	{
		if ( action == ACTION_PRESS && key == KEY_V )
		{
			m_mode = m_mode == VOX_MODE_VOXELSHAPE ? VOX_MODE_PERVOXELBOX : VOX_MODE_VOXELSHAPE;
			Rebuild();
		}
	}

	bool DrawControls() override
	{
		int mode = (int)m_mode;
		const char* modes[] = { "b3_voxelShape", "per-voxel box" };
		if ( ImGui::Combo( "Collision mode", &mode, modes, 2 ) )
		{
			m_mode = (VoxelBuildMode)mode;
			Rebuild();
		}
		if ( m_heightLabel != nullptr )
		{
			if ( ImGui::SliderInt( m_heightLabel, &m_height, m_heightMin, m_heightMax ) )
				Rebuild();
		}
		return true;
	}

protected:
	void FreeVoxelData()
	{
		for ( b3VoxelData* v : m_voxelData )
			b3DestroyVoxelData( v );
		m_voxelData.clear();
	}

	float m_voxel = 1.0f;
	VoxelBuildMode m_mode = VOX_MODE_VOXELSHAPE;
	bool m_measure = false;
	bool m_noSleep = false;
	float m_peakStepMs = 0.0f;
	float m_peakSpeed = 0.0f;
	int m_height = 0;
	int m_heightMin = 2;
	int m_heightMax = 20;
	const char* m_heightLabel = nullptr;
	const char* m_sceneName = "collision";
	std::vector<b3VoxelData*> m_voxelData;
	std::vector<b3BodyId> m_dynamicBodies;
};

class CollisionCubeStack : public VoxelCollisionSample
{
public:
	explicit CollisionCubeStack( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "cube_stack";
		m_height = 6;
		m_heightMin = 2;
		m_heightMax = 256;
		m_heightLabel = "Cubes";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -28.0f, 18.0f, 40.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 20, 20, 2 );

		const int n = m_height;		 // cubes
		const int s = 3;			 // cube edge in cells (odd -> centred on the origin)
		const uint32_t palette[6] = { 0xE06666, 0xE0A050, 0xE0D060, 0x60C060, 0x50A0E0, 0xA070D0 };
		for ( int i = 0; i < n; ++i )
		{
			std::vector<b3Vec3i> cells;
			AppendBox( cells, s, s, s, -s / 2, -s / 2, -s / 2 ); // centred cube
			float y = 1.5f + (float)i * ( (float)s + 0.05f );
			AddVoxelBody( cells, { 0.0f, y, 0.0f }, true, palette[i % 6] );
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d solid cubes -- should settle into a straight column and hold", m_height );
	}
};

class CollisionPyramid : public VoxelCollisionSample
{
public:
	explicit CollisionPyramid( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "pyramid";
		m_height = 7;
		m_heightMin = 2;
		m_heightMax = 64;
		m_heightLabel = "Base width";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 14.0f, 44.0f, { 0.0f, 5.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 24, 12, 2 );

		const int base = m_height; // boxes on the bottom row
		const int bs = 2;		   // box edge in cells
		const float pitch = 2.0f;  // touching boxes
		for ( int row = 0; row < base; ++row )
		{
			int count = base - row;
			float y = 1.0f + (float)row * pitch;			  // 2-cube centred rest height is 1.0
			float x0 = -0.5f * (float)( count - 1 ) * pitch;  // centre each row
			for ( int i = 0; i < count; ++i )
			{
				std::vector<b3Vec3i> cells;
				AppendBox( cells, bs, bs, bs, -bs / 2, -bs / 2, -bs / 2 );
				uint32_t color = ( ( row + i ) & 1 ) ? 0xC0C0D0 : 0x8090B0;
				AddVoxelBody( cells, { x0 + (float)i * pitch, y, 0.0f }, true, color );
			}
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d-wide box pyramid -- edges must not spread or slump", m_height );
	}
};

class CollisionVoxelTower : public VoxelCollisionSample
{
public:
	explicit CollisionVoxelTower( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "voxel_tower";
		m_height = 14;
		m_heightMin = 2;
		m_heightMax = 256;
		m_heightLabel = "Voxels";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -20.0f, 10.0f, 26.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 10, 10, 2 );

		const int n = m_height;
		for ( int i = 0; i < n; ++i )
		{
			std::vector<b3Vec3i> cells;
			cells.push_back( b3Vec3i{ 0, 0, 0 } ); // one voxel
			float y = 0.5f + (float)i * 1.02f;		// unit cube rest height 0.5, tiny gap
			uint32_t color = ( i & 1 ) ? 0x60C0A0 : 0xC0A060;
			AddVoxelBody( cells, { 0.0f, y, 0.0f }, true, color );
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d single voxels -- a 1-wide needle; drift/topple = instability", m_height );
	}
};

class CollisionJenga : public VoxelCollisionSample
{
public:
	explicit CollisionJenga( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "jenga";
		m_height = 14;
		m_heightMin = 2;
		m_heightMax = 256;
		m_heightLabel = "Layers";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -24.0f, 14.0f, 30.0f, { 0.0f, 8.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 12, 12, 2 );

		const int layers = m_height;
		const int len = 3; // block length in cells (3 blocks -> 3x3 footprint)
		for ( int layer = 0; layer < layers; ++layer )
		{
			float y = 0.5f + (float)layer * 1.02f; // unit-tall blocks, tiny gap
			bool alongX = ( layer & 1 ) == 0;
			uint32_t color = alongX ? 0xD0B080 : 0xB0C0D0;
			for ( int b = -1; b <= 1; ++b )
			{
				std::vector<b3Vec3i> cells;
				if ( alongX )
					AppendBox( cells, len, 1, 1, -len / 2, 0, 0 ); // long axis X
				else
					AppendBox( cells, 1, 1, len, 0, 0, -len / 2 ); // long axis Z
				float x = alongX ? 0.0f : (float)b;
				float z = alongX ? (float)b : 0.0f;
				AddVoxelBody( cells, { x, y, z }, true, color );
			}
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d layers x 3 blocks, alternating 90 deg -- must stand square", m_height );
	}
};

class CollisionExactFit : public VoxelCollisionSample
{
public:
	explicit CollisionExactFit( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "exact_fit";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -18.0f, 14.0f, 30.0f, { 0.0f, 4.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 14, 14, 2 );

		const int arm = 2;	 // plus arm reach
		const int half = 6;	 // slab footprint half-extent
		const int thick = 6; // slab / peg height

		std::vector<b3Vec3i> slab;
		for ( int x = -half; x <= half; ++x )
			for ( int z = -half; z <= half; ++z )
			{
				if ( PlusFootprint( x, z, arm ) )
					continue; // carve the hole
				for ( int y = 0; y < thick; ++y )
					slab.push_back( b3Vec3i{ x, y, z } );
			}
		AddVoxelBody( slab, { 0.0f, 0.5f, 0.0f }, false, 0x707078u );

		std::vector<b3Vec3i> peg;
		AppendPlus( peg, 0, 0, 0, arm, thick );
		AddVoxelBody( peg, { 0.0f, (float)thick + 2.5f, 0.0f }, true, 0xE0C040u );
	}

	void ExtraHud() override
	{
		DrawTextLine( "plus peg into an exact plus hole -- must slot flush, not jam" );
	}
};

class CollisionShapeStack : public VoxelCollisionSample
{
public:
	explicit CollisionShapeStack( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "shape_stack";
		m_height = 5;
		m_heightMin = 2;
		m_heightMax = 384;
		m_heightLabel = "Shapes";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -26.0f, 14.0f, 34.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 14, 14, 2 );

		const int n = m_height;
		const int arm = 2;
		const int h = 2; // plus height in cells
		const uint32_t palette[5] = { 0xE07070, 0x70C0E0, 0xE0C060, 0x80D080, 0xC090E0 };
		for ( int i = 0; i < n; ++i )
		{
			std::vector<b3Vec3i> cells;
			AppendPlus( cells, 0, 0, 0, arm, h );
			float y = 0.5f + (float)i * ( (float)h + 0.1f );
			AddVoxelBody( cells, { 0.0f, y, 0.0f }, true, palette[i % 5] );
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d plus-shaped bodies stacked -- interlocking-shape stability", m_height );
	}
};

class CollisionPile : public VoxelCollisionSample
{
public:
	explicit CollisionPile( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "pile";
		m_height = 5; // grid is m_height x m_height
		m_heightMin = 2;
		m_heightMax = 16;
		m_heightLabel = "Grid N";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -45.0f, 32.0f, 78.0f, { 0.0f, 2.0f, 0.0f } );
	}

	void BuildScene() override
	{
		const int N = m_height;
		const int stackH = 3;
		const int s = 3; // cube edge in cells
		AddFlatGround( N * 2 + 4, N * 2 + 4, 2 );

		const uint32_t palette[6] = { 0xE06666, 0xE0A050, 0xE0D060, 0x60C060, 0x50A0E0, 0xA070D0 };
		for ( int gx = 0; gx < N; ++gx )
			for ( int gz = 0; gz < N; ++gz )
				for ( int level = 0; level < stackH; ++level )
				{
					std::vector<b3Vec3i> cells;
					AppendBox( cells, s, s, s, -s / 2, -s / 2, -s / 2 );
					float x = (float)( gx - N / 2 ) * 4.0f;
					float z = (float)( gz - N / 2 ) * 4.0f;
					float y = 1.5f + (float)level * ( (float)s + 0.05f );
					AddVoxelBody( cells, { x, y, z }, true, palette[( gx + gz + level ) % 6] );
				}
	}

	void ExtraHud() override
	{
		b3Counters c = b3World_GetCounters( m_worldId );
		DrawTextLine( "%dx%d grid of 3-cube stacks = %d bodies, %d shapes -- toggle mode (V) to A/B the cost",
					  m_height, m_height, c.bodyCount, c.shapeCount );
	}
};

class CollisionConvexDrop : public VoxelCollisionSample
{
public:
	explicit CollisionConvexDrop( SampleContext* context )
		: VoxelCollisionSample( context )
	{
		m_sceneName = "convex_drop";
		m_height = 5; // grid is m_height x m_height convex bodies
		m_heightMin = 2;
		m_heightMax = 12;
		m_heightLabel = "Grid N";
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -36.0f, 26.0f, 60.0f, { 0.0f, 3.0f, 0.0f } );
	}

	void BuildScene() override
	{
		AddFlatGround( 18, 18, 2 );

		std::vector<b3Vec3i> step;
		AppendBox( step, 8, 3, 8, -4, 1, -4 );
		AddVoxelBody( step, { 0.0f, 0.0f, 0.0f }, false, 0x606068u );
		step.clear();
		AppendBox( step, 5, 5, 5, 7, 1, 7 );
		AddVoxelBody( step, { 0.0f, 0.0f, 0.0f }, false, 0x606068u );

		const int N = m_height;
		const uint32_t palette[3] = { 0xE07060, 0x60A0E0, 0x70C070 };
		for ( int gx = 0; gx < N; ++gx )
			for ( int gz = 0; gz < N; ++gz )
			{
				int kind = ( gx + gz ) % 3; // 0 sphere, 1 box, 2 capsule
				float x = (float)( gx - N / 2 ) * 3.0f;
				float z = (float)( gz - N / 2 ) * 3.0f;
				float y = 9.0f + (float)( ( gx * 7 + gz ) % 4 ); // staggered drop heights
				AddConvexBody( kind, { x, y, z }, 0.7f, palette[kind] );
			}
	}

	void ExtraHud() override
	{
		DrawTextLine( "%d convex bodies (sphere/box/capsule) on voxel terrain -- Phase 7 convex-vs-voxel",
					  (int)m_dynamicBodies.size() );
	}
};

#define COLLISION_SAMPLE( Class, name )                                                                                \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Collision", name, Create##Class )

COLLISION_SAMPLE( CollisionCubeStack, "Cube Stack" );
COLLISION_SAMPLE( CollisionPyramid, "Pyramid" );
COLLISION_SAMPLE( CollisionVoxelTower, "Voxel Tower" );
COLLISION_SAMPLE( CollisionJenga, "Jenga Tower" );
COLLISION_SAMPLE( CollisionExactFit, "Exact Fit (slot)" );
COLLISION_SAMPLE( CollisionShapeStack, "Shape Stack" );
COLLISION_SAMPLE( CollisionPile, "Voxel Pile (scale)" );
COLLISION_SAMPLE( CollisionConvexDrop, "Convex Drop" );
