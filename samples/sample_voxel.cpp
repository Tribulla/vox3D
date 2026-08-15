#include "voxel_sample.h"

class VoxelSandbox : public VoxelSample
{
public:
	explicit VoxelSandbox( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -30.0f, 20.0f, 46.0f, { 0.0f, 5.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;

		AppendBox( cells, 12, 6, 2, -20, 0, -1 );
		AddVoxels( cells, Material( b3_fractureBrick ) );

		cells.clear();
		AppendVoxelPyramid( cells, 9, -4, 0, -4 );
		AddVoxels( cells, Material( b3_fractureStone ) );

		cells.clear();
		AppendBox( cells, 3, 16, 3, 14, 0, -1 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );
	}

	void OnKey( int key ) override
	{
		if ( key != KEY_SPACE )
			return;

		static const b3FractureMaterialType mats[] = { b3_fractureConcrete, b3_fractureBrick, b3_fractureStone,
														b3_fractureWood, b3_fractureGlass, b3_fractureMetal };
		int x = -9 + 3 * ( m_dropCount % 7 );
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 3, 3, 3, x, 24, -1 );
		AddVoxels( cells, Material( mats[m_dropCount % 6] ) );
		m_dropCount++;
	}

	void ExtraHud() override
	{
		DrawTextLine( "Space: drop a voxel block    (drops so far: %d)", m_dropCount );
	}

	int m_dropCount = 0;
};

class VoxelMaterials : public VoxelSample
{
public:
	explicit VoxelMaterials( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -20.0f, 14.0f, 60.0f, { 0.0f, 7.0f, 0.0f } );
	}

	void BuildScene() override
	{
		const b3FractureMaterialType mats[6] = { b3_fractureConcrete, b3_fractureBrick, b3_fractureStone,
												 b3_fractureWood,		b3_fractureGlass, b3_fractureMetal };
		for ( int i = 0; i < 6; ++i )
		{
			int ox = -25 + i * 9;
			std::vector<b3Vec3i> cells;
			AppendBox( cells, 3, 15, 3, ox, 0, -1 );
			AddVoxels( cells, Material( mats[i] ) );
		}
	}

	void ExtraHud() override
	{
		DrawTextLine( "L->R: concrete  brick  stone  wood  glass  metal" );
	}
};

class VoxelShapes : public VoxelSample
{
public:
	explicit VoxelShapes( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -25.0f, 22.0f, 58.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;

		AppendVoxelSphere( cells, -20, 5, 0, 5 );
		AddVoxels( cells, Material( b3_fractureStone ) );

		cells.clear();
		AppendVoxelTorus( cells, -6, 6, 0, 6, 2 );
		AddVoxels( cells, Material( b3_fractureConcrete ) );

		cells.clear();
		AppendVoxelCylinderY( cells, 8, 0, 0, 3, 14 );
		AddVoxels( cells, Material( b3_fractureBrick ) );

		cells.clear();
		AppendVoxelPyramid( cells, 11, 16, 0, -5 );
		AddVoxels( cells, Material( b3_fractureStone ) );

		cells.clear();
		AppendText( cells, "VOX3D", -16, 16, -1, 2 );
		AddVoxels( cells, Material( b3_fractureMetal ) );
	}
};

class VoxelSolidVsShell : public VoxelSample
{
public:
	explicit VoxelSolidVsShell( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -28.0f, 16.0f, 52.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;

		b3FractureDef solid = b3DefaultFractureDef();
		solid.merge = true;
		AppendBox( cells, 6, 12, 6, -12, 0, -3 );
		AddVoxels( cells, Material( b3_fractureConcrete ), &solid );

		cells.clear();
		AppendBox( cells, 6, 12, 6, 6, 0, -3 );
		AddVoxels( cells, Material( b3_fractureConcrete ) ); // per-voxel (non-merged)
	}

	void ExtraHud() override
	{
		b3Counters c = b3World_GetCounters( m_worldId );
		DrawTextLine( "left: merged solid    right: per-voxel shell    world shapes = %d", c.shapeCount );
	}
};

class VoxelAnchors : public VoxelSample
{
public:
	explicit VoxelAnchors( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -35.0f, 12.0f, 50.0f, { 0.0f, 12.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendBox( cells, 26, 4, 4, -13, 14, -2 );

		m_anchorThreshold = -9; // pin the two leftmost columns
		b3FractureDef def = b3DefaultFractureDef();
		def.anchor = VoxAnchorLowX;
		def.anchorContext = &m_anchorThreshold;
		AddVoxels( cells, Material( b3_fractureConcrete ), &def );
	}

	void ExtraHud() override
	{
		DrawTextLine( "left columns anchored (fixed support); the rest is a free cantilever" );
	}

	int m_anchorThreshold = 0;
};

class VoxelCastle : public VoxelSample
{
public:
	explicit VoxelCastle( SampleContext* context )
		: VoxelSample( context )
	{
		Boot();
	}

	void SetupCamera() override
	{
		m_camera->SetView( -35.0f, 22.0f, 60.0f, { 0.0f, 6.0f, 0.0f } );
	}

	void BuildScene() override
	{
		std::vector<b3Vec3i> cells;
		AppendVoxelCylinderY( cells, -11, 0, -11, 2, 16 );
		AppendVoxelCylinderY( cells, 11, 0, -11, 2, 16 );
		AppendVoxelCylinderY( cells, -11, 0, 11, 2, 16 );
		AppendVoxelCylinderY( cells, 11, 0, 11, 2, 16 );
		AppendBox( cells, 22, 11, 2, -11, 0, -12 ); // front curtain wall
		AppendBox( cells, 22, 11, 2, -11, 0, 10 );	// back
		AppendBox( cells, 2, 11, 22, -12, 0, -11 ); // left
		AppendBox( cells, 2, 11, 22, 10, 0, -11 );	// right
		AddVoxels( cells, Material( b3_fractureStone ) );
	}

	void ExtraHud() override
	{
		DrawTextLine( "one connected keep (towers + curtain walls) - Shift+Left to breach it" );
	}
};

#define VOXEL_SAMPLE( Class, name )                                                                                    \
	static Sample* Create##Class( SampleContext* context )                                                             \
	{                                                                                                                  \
		return new Class( context );                                                                                   \
	}                                                                                                                  \
	static int sample_##Class = RegisterSample( "Voxel", name, Create##Class )

VOXEL_SAMPLE( VoxelSandbox, "Sandbox" );
VOXEL_SAMPLE( VoxelMaterials, "Materials" );
VOXEL_SAMPLE( VoxelShapes, "Shapes" );
VOXEL_SAMPLE( VoxelSolidVsShell, "Solid vs Shell" );
VOXEL_SAMPLE( VoxelAnchors, "Anchors" );
VOXEL_SAMPLE( VoxelCastle, "Castle" );
