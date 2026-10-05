#include "voxel_shape.h"

#include "core.h"
#include "box3d/constants.h"

#include <math.h>
#include <string.h>

#define B3_VOXEL_CHUNK_BITS 4
#define B3_VOXEL_CHUNK_SIZE 16 // 1 << B3_VOXEL_CHUNK_BITS
#define B3_VOXEL_CHUNK_MASK 15 // B3_VOXEL_CHUNK_SIZE - 1
#define B3_VOXEL_CHUNK_CELLS 4096 // 16^3

typedef struct b3VoxelChunk
{
	uint8_t solid[B3_VOXEL_CHUNK_CELLS]; // 0 = air, 1 = solid; index (lx<<8)|(ly<<4)|lz
	uint16_t* geometry; // allocated only for chunks containing non-cube cells
	b3Vec3i* occupied;					 // compact solid local cells (0..15 per axis)
	int occupiedCount;
	int occupiedCapacity;
	b3AABB solidBounds; // shape-local world-unit bounds of this chunk's solid cells
} b3VoxelChunk;

typedef struct b3VoxelChunkSlot
{
	uint64_t key; // 0 = empty (real keys are never 0, see b3Voxel_packChunk)
	int index;
} b3VoxelChunkSlot;

typedef struct b3VoxelGeometry
{
	b3VoxelSubBox* boxes;
	int count;
} b3VoxelGeometry;

struct b3VoxelData
{
	float voxelSize;
	float invVoxelSize;
	int cellCount; // total solid cells
	uint32_t hash;

	b3AABB localBounds;		  // shape-local world-unit bounds of all solid cells
	bool hasBounds;

	b3VoxelChunk* chunks;	  // dense array of chunks
	int chunkCount;
	int chunkCapacity;

	b3VoxelChunkSlot* slots;  // hash: chunkKey -> chunks index
	int slotCap;			  // power of two
	b3VoxelGeometry* geometries;
	int geometryCount;
	int geometryCapacity;
	b3Vec3 geometryPadding; // conservative extension beyond a cell's cube
};

static inline int b3Voxel_iaxis( b3Vec3i c, int a )
{
	return ( &c.x )[a];
}

static inline b3Vec3i b3Voxel_chunkOf( b3Vec3i cell )
{
	return (b3Vec3i){ cell.x >> B3_VOXEL_CHUNK_BITS, cell.y >> B3_VOXEL_CHUNK_BITS, cell.z >> B3_VOXEL_CHUNK_BITS };
}

static inline int b3Voxel_localIndex( int lx, int ly, int lz )
{
	return ( lx << 8 ) | ( ly << 4 ) | lz;
}

static uint64_t b3Voxel_packChunk( b3Vec3i c )
{
	const int64_t OFF = 1 << 20;
	uint64_t x = (uint64_t)( (int64_t)c.x + OFF );
	uint64_t y = (uint64_t)( (int64_t)c.y + OFF );
	uint64_t z = (uint64_t)( (int64_t)c.z + OFF );
	return ( x << 42 ) | ( y << 21 ) | z; // never 0 for the representable range
}

static uint32_t b3Voxel_mixU64( uint64_t k )
{
	k ^= k >> 33;
	k *= 0xff51afd7ed558ccdULL;
	k ^= k >> 33;
	k *= 0xc4ceb9fe1a85ec53ULL;
	k ^= k >> 33;
	return (uint32_t)k;
}

static void b3Voxel_growSlots( b3VoxelData* v )
{
	int newCap = v->slotCap ? 2 * v->slotCap : 16;
	b3VoxelChunkSlot* old = v->slots;
	int oldCap = v->slotCap;
	v->slots = (b3VoxelChunkSlot*)b3AllocZeroed( (size_t)newCap * sizeof( b3VoxelChunkSlot ) );
	v->slotCap = newCap;
	uint32_t mask = (uint32_t)( newCap - 1 );
	for ( int i = 0; i < oldCap; ++i )
	{
		if ( old[i].key == 0 )
			continue;
		uint32_t s = b3Voxel_mixU64( old[i].key ) & mask;
		while ( v->slots[s].key != 0 )
			s = ( s + 1 ) & mask;
		v->slots[s] = old[i];
	}
	if ( old )
		b3Free( old, (size_t)oldCap * sizeof( b3VoxelChunkSlot ) );
}

static int b3Voxel_findChunk( const b3VoxelData* v, uint64_t key )
{
	if ( v->slotCap == 0 )
		return -1;
	uint32_t mask = (uint32_t)( v->slotCap - 1 );
	uint32_t s = b3Voxel_mixU64( key ) & mask;
	while ( v->slots[s].key != 0 )
	{
		if ( v->slots[s].key == key )
			return v->slots[s].index;
		s = ( s + 1 ) & mask;
	}
	return -1;
}

static int b3Voxel_getOrCreateChunk( b3VoxelData* v, b3Vec3i chunkPos )
{
	uint64_t key = b3Voxel_packChunk( chunkPos );
	if ( v->slotCap == 0 || 10 * ( v->chunkCount + 1 ) >= 7 * v->slotCap )
		b3Voxel_growSlots( v );

	uint32_t mask = (uint32_t)( v->slotCap - 1 );
	uint32_t s = b3Voxel_mixU64( key ) & mask;
	while ( v->slots[s].key != 0 )
	{
		if ( v->slots[s].key == key )
			return v->slots[s].index;
		s = ( s + 1 ) & mask;
	}

	if ( v->chunkCount == v->chunkCapacity )
	{
		int nc = v->chunkCapacity ? 2 * v->chunkCapacity : 8;
		b3VoxelChunk* grown = (b3VoxelChunk*)b3Alloc( (size_t)nc * sizeof( b3VoxelChunk ) );
		if ( v->chunks )
		{
			memcpy( grown, v->chunks, (size_t)v->chunkCount * sizeof( b3VoxelChunk ) );
			b3Free( v->chunks, (size_t)v->chunkCapacity * sizeof( b3VoxelChunk ) );
		}
		v->chunks = grown;
		v->chunkCapacity = nc;
	}
	int idx = v->chunkCount++;
	b3VoxelChunk* chunk = v->chunks + idx;
	memset( chunk->solid, 0, sizeof( chunk->solid ) );
	chunk->geometry = NULL;
	chunk->occupied = NULL;
	chunk->occupiedCount = 0;
	chunk->occupiedCapacity = 0;
	chunk->solidBounds = (b3AABB){ { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };

	v->slots[s].key = key;
	v->slots[s].index = idx;
	return idx;
}

b3VoxelData* b3CreateVoxelData( const b3Vec3i* cells, int count, float voxelSize )
{
	if ( cells == NULL || count <= 0 || !( voxelSize > 0.0f ) )
		return NULL;

	b3VoxelData* v = (b3VoxelData*)b3AllocZeroed( sizeof( b3VoxelData ) );
	v->voxelSize = voxelSize;
	v->invVoxelSize = 1.0f / voxelSize;
	v->localBounds = (b3AABB){ { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };

	float half = 0.5f * voxelSize;
	uint32_t h = 2166136261u; // FNV-1a over (cells, voxelSize)

	for ( int i = 0; i < count; ++i )
	{
		b3Vec3i cell = cells[i];
		b3Vec3i chunkPos = b3Voxel_chunkOf( cell );
		int ci = b3Voxel_getOrCreateChunk( v, chunkPos );
		b3VoxelChunk* chunk = v->chunks + ci;

		int lx = cell.x & B3_VOXEL_CHUNK_MASK;
		int ly = cell.y & B3_VOXEL_CHUNK_MASK;
		int lz = cell.z & B3_VOXEL_CHUNK_MASK;
		int li = b3Voxel_localIndex( lx, ly, lz );
		if ( chunk->solid[li] )
			continue; // duplicate cell
		chunk->solid[li] = 1;

		if ( chunk->occupiedCount == chunk->occupiedCapacity )
		{
			int nc = chunk->occupiedCapacity ? 2 * chunk->occupiedCapacity : 32;
			b3Vec3i* grown = (b3Vec3i*)b3Alloc( (size_t)nc * sizeof( b3Vec3i ) );
			if ( chunk->occupied )
			{
				memcpy( grown, chunk->occupied, (size_t)chunk->occupiedCount * sizeof( b3Vec3i ) );
				b3Free( chunk->occupied, (size_t)chunk->occupiedCapacity * sizeof( b3Vec3i ) );
			}
			chunk->occupied = grown;
			chunk->occupiedCapacity = nc;
		}
		chunk->occupied[chunk->occupiedCount++] = (b3Vec3i){ lx, ly, lz };

		b3Vec3 lo = { cell.x * voxelSize - half, cell.y * voxelSize - half, cell.z * voxelSize - half };
		b3Vec3 hi = { cell.x * voxelSize + half, cell.y * voxelSize + half, cell.z * voxelSize + half };
		b3AABB* cb = &chunk->solidBounds;
		cb->lowerBound.x = b3MinFloat( cb->lowerBound.x, lo.x );
		cb->lowerBound.y = b3MinFloat( cb->lowerBound.y, lo.y );
		cb->lowerBound.z = b3MinFloat( cb->lowerBound.z, lo.z );
		cb->upperBound.x = b3MaxFloat( cb->upperBound.x, hi.x );
		cb->upperBound.y = b3MaxFloat( cb->upperBound.y, hi.y );
		cb->upperBound.z = b3MaxFloat( cb->upperBound.z, hi.z );
		b3AABB* sb = &v->localBounds;
		sb->lowerBound.x = b3MinFloat( sb->lowerBound.x, lo.x );
		sb->lowerBound.y = b3MinFloat( sb->lowerBound.y, lo.y );
		sb->lowerBound.z = b3MinFloat( sb->lowerBound.z, lo.z );
		sb->upperBound.x = b3MaxFloat( sb->upperBound.x, hi.x );
		sb->upperBound.y = b3MaxFloat( sb->upperBound.y, hi.y );
		sb->upperBound.z = b3MaxFloat( sb->upperBound.z, hi.z );

		h ^= (uint32_t)cell.x; h *= 16777619u;
		h ^= (uint32_t)cell.y; h *= 16777619u;
		h ^= (uint32_t)cell.z; h *= 16777619u;
		v->cellCount++;
	}

	h ^= (uint32_t)( voxelSize * 1024.0f ); h *= 16777619u;
	h ^= h >> 16;
	v->hash = h ? h : 1u;
	v->hasBounds = v->cellCount > 0;
	if ( !v->hasBounds )
		v->localBounds = (b3AABB){ b3Vec3_zero, b3Vec3_zero };

	return v;
}

void b3DestroyVoxelData( b3VoxelData* v )
{
	if ( v == NULL )
		return;
	for ( int i = 0; i < v->chunkCount; ++i )
	{
		b3VoxelChunk* chunk = v->chunks + i;
		if ( chunk->geometry )
			b3Free( chunk->geometry, B3_VOXEL_CHUNK_CELLS * sizeof( uint16_t ) );
		if ( chunk->occupied )
			b3Free( chunk->occupied, (size_t)chunk->occupiedCapacity * sizeof( b3Vec3i ) );
	}
	if ( v->chunks )
		b3Free( v->chunks, (size_t)v->chunkCapacity * sizeof( b3VoxelChunk ) );
	if ( v->slots )
		b3Free( v->slots, (size_t)v->slotCap * sizeof( b3VoxelChunkSlot ) );
	for ( int i = 0; i < v->geometryCount; ++i )
		b3Free( v->geometries[i].boxes, (size_t)v->geometries[i].count * sizeof( b3VoxelSubBox ) );
	if ( v->geometries )
		b3Free( v->geometries, (size_t)v->geometryCapacity * sizeof( b3VoxelGeometry ) );
	b3Free( v, sizeof( b3VoxelData ) );
}

int b3VoxelData_GetCellCount( const b3VoxelData* v )
{
	return v ? v->cellCount : 0;
}

float b3VoxelData_GetVoxelSize( const b3VoxelData* v )
{
	return v ? v->voxelSize : 0.0f;
}

b3AABB b3VoxelData_GetBounds( const b3VoxelData* v )
{
	if ( v == NULL || v->cellCount == 0 )
		return (b3AABB){ b3Vec3_zero, b3Vec3_zero };
	return (b3AABB){ b3Sub( v->localBounds.lowerBound, v->geometryPadding ),
		b3Add( v->localBounds.upperBound, v->geometryPadding ) };
}

bool b3VoxelData_IsSolid( const b3VoxelData* v, b3Vec3i cell )
{
	if ( v == NULL )
		return false;
	int ci = b3Voxel_findChunk( v, b3Voxel_packChunk( b3Voxel_chunkOf( cell ) ) );
	if ( ci < 0 )
		return false;
	int li = b3Voxel_localIndex( cell.x & B3_VOXEL_CHUNK_MASK, cell.y & B3_VOXEL_CHUNK_MASK, cell.z & B3_VOXEL_CHUNK_MASK );
	return v->chunks[ci].solid[li] != 0;
}

int b3VoxelData_GetCells( const b3VoxelData* v, b3Vec3i* cells, int capacity )
{
	if ( v == NULL || cells == NULL || capacity <= 0 )
		return 0;
	return b3Voxel_GetCells( v, cells, capacity );
}

float b3Voxel_GetVoxelSize( const b3VoxelData* v )
{
	return v->voxelSize;
}

int b3Voxel_GetCellCount( const b3VoxelData* v )
{
	return v->cellCount;
}

uint32_t b3Voxel_GetHash( const b3VoxelData* v )
{
	return v->hash;
}

bool b3Voxel_GetLocalBounds( const b3VoxelData* v, b3AABB* out )
{
	if ( v->cellCount == 0 )
		return false;
	*out = b3VoxelData_GetBounds( v );
	return true;
}

static inline bool b3Voxel_aabbIntersects( const b3AABB* a, const b3AABB* b )
{
	if ( a->upperBound.x < b->lowerBound.x || a->lowerBound.x > b->upperBound.x )
		return false;
	if ( a->upperBound.y < b->lowerBound.y || a->lowerBound.y > b->upperBound.y )
		return false;
	if ( a->upperBound.z < b->lowerBound.z || a->lowerBound.z > b->upperBound.z )
		return false;
	return true;
}

static int b3Voxel_QueryCellsInternal( const b3VoxelData* v, b3AABB queryLocal, b3Vec3i* out, int cap,
	b3VoxelCellQueryFcn* visitor, void* context )
{
	if ( v->cellCount == 0 || cap <= 0 )
		return 0;

	b3AABB clip;
	b3AABB bounds = b3VoxelData_GetBounds( v );
	clip.lowerBound = b3Max( queryLocal.lowerBound, bounds.lowerBound );
	clip.upperBound = b3Min( queryLocal.upperBound, bounds.upperBound );
	if ( clip.lowerBound.x > clip.upperBound.x || clip.lowerBound.y > clip.upperBound.y ||
		 clip.lowerBound.z > clip.upperBound.z )
		return 0;

	const float eps = 1e-6f;
	float s = v->voxelSize, inv = v->invVoxelSize;
	int gMinX = (int)ceilf( ( clip.lowerBound.x - v->geometryPadding.x ) * inv - 0.5f - eps );
	int gMinY = (int)ceilf( ( clip.lowerBound.y - v->geometryPadding.y ) * inv - 0.5f - eps );
	int gMinZ = (int)ceilf( ( clip.lowerBound.z - v->geometryPadding.z ) * inv - 0.5f - eps );
	int gMaxX = (int)floorf( ( clip.upperBound.x + v->geometryPadding.x ) * inv + 0.5f + eps );
	int gMaxY = (int)floorf( ( clip.upperBound.y + v->geometryPadding.y ) * inv + 0.5f + eps );
	int gMaxZ = (int)floorf( ( clip.upperBound.z + v->geometryPadding.z ) * inv + 0.5f + eps );
	if ( gMinX > gMaxX || gMinY > gMaxY || gMinZ > gMaxZ )
		return 0;

	int cMinX = gMinX >> B3_VOXEL_CHUNK_BITS, cMaxX = gMaxX >> B3_VOXEL_CHUNK_BITS;
	int cMinY = gMinY >> B3_VOXEL_CHUNK_BITS, cMaxY = gMaxY >> B3_VOXEL_CHUNK_BITS;
	int cMinZ = gMinZ >> B3_VOXEL_CHUNK_BITS, cMaxZ = gMaxZ >> B3_VOXEL_CHUNK_BITS;

	float half = 0.5f * s;
	int n = 0;
	for ( int cz = cMinZ; cz <= cMaxZ; ++cz )
	{
		for ( int cy = cMinY; cy <= cMaxY; ++cy )
		{
			for ( int cx = cMinX; cx <= cMaxX; ++cx )
			{
				int ci = b3Voxel_findChunk( v, b3Voxel_packChunk( (b3Vec3i){ cx, cy, cz } ) );
				if ( ci < 0 )
					continue;
				const b3VoxelChunk* chunk = v->chunks + ci;
				b3AABB chunkBounds = { b3Sub( chunk->solidBounds.lowerBound, v->geometryPadding ),
					b3Add( chunk->solidBounds.upperBound, v->geometryPadding ) };
				if ( !b3Voxel_aabbIntersects( &chunkBounds, &clip ) )
					continue;
				int base[3] = { cx << B3_VOXEL_CHUNK_BITS, cy << B3_VOXEL_CHUNK_BITS, cz << B3_VOXEL_CHUNK_BITS };
				for ( int k = 0; k < chunk->occupiedCount; ++k )
				{
					b3Vec3i lc = chunk->occupied[k];
					b3Vec3i g = { base[0] + lc.x, base[1] + lc.y, base[2] + lc.z };
					b3AABB cellBox = { { g.x * s - half, g.y * s - half, g.z * s - half },
									   { g.x * s + half, g.y * s + half, g.z * s + half } };
					cellBox.lowerBound = b3Sub( cellBox.lowerBound, v->geometryPadding );
					cellBox.upperBound = b3Add( cellBox.upperBound, v->geometryPadding );
					if ( !b3Voxel_aabbIntersects( &cellBox, &clip ) )
						continue;
					if ( visitor )
					{
						if ( !visitor( g, context ) ) return n;
						continue;
					}
					if ( n < cap )
						out[n] = g;
					n++;
					if ( n >= cap )
						return cap;
				}
			}
		}
	}
	return n;
}

int b3Voxel_QueryCells( const b3VoxelData* v, b3AABB queryLocal, b3Vec3i* out, int cap )
{
	return b3Voxel_QueryCellsInternal( v, queryLocal, out, cap, NULL, NULL );
}

void b3Voxel_VisitCells( const b3VoxelData* v, b3AABB queryLocal, b3VoxelCellQueryFcn* visitor, void* context )
{
	b3Voxel_QueryCellsInternal( v, queryLocal, NULL, INT32_MAX, visitor, context );
}

float b3Voxel_GetMinExtent( const b3VoxelData* v )
{
	float extent = 0.5f * v->voxelSize;
	for ( int i = 0; i < v->geometryCount; ++i )
	{
		const b3VoxelGeometry* geometry = v->geometries + i;
		for ( int j = 0; j < geometry->count; ++j )
		{
			b3Vec3 h = geometry->boxes[j].halfExtents;
			extent = b3MinFloat( extent, b3MinFloat( h.x, b3MinFloat( h.y, h.z ) ) );
		}
	}
	return extent;
}

static b3Vec3i b3Voxel_unpackChunk( uint64_t key )
{
	const int64_t OFF = 1 << 20;
	const uint64_t M = ( 1ull << 21 ) - 1;
	return (b3Vec3i){ (int)( (int64_t)( ( key >> 42 ) & M ) - OFF ), (int)( (int64_t)( ( key >> 21 ) & M ) - OFF ),
					  (int)( (int64_t)( key & M ) - OFF ) };
}

int b3Voxel_GetCells( const b3VoxelData* v, b3Vec3i* out, int cap )
{
	int n = 0;
	for ( int s = 0; s < v->slotCap; ++s )
	{
		if ( v->slots[s].key == 0 )
			continue;
		b3Vec3i cp = b3Voxel_unpackChunk( v->slots[s].key );
		const b3VoxelChunk* chunk = v->chunks + v->slots[s].index;
		int base[3] = { cp.x << B3_VOXEL_CHUNK_BITS, cp.y << B3_VOXEL_CHUNK_BITS, cp.z << B3_VOXEL_CHUNK_BITS };
		for ( int k = 0; k < chunk->occupiedCount; ++k )
		{
			b3Vec3i lc = chunk->occupied[k];
			if ( n < cap )
				out[n] = (b3Vec3i){ base[0] + lc.x, base[1] + lc.y, base[2] + lc.z };
			n++;
			if ( n >= cap )
				return cap;
		}
	}
	return n;
}

b3MassData b3Voxel_ComputeMass( const b3VoxelData* v, float density )
{
	b3MassData md = { 0.0f, b3Vec3_zero, b3Mat3_zero };
	if ( v == NULL || v->cellCount == 0 || density <= 0.0f )
		return md;

	float s = v->voxelSize;
	float cellMass = density * s * s * s;
	float cubeInertia = ( 1.0f / 6.0f ) * cellMass * s * s; // solid cube about its centre, per axis

	double M = 0.0;
	b3Vec3 com = b3Vec3_zero;
	for ( int slot = 0; slot < v->slotCap; ++slot )
	{
		if ( v->slots[slot].key == 0 )
			continue;
		b3Vec3i cp = b3Voxel_unpackChunk( v->slots[slot].key );
		const b3VoxelChunk* chunk = v->chunks + v->slots[slot].index;
		int base[3] = { cp.x << B3_VOXEL_CHUNK_BITS, cp.y << B3_VOXEL_CHUNK_BITS, cp.z << B3_VOXEL_CHUNK_BITS };
		for ( int k = 0; k < chunk->occupiedCount; ++k )
		{
			b3Vec3i lc = chunk->occupied[k];
			b3Vec3 c = { ( base[0] + lc.x ) * s, ( base[1] + lc.y ) * s, ( base[2] + lc.z ) * s };
			M += cellMass;
			com = b3MulAdd( com, cellMass, c );
		}
	}
	com = b3MulSV( 1.0f / (float)M, com );

	b3Matrix3 I = b3Mat3_zero;
	for ( int slot = 0; slot < v->slotCap; ++slot )
	{
		if ( v->slots[slot].key == 0 )
			continue;
		b3Vec3i cp = b3Voxel_unpackChunk( v->slots[slot].key );
		const b3VoxelChunk* chunk = v->chunks + v->slots[slot].index;
		int base[3] = { cp.x << B3_VOXEL_CHUNK_BITS, cp.y << B3_VOXEL_CHUNK_BITS, cp.z << B3_VOXEL_CHUNK_BITS };
		for ( int k = 0; k < chunk->occupiedCount; ++k )
		{
			b3Vec3i lc = chunk->occupied[k];
			b3Vec3 c = { ( base[0] + lc.x ) * s, ( base[1] + lc.y ) * s, ( base[2] + lc.z ) * s };
			b3Vec3 r = b3Sub( c, com );
			float rr = b3Dot( r, r );
			I.cx.x += cubeInertia + cellMass * ( rr - r.x * r.x );
			I.cy.y += cubeInertia + cellMass * ( rr - r.y * r.y );
			I.cz.z += cubeInertia + cellMass * ( rr - r.z * r.z );
			I.cx.y += -cellMass * r.x * r.y;
			I.cx.z += -cellMass * r.x * r.z;
			I.cy.z += -cellMass * r.y * r.z;
		}
	}
	I.cy.x = I.cx.y;
	I.cz.x = I.cx.z;
	I.cz.y = I.cy.z;

	md.mass = (float)M;
	md.center = com;
	md.inertia = I;
	return md;
}

b3VoxelData* b3CreateVoxelDataEx( const b3Vec3i* cells, const uint16_t* geomIndices, int count, float voxelSize )
{
	b3VoxelData* v = b3CreateVoxelData( cells, count, voxelSize );
	if ( v && geomIndices ) b3Voxel_AddCellsEx( v, cells, geomIndices, count );
	return v;
}

int b3VoxelData_AddGeometry( b3VoxelData* voxels, const b3VoxelSubBox* boxes, int count )
{
	if ( voxels == NULL || boxes == NULL || count <= 0 || voxels->geometryCount >= UINT16_MAX ) return 0;
	for ( int i = 0; i < count; ++i )
	{
		if ( !b3IsValidVec3( boxes[i].center ) || !b3IsValidVec3( boxes[i].halfExtents ) ||
			 boxes[i].halfExtents.x <= 0 || boxes[i].halfExtents.y <= 0 || boxes[i].halfExtents.z <= 0 ) return 0;
	}
	if ( voxels->geometryCount == voxels->geometryCapacity )
	{
		int capacity = voxels->geometryCapacity ? 2 * voxels->geometryCapacity : 8;
		b3VoxelGeometry* grown = (b3VoxelGeometry*)b3Alloc( (size_t)capacity * sizeof( b3VoxelGeometry ) );
		if ( voxels->geometries )
		{
			memcpy( grown, voxels->geometries, (size_t)voxels->geometryCount * sizeof( b3VoxelGeometry ) );
			b3Free( voxels->geometries, (size_t)voxels->geometryCapacity * sizeof( b3VoxelGeometry ) );
		}
		voxels->geometries = grown;
		voxels->geometryCapacity = capacity;
	}
	b3VoxelGeometry* geometry = voxels->geometries + voxels->geometryCount++;
	geometry->boxes = (b3VoxelSubBox*)b3Alloc( (size_t)count * sizeof( b3VoxelSubBox ) );
	memcpy( geometry->boxes, boxes, (size_t)count * sizeof( b3VoxelSubBox ) );
	geometry->count = count;
	float half = 0.5f * voxels->voxelSize;
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 extent = b3Add( b3Abs( boxes[i].center ), boxes[i].halfExtents );
		voxels->geometryPadding = b3Max( voxels->geometryPadding, b3Sub( extent, (b3Vec3){ half, half, half } ) );
	}
	return voxels->geometryCount;
}

uint16_t b3VoxelData_GetCellGeometry( const b3VoxelData* voxels, b3Vec3i cell )
{
	if ( voxels == NULL ) return 0;
	int ci = b3Voxel_findChunk( voxels, b3Voxel_packChunk( b3Voxel_chunkOf( cell ) ) );
	if ( ci < 0 ) return 0;
	const b3VoxelChunk* chunk = voxels->chunks + ci;
	int li = b3Voxel_localIndex( cell.x & 15, cell.y & 15, cell.z & 15 );
	return chunk->solid[li] && chunk->geometry ? chunk->geometry[li] : 0;
}

bool b3Voxel_HasGeometry( const b3VoxelData* v )
{
	return v && v->geometryCount > 0;
}

b3Vec3 b3Voxel_GetGeometryPadding( const b3VoxelData* v )
{
	return v->geometryPadding;
}

int b3Voxel_GetCellBoxes( const b3VoxelData* v, b3Vec3i cell, b3VoxelSubBox* fallback, const b3VoxelSubBox** boxes )
{
	uint16_t index = v->geometryCount > 0 ? b3VoxelData_GetCellGeometry( v, cell ) : 0;
	if ( index > 0 && index <= v->geometryCount )
	{
		const b3VoxelGeometry* geometry = v->geometries + index - 1;
		*boxes = geometry->boxes;
		return geometry->count;
	}
	float h = 0.5f * v->voxelSize;
	*fallback = (b3VoxelSubBox){ b3Vec3_zero, { h, h, h } };
	*boxes = fallback;
	return 1;
}

bool b3Voxel_IsInternalFace( const b3VoxelData* voxel, b3Vec3i cell, b3VoxelSubBox box, b3AABB patch, b3Vec3 normal )
{
	float n[3] = { normal.x, normal.y, normal.z };
	float lo[3] = { patch.lowerBound.x, patch.lowerBound.y, patch.lowerBound.z };
	float hi[3] = { patch.upperBound.x, patch.upperBound.y, patch.upperBound.z };
	float c[3] = { box.center.x, box.center.y, box.center.z };
	float h[3] = { box.halfExtents.x, box.halfExtents.y, box.halfExtents.z };
	int coord[3] = { cell.x, cell.y, cell.z };
	int axis = b3AbsFloat( n[0] ) >= b3AbsFloat( n[1] ) ? 0 : 1;
	axis = b3AbsFloat( n[axis] ) >= b3AbsFloat( n[2] ) ? axis : 2;
	float sign = n[axis] >= 0.0f ? 1.0f : -1.0f;
	float size = voxel->voxelSize;
	float face = coord[axis] * size + c[axis] + sign * h[axis];
	float boundary = ( coord[axis] + 0.5f * sign ) * size;
	float tolerance = B3_LINEAR_SLOP;
	if ( b3AbsFloat( face - boundary ) > tolerance || lo[axis] > face + tolerance || hi[axis] < face - tolerance )
	{
		return false;
	}
	coord[axis] += sign > 0.0f ? 1 : -1;
	b3Vec3i neighbor = { coord[0], coord[1], coord[2] };
	if ( !b3VoxelData_IsSolid( voxel, neighbor ) )
	{
		return false;
	}
	b3VoxelSubBox fallback;
	const b3VoxelSubBox* boxes;
	int count = b3Voxel_GetCellBoxes( voxel, neighbor, &fallback, &boxes );
	for ( int i = 0; i < count; ++i )
	{
		float center[3] = { boxes[i].center.x, boxes[i].center.y, boxes[i].center.z };
		float half[3] = { boxes[i].halfExtents.x, boxes[i].halfExtents.y, boxes[i].halfExtents.z };
		bool covered = true;
		for ( int j = 0; j < 3; ++j )
		{
			float lower = coord[j] * size + center[j] - half[j];
			float upper = coord[j] * size + center[j] + half[j];
			if ( j == axis )
			{
				covered &= sign > 0.0f ? lower <= face + tolerance && upper > face + tolerance :
					upper >= face - tolerance && lower < face - tolerance;
			}
			else
			{
				covered &= lower - tolerance <= lo[j] && hi[j] <= upper + tolerance;
			}
		}
		if ( covered )
		{
			return true;
		}
	}
	return false;
}

bool b3Voxel_AddCellsEx( b3VoxelData* v, const b3Vec3i* cells, const uint16_t* geomIndices, int count )
{
	bool changed = b3Voxel_AddCells( v, cells, count );
	if ( v == NULL || cells == NULL || count <= 0 ) return changed;
	bool geometryChanged = false;
	for ( int i = 0; i < count; ++i )
	{
		int ci = b3Voxel_findChunk( v, b3Voxel_packChunk( b3Voxel_chunkOf( cells[i] ) ) );
		if ( ci < 0 ) continue;
		b3VoxelChunk* chunk = v->chunks + ci;
		int li = b3Voxel_localIndex( cells[i].x & 15, cells[i].y & 15, cells[i].z & 15 );
		uint16_t index = geomIndices ? geomIndices[i] : 0;
		if ( index != 0 && chunk->geometry == NULL )
			chunk->geometry = (uint16_t*)b3AllocZeroed( B3_VOXEL_CHUNK_CELLS * sizeof( uint16_t ) );
		if ( chunk->geometry && chunk->geometry[li] != index )
		{
			chunk->geometry[li] = index;
			changed = true;
			geometryChanged = true;
		}
	}
	if ( geometryChanged )
	{
		v->hash = v->hash * 16777619u + 1u;
		if ( v->hash == 0 ) v->hash = 1;
	}
	return changed;
}

bool b3Voxel_RemoveCells( b3VoxelData* v, const b3Vec3i* cells, int count )
{
	if ( v == NULL || cells == NULL || count <= 0 || v->cellCount == 0 )
		return false;
	bool* dirty = (bool*)b3AllocZeroed( (size_t)v->chunkCount * sizeof( bool ) );
	bool changed = false;
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3i cell = cells[i];
		int ci = b3Voxel_findChunk( v, b3Voxel_packChunk( b3Voxel_chunkOf( cell ) ) );
		if ( ci < 0 )
			continue;
		b3VoxelChunk* chunk = v->chunks + ci;
		b3Vec3i local = { cell.x & 15, cell.y & 15, cell.z & 15 };
		int li = b3Voxel_localIndex( local.x, local.y, local.z );
		if ( !chunk->solid[li] )
			continue;
		chunk->solid[li] = 0;
		if ( chunk->geometry ) chunk->geometry[li] = 0;
		for ( int k = 0; k < chunk->occupiedCount; ++k )
		{
			b3Vec3i c = chunk->occupied[k];
			if ( c.x == local.x && c.y == local.y && c.z == local.z )
			{
				chunk->occupied[k] = chunk->occupied[--chunk->occupiedCount];
				break;
			}
		}
		--v->cellCount;
		dirty[ci] = true;
		changed = true;
	}
	if ( changed )
	{
		v->localBounds = (b3AABB){ { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };
		float half = 0.5f * v->voxelSize;
		for ( int s = 0; s < v->slotCap; ++s )
		{
			if ( v->slots[s].key == 0 )
				continue;
			int ci = v->slots[s].index;
			b3VoxelChunk* chunk = v->chunks + ci;
			if ( dirty[ci] )
			{
				chunk->solidBounds = (b3AABB){ { FLT_MAX, FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX, -FLT_MAX } };
				b3Vec3i cp = b3Voxel_unpackChunk( v->slots[s].key );
				for ( int k = 0; k < chunk->occupiedCount; ++k )
				{
					b3Vec3i lc = chunk->occupied[k];
					b3Vec3 center = { ( cp.x * 16 + lc.x ) * v->voxelSize,
						( cp.y * 16 + lc.y ) * v->voxelSize, ( cp.z * 16 + lc.z ) * v->voxelSize };
					b3Vec3 extents = { half, half, half };
					chunk->solidBounds.lowerBound = b3Min( chunk->solidBounds.lowerBound, b3Sub( center, extents ) );
					chunk->solidBounds.upperBound = b3Max( chunk->solidBounds.upperBound, b3Add( center, extents ) );
				}
			}
			if ( chunk->occupiedCount > 0 )
			{
				v->localBounds.lowerBound = b3Min( v->localBounds.lowerBound, chunk->solidBounds.lowerBound );
				v->localBounds.upperBound = b3Max( v->localBounds.upperBound, chunk->solidBounds.upperBound );
			}
		}
		v->hasBounds = v->cellCount > 0;
		if ( !v->hasBounds )
			v->localBounds = (b3AABB){ b3Vec3_zero, b3Vec3_zero };
		v->hash = v->hash * 16777619u + 1u;
		if ( v->hash == 0 ) v->hash = 1;
	}
	b3Free( dirty, (size_t)v->chunkCount * sizeof( bool ) );
	return changed;
}

bool b3Voxel_AddCells( b3VoxelData* v, const b3Vec3i* cells, int count )
{
	if ( v == NULL || cells == NULL || count <= 0 )
		return false;
	bool changed = false;
	float half = 0.5f * v->voxelSize;
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3i cell = cells[i];
		int ci = b3Voxel_getOrCreateChunk( v, b3Voxel_chunkOf( cell ) );
		b3VoxelChunk* chunk = v->chunks + ci;
		int lx = cell.x & 15, ly = cell.y & 15, lz = cell.z & 15;
		int li = b3Voxel_localIndex( lx, ly, lz );
		if ( chunk->solid[li] )
			continue;
		if ( chunk->occupiedCount == chunk->occupiedCapacity )
		{
			int nc = chunk->occupiedCapacity ? 2 * chunk->occupiedCapacity : 32;
			b3Vec3i* grown = (b3Vec3i*)b3Alloc( (size_t)nc * sizeof( b3Vec3i ) );
			if ( chunk->occupied )
			{
				memcpy( grown, chunk->occupied, (size_t)chunk->occupiedCount * sizeof( b3Vec3i ) );
				b3Free( chunk->occupied, (size_t)chunk->occupiedCapacity * sizeof( b3Vec3i ) );
			}
			chunk->occupied = grown;
			chunk->occupiedCapacity = nc;
		}
		chunk->solid[li] = 1;
		chunk->occupied[chunk->occupiedCount++] = (b3Vec3i){ lx, ly, lz };
		b3Vec3 center = { cell.x * v->voxelSize, cell.y * v->voxelSize, cell.z * v->voxelSize };
		b3Vec3 extents = { half, half, half };
		b3Vec3 lo = b3Sub( center, extents ), hi = b3Add( center, extents );
		// Empty chunks retain their hash slot so additions can reuse their storage.
		if ( chunk->occupiedCount == 1 ) chunk->solidBounds = (b3AABB){ lo, hi };
		else
		{
			chunk->solidBounds.lowerBound = b3Min( chunk->solidBounds.lowerBound, lo );
			chunk->solidBounds.upperBound = b3Max( chunk->solidBounds.upperBound, hi );
		}
		if ( v->cellCount == 0 ) v->localBounds = (b3AABB){ lo, hi };
		else
		{
			v->localBounds.lowerBound = b3Min( v->localBounds.lowerBound, lo );
			v->localBounds.upperBound = b3Max( v->localBounds.upperBound, hi );
		}
		++v->cellCount;
		v->hasBounds = true;
		changed = true;
	}
	if ( changed )
	{
		v->hash = v->hash * 16777619u + 1u;
		if ( v->hash == 0 ) v->hash = 1;
	}
	return changed;
}

void b3Voxel_ApplyAerodynamics( const b3VoxelData* v, b3Transform transform, b3Vec3 localCenterOfMass,
								b3Vec3 linearVelocity, b3Vec3 angularVelocity, b3Vec3 wind, float drag,
								float lift, float maxSpeed, float airDensity, b3Vec3* outForce, b3Vec3* outTorque )
{
	if ( v == NULL || v->cellCount == 0 )
	{
		return;
	}

	static const b3Vec3i kFaceOffsets[6] = {
		{ 1, 0, 0 },  // +X
		{ -1, 0, 0 }, // -X
		{ 0, 1, 0 },  // +Y
		{ 0, -1, 0 }, // -Y
		{ 0, 0, 1 },  // +Z
		{ 0, 0, -1 }  // -Z
	};

	static const b3Vec3 kFaceNormals[6] = {
		{ 1.0f, 0.0f, 0.0f },
		{ -1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 0.0f, -1.0f }
	};

	b3Matrix3 matrix = b3MakeMatrixFromQuat( transform.q );
	float s = v->voxelSize;
	float faceArea = s * s;
	b3Vec3 totalForce = { 0 };
	b3Vec3 totalTorque = { 0 };

	for ( int slot = 0; slot < v->slotCap; ++slot )
	{
		if ( v->slots[slot].key == 0 )
		{
			continue;
		}

		b3Vec3i cp = b3Voxel_unpackChunk( v->slots[slot].key );
		const b3VoxelChunk* chunk = v->chunks + v->slots[slot].index;
		int base[3] = { cp.x << B3_VOXEL_CHUNK_BITS, cp.y << B3_VOXEL_CHUNK_BITS, cp.z << B3_VOXEL_CHUNK_BITS };

		for ( int k = 0; k < chunk->occupiedCount; ++k )
		{
			b3Vec3i lc = chunk->occupied[k];
			b3Vec3i g = { base[0] + lc.x, base[1] + lc.y, base[2] + lc.z };

			for ( int f = 0; f < 6; ++f )
			{
				int nlx = lc.x + kFaceOffsets[f].x;
				int nly = lc.y + kFaceOffsets[f].y;
				int nlz = lc.z + kFaceOffsets[f].z;
				bool isNeighborSolid;

				if ( (unsigned)nlx < B3_VOXEL_CHUNK_SIZE && (unsigned)nly < B3_VOXEL_CHUNK_SIZE &&
					 (unsigned)nlz < B3_VOXEL_CHUNK_SIZE )
				{
					isNeighborSolid = ( chunk->solid[( nlx << 8 ) | ( nly << 4 ) | nlz] != 0 );
				}
				else
				{
					b3Vec3i neighborCell = { g.x + kFaceOffsets[f].x, g.y + kFaceOffsets[f].y,
											 g.z + kFaceOffsets[f].z };
					isNeighborSolid = b3VoxelData_IsSolid( v, neighborCell );
				}

				if ( isNeighborSolid )
				{
					continue;
				}

				// Face center in local coordinates
				b3Vec3 fn = kFaceNormals[f];
				b3Vec3 localFaceCenter = {
					( (float)g.x + 0.5f * fn.x ) * s,
					( (float)g.y + 0.5f * fn.y ) * s,
					( (float)g.z + 0.5f * fn.z ) * s,
				};

				b3Vec3 normal = b3MulMV( matrix, fn );
				b3Vec3 lever = b3MulMV( matrix, b3Sub( localFaceCenter, localCenterOfMass ) );
				b3Vec3 centerVelocity = b3Add( linearVelocity, b3Cross( angularVelocity, lever ) );
				b3Vec3 relativeVelocity = b3Sub( wind, centerVelocity );

				float speed;
				b3Vec3 direction = b3GetLengthAndNormalize( &speed, relativeVelocity );
				float cosTheta = -b3Dot( normal, direction );

				if ( cosTheta > FLT_EPSILON )
				{
					b3Vec3 liftDir = b3Sub( b3MulSV( -1.0f, normal ), b3MulSV( cosTheta, direction ) );
					float liftDirLen = b3Length( liftDir );
					if ( liftDirLen > FLT_EPSILON )
					{
						liftDir = b3MulSV( 1.0f / liftDirLen, liftDir );
					}

					speed = b3MinFloat( speed, maxSpeed );

					float qA = 0.5f * airDensity * faceArea * speed * speed;
					float sinTheta = liftDirLen;
					float cl = 2.0f * cosTheta * sinTheta;
					float cd = 1.28f * cosTheta * cosTheta + 0.02f;

					b3Vec3 dragForce = b3MulSV( qA * drag * cd, direction );
					b3Vec3 liftForce = b3MulSV( qA * lift * cl, liftDir );
					b3Vec3 deltaForce = b3Add( dragForce, liftForce );
					b3Vec3 deltaTorque = b3Cross( lever, deltaForce );

					totalForce = b3Add( totalForce, deltaForce );
					totalTorque = b3Add( totalTorque, deltaTorque );
				}
			}
		}
	}

	*outForce = b3Add( *outForce, totalForce );
	*outTorque = b3Add( *outTorque, totalTorque );
}
