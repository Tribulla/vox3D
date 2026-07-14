#include "voxel_shape.h"

#include "core.h"

#include <math.h>
#include <string.h>

#define B3_VOXEL_CHUNK_BITS 4
#define B3_VOXEL_CHUNK_SIZE 16 // 1 << B3_VOXEL_CHUNK_BITS
#define B3_VOXEL_CHUNK_MASK 15 // B3_VOXEL_CHUNK_SIZE - 1
#define B3_VOXEL_CHUNK_CELLS 4096 // 16^3

typedef struct b3VoxelChunk
{
	uint8_t solid[B3_VOXEL_CHUNK_CELLS]; // 0 = air, 1 = solid; index (lx<<8)|(ly<<4)|lz
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
		if ( chunk->occupied )
			b3Free( chunk->occupied, (size_t)chunk->occupiedCapacity * sizeof( b3Vec3i ) );
	}
	if ( v->chunks )
		b3Free( v->chunks, (size_t)v->chunkCapacity * sizeof( b3VoxelChunk ) );
	if ( v->slots )
		b3Free( v->slots, (size_t)v->slotCap * sizeof( b3VoxelChunkSlot ) );
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
	return v->localBounds;
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
	*out = v->localBounds;
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

int b3Voxel_QueryCells( const b3VoxelData* v, b3AABB queryLocal, b3Vec3i* out, int cap )
{
	if ( v->cellCount == 0 || cap <= 0 )
		return 0;

	b3AABB clip;
	clip.lowerBound.x = b3MaxFloat( queryLocal.lowerBound.x, v->localBounds.lowerBound.x );
	clip.lowerBound.y = b3MaxFloat( queryLocal.lowerBound.y, v->localBounds.lowerBound.y );
	clip.lowerBound.z = b3MaxFloat( queryLocal.lowerBound.z, v->localBounds.lowerBound.z );
	clip.upperBound.x = b3MinFloat( queryLocal.upperBound.x, v->localBounds.upperBound.x );
	clip.upperBound.y = b3MinFloat( queryLocal.upperBound.y, v->localBounds.upperBound.y );
	clip.upperBound.z = b3MinFloat( queryLocal.upperBound.z, v->localBounds.upperBound.z );
	if ( clip.lowerBound.x > clip.upperBound.x || clip.lowerBound.y > clip.upperBound.y ||
		 clip.lowerBound.z > clip.upperBound.z )
		return 0;

	const float eps = 1e-6f;
	float s = v->voxelSize, inv = v->invVoxelSize;
	int gMinX = (int)ceilf( clip.lowerBound.x * inv - 0.5f - eps );
	int gMinY = (int)ceilf( clip.lowerBound.y * inv - 0.5f - eps );
	int gMinZ = (int)ceilf( clip.lowerBound.z * inv - 0.5f - eps );
	int gMaxX = (int)floorf( clip.upperBound.x * inv + 0.5f + eps );
	int gMaxY = (int)floorf( clip.upperBound.y * inv + 0.5f + eps );
	int gMaxZ = (int)floorf( clip.upperBound.z * inv + 0.5f + eps );
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
				if ( !b3Voxel_aabbIntersects( &chunk->solidBounds, &clip ) )
					continue;
				int base[3] = { cx << B3_VOXEL_CHUNK_BITS, cy << B3_VOXEL_CHUNK_BITS, cz << B3_VOXEL_CHUNK_BITS };
				for ( int k = 0; k < chunk->occupiedCount; ++k )
				{
					b3Vec3i lc = chunk->occupied[k];
					b3Vec3i g = { base[0] + lc.x, base[1] + lc.y, base[2] + lc.z };
					b3AABB cellBox = { { g.x * s - half, g.y * s - half, g.z * s - half },
									   { g.x * s + half, g.y * s + half, g.z * s + half } };
					if ( !b3Voxel_aabbIntersects( &cellBox, &clip ) )
						continue;
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
