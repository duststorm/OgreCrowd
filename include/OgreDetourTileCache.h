#ifndef OGREDETOURTILECACHE_H
#define OGREDETOURTILECACHE_H

#include "OgreRecast.h"
#include "DetourTileCache/DetourTileCacheBuilder.h"
#include "DetourTileCache/DetourTileCache.h"
#include "Detour/DetourCommon.h"
#include "RecastContrib/fastlz/fastlz.h"
#include "RecastInputGeom.h"

const float TEMP_OBSTACLE_RADIUS = 1.0f;
const float TEMP_OBSTACLE_HEIGHT = 2.0f;

struct MeshProcess : public dtTileCacheMeshProcess
{
    InputGeom* m_geom;

    inline MeshProcess() : m_geom(0) {
    }

    inline void init(InputGeom* geom) {
        m_geom = geom;
    }

    virtual void process(struct dtNavMeshCreateParams* params,
                         unsigned char* polyAreas, unsigned short* polyFlags)
    {
        // Update poly flags from areas.
        for (int i = 0; i < params->polyCount; ++i)
        {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
                polyAreas[i] = SAMPLE_POLYAREA_GROUND;

            if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
                polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
                polyAreas[i] == SAMPLE_POLYAREA_ROAD)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }

        // Pass in off-mesh connections.
        if (m_geom)
        {
            // TODO implement off-mesh connections
            /*
            params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
            params->offMeshConRad = m_geom->getOffMeshConnectionRads();
            params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
            params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
            params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
            params->offMeshConUserID = m_geom->getOffMeshConnectionId();
            params->offMeshConCount = m_geom->getOffMeshConnectionCount();
            */
        }

    }
};



struct FastLZCompressor : public dtTileCacheCompressor
{
        virtual int maxCompressedSize(const int bufferSize)
        {
                return (int)(bufferSize* 1.05f);
        }

        virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
                                                          unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
        {
                *compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
                return DT_SUCCESS;
        }

        virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                                                                unsigned char* buffer, const int maxBufferSize, int* bufferSize)
        {
                *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
                return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
        }
};



struct LinearAllocator : public dtTileCacheAlloc
{
    unsigned char* buffer;
    int capacity;
    int top;
    int high;

    LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
    {
        resize(cap);
    }

    ~LinearAllocator()
    {
        dtFree(buffer);
    }

    void resize(const int cap)
    {
        if (buffer) dtFree(buffer);
        buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
        capacity = cap;
    }

    virtual void reset()
    {
        high = dtMax(high, top);
        top = 0;
    }

    virtual void* alloc(const int size)
    {
        if (!buffer)
            return 0;
        if (top+size > capacity)
            return 0;
        unsigned char* mem = &buffer[top];
        top += size;
        return mem;
    }

    virtual void free(void* /*ptr*/)
    {
        // Empty
    }
};


/**
  * Maximum layers (floor levels) that 2D navmeshes can have in the tilecache.
  * This determines the domain size of the tilecache pages, as their dimensions
  * are width*height*layers.
  **/
static const int MAX_LAYERS = 32;

struct TileCacheData
{
    unsigned char* data;
    int dataSize;
};

struct RasterizationContext
{
    RasterizationContext() :
            solid(0),
            triareas(0),
            lset(0),
            chf(0),
            ntiles(0)
    {
        memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
    }

    ~RasterizationContext()
    {
        rcFreeHeightField(solid);
        delete [] triareas;
        rcFreeHeightfieldLayerSet(lset);
        rcFreeCompactHeightfield(chf);
        for (int i = 0; i < MAX_LAYERS; ++i)
        {
            dtFree(tiles[i].data);
            tiles[i].data = 0;
        }
    }

    rcHeightfield* solid;
    unsigned char* triareas;
    rcHeightfieldLayerSet* lset;
    rcCompactHeightfield* chf;
    TileCacheData tiles[MAX_LAYERS];
    int ntiles;
};


struct BuildContext
{
        inline BuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
        inline ~BuildContext() { purge(); }
        void purge()
        {
                dtFreeTileCacheLayer(alloc, layer);
                layer = 0;
                dtFreeTileCacheContourSet(alloc, lcset);
                lcset = 0;
                dtFreeTileCachePolyMesh(alloc, lmesh);
                lmesh = 0;
        }
        struct dtTileCacheLayer* layer;
        struct dtTileCacheContourSet* lcset;
        struct dtTileCachePolyMesh* lmesh;
        struct dtTileCacheAlloc* alloc;
};


static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
        const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
        const int gridSize = gridWidth * gridHeight;
        return headerSize + gridSize*4;
}



/**
  * DetourTileCache manages a large grid of individual navmeshes stored in pages to
  * allow managing a navmesh for a very large map. Navmesh pages can be requested
  * when needed or swapped out when they are no longer needed.
  * Using a tilecache the navigation problem is localized to one tile, but pathfinding
  * can still find a path that references to other neighbour tiles on the higher hierarchy
  * level of the tilecache. Localizing the pathfinding problem allows it to be more scalable,
  * also for very large worlds.
  * DetouTileCache stores navmeshes in an intermediary format as 2D heightfields
  * that can have multiple levels. It allows to quickly generate a 3D navmesh from
  * this intermediary format, with the additional option of adding or removing
  * temporary obstacles to the navmesh and regenerating it.
  **/
class OgreDetourTileCache
{
public:
    OgreDetourTileCache(OgreRecast *recast);
    ~OgreDetourTileCache(void);

    /**
      * This is an Ogre adaptation of Sample_TempObstacles::handleBuild()
      * First init the OgreRecast module like you would construct a simple single
      * navmesh, then invoke this method instead of OgreRecast::NavMeshBuild() to create
      * a tileCache from the specified ogre geometry.
      * The specified ogre entities need to be added to a scenenode in the scene before this
      * method is called.
      * The resulting navmesh will be created in the OgreRecast module, at OgreRecast::m_navMesh;
      **/
    bool TileCacheBuild(std::vector<Ogre::Entity*> srcMeshes);

    /**
      * Build the 2D navigation grid divided in layers that is the intermediary format stored in the tilecache.
      * Builds the specified tile from the given input geometry. Only the part of the geometry that intersects the
      * needed tile is used.
      * From this format a 3D navmesh can be quickly generated at runtime.
      * This process uses a large part of the recast navmesh building pipeline (implemented in OgreRecast::NavMeshBuild()),
      * up till step 4.
      **/
    int rasterizeTileLayers(InputGeom* geom, const int tx, const int ty, const rcConfig& cfg, TileCacheData* tiles, const int maxTiles);

    /**
      * Gets world position of tile with specified index.
      **/
    void getTilePos(const float* pos, int& tx, int& ty);

    void handleUpdate(const float dt);

    void handleMeshChanged(class InputGeom* geom);

    void clearAllTempObstacles(void);

    dtObstacleRef addTempObstacle(Ogre::Vector3 pos);

    dtObstacleRef    removeTempObstacle(const float* sp, const float* sq);

    void drawDetail(const int tx, const int ty);

    void drawNavMesh(void);

    void drawPolyMesh(const struct dtTileCachePolyMesh &mesh, const float *orig, const float cs, const float ch, const struct dtTileCacheLayer &regionLayers, bool colorRegions=true);

    dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq);

    void drawObstacles(const dtTileCache* tc);

    OgreRecast *m_recast;


protected:
    InputGeom* m_geom;

    bool m_keepInterResults;

    struct LinearAllocator *m_talloc;
    struct FastLZCompressor* m_tcomp;

    /**
      * Callback handler that processes right after processing
      * a tile mesh. Adds off-mesh connections to the mesh.
      **/
    struct MeshProcess *m_tmproc;
    class dtTileCache *m_tileCache;

    float m_cacheBuildTimeMs;
    int m_cacheCompressedSize;
    int m_cacheRawSize;
    int m_cacheLayerCount;
    int m_cacheBuildMemUsage;

    int m_maxTiles;
    int m_maxPolysPerTile;
    float m_tileSize;

    float m_cellSize;

    int m_tw;
    int m_th;
};

#endif // OGREDETOURTILECACHE_H
