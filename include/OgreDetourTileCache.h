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


struct TileSelection
{
    Ogre::AxisAlignedBox bounds;
    int minTx;
    int maxTx;
    int minTy;
    int maxTy;
};


/**
  * Struct to store a request to add or remove
  * a convex obstacle to the tilecache as a deferred
  * command (currently unused).
  **/
struct ogredtTileCacheConvexObstacle
{
    ConvexVolume *obstacle;
    dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
    dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
    unsigned short salt;
    unsigned char state;
    unsigned char ntouched;
    unsigned char npending;
    ogredtTileCacheConvexObstacle* next;    // Single linked list
};

/**
  * Implementation of the meshProcess callback that detourTileCache
  * does after building a navmesh. It allows you to do some extra
  * processing on the navmesh, such as connecting off-mesh connections
  * and assigning flags to certain poly areas.
  * The reason it is initialized with an inputGeom object is that it
  * is intended that the inputGeom not only stores the input geometry,
  * but also information that has to be added to the navmesh in this
  * post-processing phase.
  **/
struct MeshProcess : public dtTileCacheMeshProcess
{
    InputGeom* m_geom;

    inline MeshProcess() : m_geom(0) {
    }

    inline void init(InputGeom* geom) {
        m_geom = geom;
    }

    /**
     * Callback that happens after navmesh has been constructed.
     * Allows you to do some additional post-processing on the navmesh,
     * such as adding off-mesh connections or marking poly areas with
     * certain flags.
     **/
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


/**
  * FastLZ implementation of detour tile cache tile compressor.
  * You can define a custom implementation if you wish to use
  * a different compression algorithm for compressing your
  * detour heightfield tiles.
  * The result of compression runs is the data that detourTileCache
  * stores in memory (or can save out to disk).
  * The compressed heightfield tiles are stored in ram as they allow
  * to quickly generate a navmesh tile, possibly with obstacles added
  * to them, without the need for a full rebuild.
  **/
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


/**
  * Allows a custom memory allocation technique to be implemented
  * for storing compressed tiles. This implementation does a linear
  * memory allocation.
  **/
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

/**
  * Struct that stores the actual tile data in binary form.
  **/
struct TileCacheData
{
    unsigned char* data;
    int dataSize;
};

/**
  * Rasterization context stores temporary data used
  * when rasterizing inputGeom into a navmesh.
  **/
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


/**
  * Build context stores temporary data used while
  * building a navmesh tile.
  **/
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


/**
  * Calculate the memory space used by the tilecache.
  **/
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
    /**
      * Create a tilecache that will build a tiled recast navmesh stored at the specified
      * OgreRecast component. Will use specified tilesize (a multiple of 8 between 16 and 128),
      * all other configuration parameters are copied from the OgreRecast component configuration.
      * Tilesize is the number of (recast) cells per tile.
      **/
    OgreDetourTileCache(OgreRecast *recast, int tileSize = 48);
    ~OgreDetourTileCache(void);

    /**
      * Configure the tilecache for building navmesh tiles from the specified input geometry.
      * The inputGeom is mainly used for determining the bounds of the world for which a navmesh
      * will be built, so at least bmin and bmax of inputGeom should be set to your world's outer
      * bounds. This world bounding box is used to calculate the grid size that the tilecache has
      * to initialize.
      * This method has to be called once after construction, and before any tile builds happen.
      **/
    bool configure(InputGeom *inputGeom);

    /**
      * Find tiles that (partially or completely) intersect the specified bounding area.
      * The selectionArea has to be in world units.
      * TileCache needs to be configured before this method can work (needs to know world size
      * of tilecache).
      * TileSelection contains bounding box aligned to the tile bounds and tx ty index ranges
      * for the affected tiles. Note that tile ranges are inclusive! (eg. if minTx=1 and maxTx=1
      * then tile at x=1 has to be rebuilt)
      * It is not necessary for tiles to be already built in order for
      * them to be included in the selection.
      * Make sure you use the included bounding box instead of an arbitrary selection bounding
      * box to bound inputGeom used for rebuilding tiles. Or you might not include all geometry
      * needed to rebuild all affected tiles correctly.
      **/
    TileSelection getTileSelection(const Ogre::AxisAlignedBox &selectionArea);

    /**
      * Returns a bounding box that matches the tile bounds of this cache and that is at least
      * as large as the specified selectionArea bounding box. Height (y) coordinates will be set
      * to the min and max height of this tilecache. (tile selection only happens in x-z plane).
      * Use this function to get correct bounding boxes to cull your inputGeom or scene geometry
      * with for tile rebuilding.
      **/
    Ogre::AxisAlignedBox getTileAlignedBox(const Ogre::AxisAlignedBox &selectionArea);


    /**
      * Build all tiles of the tilecache and construct a recast navmesh from the
      * specified entities. These entities need to be already added to the scene so that
      * their world position and orientation can be calculated.
      *
      * This is an Ogre adaptation of Sample_TempObstacles::handleBuild()
      * First init the OgreRecast module like you would construct a simple single
      * navmesh, then invoke this method instead of OgreRecast::NavMeshBuild() to create
      * a tileCache from the specified ogre geometry.
      * The specified ogre entities need to be added to a scenenode in the scene before this
      * method is called.
      * The resulting navmesh will be created in the OgreRecast module, at OgreRecast::m_navMesh;
      *
      * Will issue a configure() call so the entities specified will determine the world bounds
      * of the tilecache.
      **/
    bool TileCacheBuild(std::vector<Ogre::Entity*> srcMeshes);

    /**
      * Build all navmesh tiles from specified input geom.
      *
      * Will issue a configure() call so the inputGeom specified will determine the world bounds
      * of the tilecache. Therefore you must specify the inputGeom for the entire world.
      *
      * @see OgreDetourTileCache::TileCacheBuild(std::vector<Ogre::Entity*>)
      **/
    bool TileCacheBuild(InputGeom *inputGeom);

// TODO maybe provide isLoaded(tx, ty) method

// TODO create better distinction between loading compressed tiles in cache and building navmesh from them?

// TODO are both updateFromGeometry() and buildTiles() necessary, or can update be dropped? It might be confusing.


    /**
      * Build or rebuild a cache tile at the specified x and y position in the tile grid.
      * Tile is built or rebuilt no matter whether there was already a tile at that position in the grid
      * or not. If there previously was a tile in the specified grid position, it is first removed from the
      * tilecache and replaced with the new one.
      *
      * At the moment this will issue an immediate update of the navmesh at the
      * corresponding tiles. (the alternative is adding a request that is processed as deferred command)
      *
      * Note that you can speed this up by building an inputGeom from only the area that is rebuilt.
      * Don't use an arbitrary bounding box for culling the inputGeom, but use getTileAlignedBox() instead!
      **/
    bool buildTile(const int tx, const int ty, InputGeom *inputGeom);

    /**
      * Build or rebuild a cache tiles or tiles that cover the specified bounding box area.
      *
      * The tiles are built or rebuilt no matter whether there was already a tile at that position in the grid
      * or not. If there previously was a tile in the specified grid position, it is first removed from the
      * tilecache and replaced with the new one.
      *
      * Make sure that the specified inputGeom is either the inputGeom of the complete scene (inefficient) or is
      * built with a tile aligned bounding box (getTileAlignedBox())! The areaToUpdate value can be arbitrary,
      * but will be converted to a tile aligned box.
      *
      * At the moment this will issue an immediate update of the navmesh at the
      * corresponding tiles. (the alternative is adding a request that is processed as deferred command)
      **/
    void buildTiles(InputGeom *inputGeom, const Ogre::AxisAlignedBox *areaToUpdate = NULL);

    /**
      * Build or rebuild tile from list of entities.
      * @see{buildTiles(InputGeom*, const Ogre::AxisAlignedBox*)}
      **/
    void buildTiles(std::vector<Ogre::Entity*> srcEntities, const Ogre::AxisAlignedBox *areaToUpdate = NULL);


// TODO maybe also add a unloadAllTilesExcept(boundingBox) method

    /**
      * Unload all tiles that cover the specified bounding box. The tiles are removed from the
      * cache.
      **/
    void unloadTiles(const Ogre::AxisAlignedBox &areaToUpdate);

    /**
      * Gets world position of tile with specified index.
      **/
    void getTilePos(const float* pos, int& tx, int& ty);

    /**
      * Update (tick) the tilecache.
      * You must call this method in your render loop continuously to dynamically
      * update the navmesh when obstacles are added or removed.
      * Navmesh rebuilding happens per tile and only where needed. Tile rebuilding is
      * timesliced.
      **/
    void handleUpdate(const float dt);

    /**
      * Remove all (cylindrical) temporary obstacles from the tilecache.
      * The navmesh will be rebuilt after the next (one or more) update()
      * call.
      **/
    void clearAllTempObstacles(void);

    /**
      * Add a temporary (cylindrical) obstacle to the tilecache (as a deferred request).
      * The navmesh will be updated correspondingly after the next (one or many)
      * update() call as a deferred command.
      * If m_tileCache->m_params->maxObstacles obstacles are already added, this call
      * will have no effect. Also, at one time only MAX_REQUESTS can be added, or nothing
      * will happen.
      *
      * If successful returns a reference to the added obstacle.
      **/
    dtObstacleRef addTempObstacle(Ogre::Vector3 pos);

    /**
      * Remove temporary (cylindrical) obstacle with specified reference. The affected tiles
      * will be rebuilt. This operation is deferred and will happen in one of the next
      * update() calls. At one time only MAX_REQUESTS obstacles can be removed, or nothing will happen.
      **/
    bool removeTempObstacle(dtObstacleRef obstacleRef);

    /**
      * Remove a temporary (cylindrical) obstacle from the tilecache (as a deferred request).
      * Uses a ray query to find the temp obstacle.
      * The navmesh will be updated correspondingly after the next (one or many)
      * update() call as a deferred command.
      * At one time only MAX_REQUESTS obstacles can be removed, or nothing will happen.
      **/
    dtObstacleRef removeTempObstacle(Ogre::Vector3 raySource, Ogre::Vector3 rayHit);

    /**
      * Execute a ray intersection query to find the first temporary (cylindrical) obstacle that
      * hits the ray, if any.
      **/
    dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq);

    /**
      * Returns a list of tile references to compressed tiles that cover the specified bounding
      * box area.
      **/
    std::vector<dtCompressedTileRef> getTilesContainingBox(Ogre::Vector3 boxMin, Ogre::Vector3 boxMax);

    /**
      * Returns a list of tile references to compressed tiles that cover the area of a circle with
      * specified radius around the specified position.
      **/
    std::vector<dtCompressedTileRef> getTilesAroundPoint(Ogre::Vector3 point, Ogre::Real radius);

    /**
      * Add a convex shaped temporary obstacle to the tilecache in pretty much the same way as cylindrical
      * obstacles are added.
      * Currently this is implemented a lot less efficiently than cylindrical obstacles, as it issues a complete
      * rebuild of the affected tiles, instead of just cutting out the poly area of the obstacle.
      * This is a big TODO that I'm holding off because it requires changes to the recast libraries themselves.
      * I wait in hopes that this feature will appear in the original recast code.
      * In the meanwhile, if you are looking for this, someone implemented it and shared it on the mailing list:
      *     http://groups.google.com/group/recastnavigation/msg/92d5f131561ddad1
      * And corresponding ticket: http://code.google.com/p/recastnavigation/issues/detail?id=206
      *
      * The current implementation of convex obstacles is very simple and not deferred. Also obstacles
      * are stored in the inputGeom, which is not really nice.
      **/
//TODO by adding deferred tasking to add and removeConvexShapeObstacle one can add multiple shapes at once to the same tile without it being rebuilt multiple times
    int addConvexShapeObstacle(ConvexVolume *obstacle);

    /**
      * Remove convex obstacle from the tileCache. The affected navmesh tiles will be rebuilt.
      **/
    bool removeConvexShapeObstacle(ConvexVolume* convexHull);

    /**
      * Remove convex obstacle with specified id from the tileCache. The affected navmesh tiles will be rebuilt.
      * If removedObstacle is a valid pointer it will contain a reference to the removed obstacle.
      **/
    bool removeConvexShapeObstacleById(int obstacleIndex, ConvexVolume** removedObstacle = NULL);

    /**
      * Raycast the inputGeom and remove the hit convex obstacle. The affected navmesh tiles will be rebuilt.
      * If removedObstacle is a valid pointer it will contain a reference to the removed obstacle.
      **/
    int removeConvexShapeObstacle(Ogre::Vector3 raySource, Ogre::Vector3 rayHit, ConvexVolume** removedObstacle = NULL);

    /**
      * Returns the id of the specified convex obstacle. Returns -1 if this obstacle is not currently added to the tilecache.
      * Note: Ids are just array indexes and can change when obstacles are added or removed. Use with care!
      **/
    int getConvexShapeObstacleId(ConvexVolume *convexHull);

    /**
      * Returns the convex obstacle with specified id or index.
      **/
    ConvexVolume* getConvexShapeObstacle(int obstacleIndex);

    /**
      * Raycast inputGeom to find intersection with a convex obstacle. Returns the id of the hit
      * obstacle, -1 if none hit.
      **/
    int hitTestConvexShapeObstacle(Ogre::Vector3 raySource, Ogre::Vector3 rayHit);

    /**
      * Remove the tile with specified reference from the tilecache. The associated navmesh tile will also
      * be removed.
      **/
    bool removeTile(dtCompressedTileRef tileRef);

    /**
      * Debug draw the tile at specified grid location.
      **/
    void drawDetail(const int tx, const int ty);

    /**
      * Debug draw all tiles in the navmesh.
      **/
    void drawNavMesh(void);

    /**
      * Unused debug drawing function from the original recast demo.
      * Used for drawing the obstacles in the scene.
      * In this demo application we use the Obstacle class to represent obstacles in the scene.
      **/
    void drawObstacles(const dtTileCache* tc);


    /**
      * Ogre Recast component that holds the recast config and where the navmesh will be built.
      **/
    OgreRecast *m_recast;


    /**
     * Max number of layers a tile can have
     **/
    static const int EXPECTED_LAYERS_PER_TILE;

    /**
     * Max number of (temp) obstacles that can be added to the tilecache
     **/
    static const int MAX_OBSTACLES;

    /**
     *
     * Extra padding added to the border size of tiles (together with agent radius)
     **/
    static const float BORDER_PADDING;

    /**
      * Set to false to disable debug drawing. Improves performance.
      **/
    static bool DEBUG_DRAW;

    /**
      * Set to true to draw the bounding box of the tile areas that were rebuilt.
      **/
    static bool DEBUG_DRAW_REBUILT_BB;

protected:

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
      * Debug draw a navmesh poly
      **/
    void drawPolyMesh(const Ogre::String tileName, const struct dtTileCachePolyMesh &mesh, const float *orig, const float cs, const float ch, const struct dtTileCacheLayer &regionLayers, bool colorRegions=true);

    /**
      * Inits the tilecache. Helper used by constructors.
      **/
    bool initTileCache(void);


    /**
      * InputGeom from which the tileCache is initially inited (it's bounding box is considered the bounding box
      * for the entire world that the navmesh will cover). Tile build methods without specific geometry or entity
      * input will build navmesh from this geometry.
      * It also stored the convex temp obstacles. (will be gone in the future)
      * In the future this variable will probably disappear.
      **/
    InputGeom* m_geom;
// TODO maybe in the future I don't want to store inputgeom anymore, at the moment it's only used for adding convex shapes (what really should be done from compressed tiles instead of rebuilding from input geom) The whole navmesh can be stored as compressed tiles, the input geom does not need to be stored.

    /**
      * Set to true to keep intermediary results from navmesh build for debugging purposes.
      * Set to false to free up memory after navmesh was built.
      * Same as in official recast demo. (it's a checkbox in the gui)
      **/
    bool m_keepInterResults;

    /**
      * The tile cache memory allocator implementation used.
      **/
    struct LinearAllocator *m_talloc;
    /**
      * The tile compression implementation used.
      **/
    struct FastLZCompressor* m_tcomp;

    /**
      * Callback handler that processes right after processing
      * a tile mesh. Adds off-mesh connections to the mesh.
      **/
    struct MeshProcess *m_tmproc;

    /**
      * The detourTileCache component this class wraps.
      **/
    class dtTileCache *m_tileCache;

    /**
      * Recast config (copied from the OgreRecast component).
      **/
    rcConfig m_cfg;

    /**
      * DetourTileCache configuration parameters.
      **/
    dtTileCacheParams m_tcparams;

    /**
      * Context that stores temporary working variables when navmesh building.
      **/
    rcContext *m_ctx;

    /**
      * Metrics for measuring and profiling build times and memory usage.
      **/
    float m_cacheBuildTimeMs;
    int m_cacheCompressedSize;
    int m_cacheRawSize;
    int m_cacheLayerCount;
    int m_cacheBuildMemUsage;

    /**
      * Configuration parameters.
      **/
    int m_maxTiles;
    int m_maxPolysPerTile;
    int m_tileSize;

    float m_cellSize;

    /**
      * Size of the tile grid (x dimension)
      **/
    int m_tw;
    /**
      * Size of the tile grid (y dimension)
      **/
    int m_th;

    /**
      * Unused.
      * Could serve for deferring convex obstacle adding/removing requests.
      **/
    ConvexVolume* mChangedConvexVolumes[InputGeom::MAX_VOLUMES];    // TODO is this really MAX_VOLUMES? would be more like MAX_REQUESTS
    int mChangedConvexVolumesCount;

    /**
      * Pointer to debug drawn bounding box of rebuilt tiles.
      * Used when DEBUG_DRAW_REBUILT_BB is true.
      **/
    Ogre::ManualObject* mDebugRebuiltBB;
};

#endif // OGREDETOURTILECACHE_H
