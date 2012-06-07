#include "OgreDetourTileCache.h"
#include "DetourTileCache/DetourTileCache.h"
#include <float.h>

static const int EXPECTED_LAYERS_PER_TILE = 4;

OgreDetourTileCache::OgreDetourTileCache(OgreRecast *recast)
    : m_recast(recast),
    m_keepInterResults(false),
    m_tileCache(0),
    m_cacheBuildTimeMs(0),
    m_cacheCompressedSize(0),
    m_cacheRawSize(0),
    m_cacheLayerCount(0),
    m_cacheBuildMemUsage(0),
    m_maxTiles(0),
    m_maxPolysPerTile(0),
    m_tileSize(48),
    m_cellSize(0),
    m_tcomp(0)
{
    m_talloc = new LinearAllocator(32000);
    m_tmproc = new MeshProcess;
}

OgreDetourTileCache::~OgreDetourTileCache()
{
    //    dtFreeNavMesh(m_navMesh);
    //    m_navMesh = 0;
    dtFreeTileCache(m_tileCache);
}


bool OgreDetourTileCache::TileCacheBuild()
{
    rcContext *ctx = m_recast->m_ctx;

    dtStatus status;


    //        m_tmproc->init(m_geom);


    // Use config params set in recast module
    rcConfig cfg = m_recast->m_cfg;

    m_cellSize = cfg.cs;


    // Init cache
//    const float* bmin = m_geom->getMeshBoundsMin();
//    const float* bmax = m_geom->getMeshBoundsMax();
    const float *bmin = cfg.bmin;
    const float *bmax = cfg.bmax;

    // Determine grid size (number of tiles)
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;    // Tile width
    const int th = (gh + ts-1) / ts;    // Tile height

    Ogre::LogManager::getSingletonPtr()->logMessage("Tiles: "+Ogre::StringConverter::toString(gw) + " x " + Ogre::StringConverter::toString(gh));

    // Max tiles and max polys affect how the tile IDs are caculated.
    // There are 22 bits available for identifying a tile and a polygon.
    int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
    if (tileBits > 14) tileBits = 14;
    int polyBits = 22 - tileBits;
    m_maxTiles = 1 << tileBits;
    m_maxPolysPerTile = 1 << polyBits;
    Ogre::LogManager::getSingletonPtr()->logMessage("Max Tiles: " + Ogre::StringConverter::toString(m_maxTiles));
    Ogre::LogManager::getSingletonPtr()->logMessage("Max Polys: " + Ogre::StringConverter::toString(m_maxPolysPerTile));

    // Generation params (some are taken from OgreRecast module)
    cfg.tileSize = (int)m_tileSize;
    cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
    cfg.width = cfg.tileSize + cfg.borderSize*2;
    cfg.height = cfg.tileSize + cfg.borderSize*2;

    // Tile cache params.
    dtTileCacheParams tcparams;
    memset(&tcparams, 0, sizeof(tcparams));
    rcVcopy(tcparams.orig, bmin);
    tcparams.cs = cfg.cs;
    tcparams.ch = cfg.ch;
    tcparams.width = (int)m_tileSize;
    tcparams.height = (int)m_tileSize;
    tcparams.walkableHeight = cfg.walkableHeight;
    tcparams.walkableRadius = cfg.walkableRadius;
    tcparams.walkableClimb = cfg.walkableClimb;
    tcparams.maxSimplificationError = cfg.maxSimplificationError;
    tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
    tcparams.maxObstacles = 128;    // Max number of temp obstacles that can be added to or removed from navmesh

    dtFreeTileCache(m_tileCache);

    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
        return false;
    }
    status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
        return false;
    }

    dtFreeNavMesh(m_recast->m_navMesh);

    m_recast->m_navMesh = dtAllocNavMesh();
    if (!m_recast->m_navMesh)
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
        return false;
    }

    // Init multi-tile navmesh parameters
    dtNavMeshParams params;
    memset(&params, 0, sizeof(params));
    rcVcopy(params.orig, tcparams.orig);   // Set world-space origin of tile grid
    params.tileWidth = m_tileSize*tcparams.cs;
    params.tileHeight = m_tileSize*tcparams.cs;
    params.maxTiles = m_maxTiles;
    params.maxPolys = m_maxPolysPerTile;

    status = m_recast->m_navMesh->init(&params);
    if (dtStatusFailed(status))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
        return false;
    }

/*
    status = m_navQuery->init(m_navMesh, 2048);
    if (dtStatusFailed(status))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
        return false;
    }
*/

    // Preprocess tiles.

    ctx->resetTimers();

    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;

    for (int y = 0; y < th; ++y)
    {
        for (int x = 0; x < tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS];
            memset(tiles, 0, sizeof(tiles));
            int ntiles = rasterizeTileLayers(x, y, tiles, MAX_LAYERS);

            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
                if (dtStatusFailed(status))
                {
                    dtFree(tile->data);
                    tile->data = 0;
                    continue;
                }

                m_cacheLayerCount++;
                m_cacheCompressedSize += tile->dataSize;
                m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
            }
        }
    }

    // Build initial meshes
    ctx->startTimer(RC_TIMER_TOTAL);
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x)
            m_tileCache->buildNavMeshTilesAt(x,y, m_recast->m_navMesh);
    ctx->stopTimer(RC_TIMER_TOTAL);

    m_cacheBuildTimeMs = ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
    m_cacheBuildMemUsage = m_talloc->high;


    const dtNavMesh* nav = m_recast->m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header)
            navmeshMemUsage += tile->dataSize;
    }
    printf("navmeshMemUsage = %.1f kB", navmeshMemUsage/1024.0f);


    return true;
}


int OgreDetourTileCache::rasterizeTileLayers(const int tx, const int ty, TileCacheData* tiles, const int maxTiles)
{
    // Continue partial process from OgreRecast::NavMeshBuild()


    FastLZCompressor comp;

    RasterizationContext rc;
    rcContext *ctx = m_recast->m_ctx;
    rcConfig cfg;
    memcpy(&cfg, &(m_recast->m_cfg), sizeof(cfg));


    // Tile bounds.
    const float tcs = cfg.tileSize * cfg.cs;

    rcConfig tcfg;
    memcpy(&tcfg, &cfg, sizeof(tcfg));

    tcfg.bmin[0] = cfg.bmin[0] + tx*tcs;
    tcfg.bmin[1] = cfg.bmin[1];
    tcfg.bmin[2] = cfg.bmin[2] + ty*tcs;
    tcfg.bmax[0] = cfg.bmin[0] + (tx+1)*tcs;
    tcfg.bmax[1] = cfg.bmax[1];
    tcfg.bmax[2] = cfg.bmin[2] + (ty+1)*tcs;
    tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
    tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;


    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
        return 0;
    }
    if (!rcBuildHeightfieldLayers(ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
        return 0;
    }

    rc.ntiles = 0;
    for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
    {
        TileCacheData* tile = &rc.tiles[rc.ntiles++];
        const rcHeightfieldLayer* layer = &rc.lset->layers[i];

        // Store header
        dtTileCacheLayerHeader header;
        header.magic = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;

        // Tile layer location in the navmesh.
        header.tx = tx;
        header.ty = ty;
        header.tlayer = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);

        // Tile info.
        header.width = (unsigned char)layer->width;
        header.height = (unsigned char)layer->height;
        header.minx = (unsigned char)layer->minx;
        header.maxx = (unsigned char)layer->maxx;
        header.miny = (unsigned char)layer->miny;
        header.maxy = (unsigned char)layer->maxy;
        header.hmin = (unsigned short)layer->hmin;
        header.hmax = (unsigned short)layer->hmax;

        dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
                                                &tile->data, &tile->dataSize);
        if (dtStatusFailed(status))
        {
            return 0;
        }
    }

    // Transfer ownsership of tile data from build context to the caller.
    int n = 0;
    for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
    {
        tiles[n++] = rc.tiles[i];
        rc.tiles[i].data = 0;
        rc.tiles[i].dataSize = 0;
    }

    return n;
}



void OgreDetourTileCache::handleUpdate(const float dt)
{
    handleUpdate(dt);

    if (!m_recast->m_navMesh)
        return;
    if (!m_tileCache)
        return;

    m_tileCache->update(dt, m_recast->m_navMesh);
}


void OgreDetourTileCache::getTilePos(const float* pos, int& tx, int& ty)
{
//    if (!m_geom) return;

    const float* bmin = m_recast->m_cfg.bmin;

    const float ts = m_tileSize*m_cellSize;
    tx = (int)((pos[0] - bmin[0]) / ts);
    ty = (int)((pos[2] - bmin[2]) / ts);
}


void OgreDetourTileCache::handleMeshChanged(class InputGeom* geom)
{
//    Sample::handleMeshChanged(geom);

    dtFreeTileCache(m_tileCache);
    m_tileCache = 0;

    dtFreeNavMesh(m_recast->m_navMesh);
    m_recast->m_navMesh = 0;

}


void OgreDetourTileCache::clearAllTempObstacles()
{
    if (!m_tileCache)
        return;
    for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY) continue;
        m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
    }
}


void OgreDetourTileCache::addTempObstacle(const float* pos)
{
    if (!m_tileCache)
        return;
    float p[3];
    dtVcopy(p, pos);
    p[1] -= 0.5f;
    m_tileCache->addObstacle(p, 1.0f, 2.0f, 0);
}

void OgreDetourTileCache::removeTempObstacle(const float* sp, const float* sq)
{
    if (!m_tileCache)
        return;
    dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
    m_tileCache->removeObstacle(ref);
}

static bool isectSegAABB(const float* sp, const float* sq,
                                                 const float* amin, const float* amax,
                                                 float& tmin, float& tmax)
{
        static const float EPS = 1e-6f;

        float d[3];
        rcVsub(d, sq, sp);
        tmin = 0;  // set to -FLT_MAX to get first hit on line
        tmax = FLT_MAX;		// set to max distance ray can travel (for segment)

        // For all three slabs
        for (int i = 0; i < 3; i++)
        {
                if (fabsf(d[i]) < EPS)
                {
                        // Ray is parallel to slab. No hit if origin not within slab
                        if (sp[i] < amin[i] || sp[i] > amax[i])
                                return false;
                }
                else
                {
                        // Compute intersection t value of ray with near and far plane of slab
                        const float ood = 1.0f / d[i];
                        float t1 = (amin[i] - sp[i]) * ood;
                        float t2 = (amax[i] - sp[i]) * ood;
                        // Make t1 be intersection with near plane, t2 with far plane
                        if (t1 > t2) rcSwap(t1, t2);
                        // Compute the intersection of slab intersections intervals
                        if (t1 > tmin) tmin = t1;
                        if (t2 < tmax) tmax = t2;
                        // Exit with no collision as soon as slab intersection becomes empty
                        if (tmin > tmax) return false;
                }
        }

        return true;
}

dtObstacleRef OgreDetourTileCache::hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
        float tmin = FLT_MAX;
        const dtTileCacheObstacle* obmin = 0;
        for (int i = 0; i < tc->getObstacleCount(); ++i)
        {
                const dtTileCacheObstacle* ob = tc->getObstacle(i);
                if (ob->state == DT_OBSTACLE_EMPTY)
                        continue;

                float bmin[3], bmax[3], t0,t1;
                tc->getObstacleBounds(ob, bmin,bmax);

                if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
                {
                        if (t0 < tmin)
                        {
                                tmin = t0;
                                obmin = ob;
                        }
                }
        }
        return tc->getObstacleRef(obmin);
}

