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
    m_tcomp(0),
    m_geom(0),
    m_th(0),
    m_tw(0)
{
    m_talloc = new LinearAllocator(32000);
    m_tcomp = new FastLZCompressor;
    m_tmproc = new MeshProcess;
}

OgreDetourTileCache::~OgreDetourTileCache()
{
    //    dtFreeNavMesh(m_navMesh);
    //    m_navMesh = 0;
    dtFreeTileCache(m_tileCache);
}


bool OgreDetourTileCache::TileCacheBuild(std::vector<Ogre::Entity*> srcMeshes)
{
    rcContext *ctx = m_recast->m_ctx;

    dtStatus status;

    m_geom = new InputGeom(srcMeshes);

    if (!m_geom || m_geom->isEmpty()) {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
        return false;
    }

    m_tmproc->init(m_geom);


    // Init cache
    const float* bmin = m_geom->getMeshBoundsMin();
    const float* bmax = m_geom->getMeshBoundsMax();

    // Navmesh generation params.
    // Use config from recast module
    m_recast->configure();  // Set recast params
    rcConfig cfg = m_recast->m_cfg;

    // Most params are taken from OgreRecast::configure, except for these:
    cfg.tileSize = (int)m_tileSize;
    cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
    cfg.width = cfg.tileSize + cfg.borderSize*2;
    cfg.height = cfg.tileSize + cfg.borderSize*2;

    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);

    m_cellSize = cfg.cs;

    // Determine grid size (number of tiles)
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;    // Tile width
    const int th = (gh + ts-1) / ts;    // Tile height
    m_tw = tw;
    m_th = th;
    Ogre::LogManager::getSingletonPtr()->logMessage("Tiles: "+Ogre::StringConverter::toString(gw) + " x " + Ogre::StringConverter::toString(gh));


    // Max tiles and max polys affect how the tile IDs are caculated.
    // There are 22 bits available for identifying a tile and a polygon.
// TODO add more of the options available in the original recastDemo application GUI
    int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
    if (tileBits > 14) tileBits = 14;
    int polyBits = 22 - tileBits;
    m_maxTiles = 1 << tileBits;
    m_maxPolysPerTile = 1 << polyBits;
    Ogre::LogManager::getSingletonPtr()->logMessage("Max Tiles: " + Ogre::StringConverter::toString(m_maxTiles));
    Ogre::LogManager::getSingletonPtr()->logMessage("Max Polys: " + Ogre::StringConverter::toString(m_maxPolysPerTile));


    // Tile cache params.
    dtTileCacheParams tcparams;
    memset(&tcparams, 0, sizeof(tcparams));
    rcVcopy(tcparams.orig, bmin);
    // Copy some parameters from recast config
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


    // BUILD TileCache
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

    // TODO: this is redundant from OgreRecast class functionality. Leave those functions there and allocate the m_navMesh var in OgreRecast?
    // Init recast navmeshquery with created navmesh
    m_recast->m_navQuery = dtAllocNavMeshQuery();
    status = m_recast->m_navQuery->init(m_recast->m_navMesh, 2048);
    if (dtStatusFailed(status))
    {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
        return false;
    }



    // Preprocess tiles.
    // Prepares navmesh tiles in a 2D intermediary format that allows quick conversion to a 3D navmesh

//    ctx->resetTimers();

    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;

    for (int y = 0; y < th; ++y)
    {
        for (int x = 0; x < tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS];
            memset(tiles, 0, sizeof(tiles));
            int ntiles = rasterizeTileLayers(m_geom, x, y, cfg, tiles, MAX_LAYERS);  // This is where the tile is built

            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);  // Add tiles to tileCache
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
    // Builds detour compatible navmesh from all tiles.
    // A tile will have to be rebuilt if something changes, eg. a temporary obstacle is placed on it.
//    ctx->startTimer(RC_TIMER_TOTAL);
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x)
            m_tileCache->buildNavMeshTilesAt(x,y, m_recast->m_navMesh);
//    ctx->stopTimer(RC_TIMER_TOTAL);

//    m_cacheBuildTimeMs = ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
    m_cacheBuildMemUsage = m_talloc->high;


    // Count the total size of all generated tiles of the tiled navmesh
    const dtNavMesh* nav = m_recast->m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header)
            navmeshMemUsage += tile->dataSize;
    }



//    printf("navmeshMemUsage = %.1f kB\n", navmeshMemUsage/1024.0f);
    Ogre::LogManager::getSingletonPtr()->logMessage("navmeshMemUsage = "+ Ogre::StringConverter::toString(navmeshMemUsage/1024.0f) +" kB");


    return true;
}


int OgreDetourTileCache::rasterizeTileLayers(InputGeom* geom, const int tx, const int ty, const rcConfig& cfg, TileCacheData* tiles, const int maxTiles)
{
    if(!m_recast->m_ctx)
        m_recast->m_ctx=new rcContext(true);

    rcContext *ctx = m_recast->m_ctx;

    if (!geom || geom->isEmpty() || !geom->getChunkyMesh()) {
        ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
        return 0;
    }

    FastLZCompressor comp;
    RasterizationContext rc;

    const float* verts = geom->getVerts();
    const int nverts = geom->getVertCount();
    const rcChunkyTriMesh* chunkyMesh = geom->getChunkyMesh();

    // Tile bounds.
    const float tcs = m_tileSize * m_cellSize;

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


    // This is part of the regular recast navmesh generation pipeline as in OgreRecast::NavMeshBuild()
    // but only up till step 4 and slightly modified.


    // Allocate voxel heightfield where we rasterize our input data to.
    rc.solid = rcAllocHeightfield();
    if (!rc.solid)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return 0;
    }
    if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return 0;
    }

    // Allocate array that can hold triangle flags.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
    if (!rc.triareas)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
        return 0;
    }

    float tbmin[2], tbmax[2];
    tbmin[0] = tcfg.bmin[0];
    tbmin[1] = tcfg.bmin[2];
    tbmax[0] = tcfg.bmax[0];
    tbmax[1] = tcfg.bmax[2];
    int cid[512];// TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
    if (!ncid)
    {
        return 0; // empty
    }

    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* tris = &chunkyMesh->tris[node.i*3];
        const int ntris = node.n;

        memset(rc.triareas, 0, ntris*sizeof(unsigned char));
        rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);

        rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
    }

    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
    rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
    rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);


    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return 0;
    }
    if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return 0;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *rc.chf))
    {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return 0;
    }

    // (Optional) Mark areas.
    const ConvexVolume* vols = geom->getConvexVolumes();
    for (int i  = 0; i < geom->getConvexVolumeCount(); ++i)
    {
        rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts,
                             vols[i].hmin, vols[i].hmax,
                             (unsigned char)vols[i].area, *rc.chf);
    }



    // Up till this part was more or less the same as OgreRecast::NavMeshBuild()
    // The following part is specific for creating a 2D intermediary navmesh tile.

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
    if (!m_recast->m_navMesh)
        return;
    if (!m_tileCache)
        return;

    m_tileCache->update(dt, m_recast->m_navMesh);
}


void OgreDetourTileCache::getTilePos(const float* pos, int& tx, int& ty)
{
//    if (!m_geom) return;
// TODO is it correct to read from OgreRecast cfg here?
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


dtObstacleRef OgreDetourTileCache::addTempObstacle(Ogre::Vector3 pos)
{
    if (!m_tileCache)
        return 0;

    float p[3];
    OgreRecast::OgreVect3ToFloatA(pos, p);
    p[1] -= 0.5f;
    dtObstacleRef result;
    dtObstacleRef rval = m_tileCache->addObstacle(p, TEMP_OBSTACLE_RADIUS, TEMP_OBSTACLE_HEIGHT, &result);
    if(rval != DT_SUCCESS)
        return 0;

    return result;
}

dtObstacleRef OgreDetourTileCache::removeTempObstacle(const float* sp, const float* sq)
{
    if (!m_tileCache)
        return 0;
    dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
    m_tileCache->removeObstacle(ref);

    return ref;
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

void drawObstacles(const dtTileCache* tc)
{
        // Draw obstacles
        for (int i = 0; i < tc->getObstacleCount(); ++i)
        {
                const dtTileCacheObstacle* ob = tc->getObstacle(i);
                if (ob->state == DT_OBSTACLE_EMPTY) continue;
                float bmin[3], bmax[3];
                tc->getObstacleBounds(ob, bmin,bmax);

                unsigned int col = 0;
/*
                if (ob->state == DT_OBSTACLE_PROCESSING)
                        col = duRGBA(255,255,0,128);
                else if (ob->state == DT_OBSTACLE_PROCESSED)
                        col = duRGBA(255,192,0,192);
                else if (ob->state == DT_OBSTACLE_REMOVING)
                        col = duRGBA(220,0,0,128);
*/
// TODO draw in Ogre scene
//		duDebugDrawCylinder(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], col);
//		duDebugDrawCylinderWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duDarkenCol(col), 2);
        }
}


void OgreDetourTileCache::drawNavMesh()
{
    for (int y = 0; y < m_th; ++y)
    {
        for (int x = 0; x < m_tw; ++x)
        {
            drawDetail(x, y);
        }
    }
}

void OgreDetourTileCache::drawDetail(const int tx, const int ty)
{
    struct TileCacheBuildContext
    {
        inline TileCacheBuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
        inline ~TileCacheBuildContext() { purge(); }
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

    dtCompressedTileRef tiles[MAX_LAYERS];
    const int ntiles = m_tileCache->getTilesAt(tx,ty,tiles,MAX_LAYERS);

    dtTileCacheAlloc* talloc = m_tileCache->getAlloc();
    dtTileCacheCompressor* tcomp = m_tileCache->getCompressor();
    const dtTileCacheParams* params = m_tileCache->getParams();

    for (int i = 0; i < ntiles; ++i)
    {
        const dtCompressedTile* tile = m_tileCache->getTileByRef(tiles[i]);

        talloc->reset();

        TileCacheBuildContext bc(talloc);
        const int walkableClimbVx = (int)(params->walkableClimb / params->ch);
        dtStatus status;

        // Decompress tile layer data.
        status = dtDecompressTileCacheLayer(talloc, tcomp, tile->data, tile->dataSize, &bc.layer);
        if (dtStatusFailed(status))
            return;

        // Build navmesh
        status = dtBuildTileCacheRegions(talloc, *bc.layer, walkableClimbVx);
        if (dtStatusFailed(status))
            return;

        bc.lcset = dtAllocTileCacheContourSet(talloc);
        if (!bc.lcset)
            return;
        status = dtBuildTileCacheContours(talloc, *bc.layer, walkableClimbVx,
                                          params->maxSimplificationError, *bc.lcset);
        if (dtStatusFailed(status))
            return;

        bc.lmesh = dtAllocTileCachePolyMesh(talloc);
        if (!bc.lmesh)
            return;
        status = dtBuildTileCachePolyMesh(talloc, *bc.lcset, *bc.lmesh);
        if (dtStatusFailed(status))
            return;

        // Draw navmesh
//        duDebugDrawTileCachePolyMesh(dd, *bc.lmesh, tile->header->bmin, params->cs, params->ch);
        drawPolyMesh(*bc.lmesh, tile->header->bmin, params->cs, params->ch, *bc.layer);

    }
}

void OgreDetourTileCache::drawPolyMesh(const struct dtTileCachePolyMesh &mesh, const float *orig, const float cs, const float ch, const struct dtTileCacheLayer &regionLayers, bool colorRegions)
{
            const int nvp = mesh.nvp;

            const unsigned short* verts = mesh.verts;
            const unsigned short* polys = mesh.polys;
            const unsigned char* areas = mesh.areas;
            const unsigned char *regs = regionLayers.regs;
            const int nverts = mesh.nverts;
            const int npolys = mesh.npolys;
            const int maxpolys = m_maxPolysPerTile;

            unsigned short *regions = new unsigned short[npolys];
            for (int i =0; i< npolys; i++) {
                regions[i] = (const unsigned short)regs[i];
            }

            m_recast->CreateRecastPolyMesh(verts, nverts, polys, npolys, areas, maxpolys, regions, nvp, cs, ch, orig, colorRegions);

            delete[] regions;
}
