#include "OgreRecastDemo.h"


OgreRecastDemo::OgreRecastDemo(Ogre::SceneManager* sceneMgr)
    : m_pSceneMgr(sceneMgr)
{
   // Init recast stuff in a safe state
   
   m_triareas=NULL;
   m_solid=NULL ;
   m_chf=NULL ;
   m_cset=NULL;
   m_pmesh=NULL;
   //m_cfg;   
   m_dmesh=NULL ;
   m_geom=NULL;
   m_navMesh=NULL;
   m_navQuery=NULL;
   //m_navMeshDrawFlags;
   m_ctx=NULL ;

   RecastCleanup() ; //?? don't know if I should do this prior to making any recast stuff, but the demo did.
   m_pRecastMOPath=NULL ;


   m_pLog = Ogre::LogManager::getSingletonPtr();
}


/**
 * Cleanup recast stuff, not debug manualobjects.
**/
void OgreRecastDemo::RecastCleanup()
{
   if(m_triareas) delete [] m_triareas;
   m_triareas = 0;

   rcFreeHeightField(m_solid);
   m_solid = 0;
   rcFreeCompactHeightfield(m_chf);
   m_chf = 0;
   rcFreeContourSet(m_cset);
   m_cset = 0;
   rcFreePolyMesh(m_pmesh);
   m_pmesh = 0;
   rcFreePolyMeshDetail(m_dmesh);
   m_dmesh = 0;
   dtFreeNavMesh(m_navMesh);
   m_navMesh = 0;

   dtFreeNavMeshQuery(m_navQuery);
   m_navQuery = 0 ;

   if(m_ctx) delete m_ctx ;
}







/**
 * Now for the navmesh creation function. 
 * I've mostly taken this from the demo, apart from the top part where I create the triangles. Recast needs a bunch of input vertices and triangles from your map to build the navigation mesh. Where you get those verts amd triangles is up to you, my map loader was already outputing verts and triangle so it was easy to use those. Make sure the triangles wind the correct way or Recast will try to build the navmesh on the outside of your map.
 * There's some potentially groovy stuff in there that I haven't touched, like filtering and different weights for different types of zones. Also I've just gone for the simplest navmesh type, there's also other modes like tiling navmeshes which I've ignored.
 *
 * Perhaps the most important part of the above is setting the agent size with m_agentHeight and m_agentRadius, and the voxel cell size used, m_cellSize and m_cellHeight. In my project 32.0 units is 1 meter, so I've set the agent to 48 units high, and the cell sizes are quite large. The original cell sizes in the Recast/Detour demo were down around 0.3.
**/
bool OgreRecastDemo::NavMeshBuild(Ogre::Entity* srcMesh)
{
    // TODO: clean up unused variables


   // convert our geometry into the recast format

   m_pLog->logMessage("NavMeshBuild Start");



   int nLoop=0 ;
   
   /*
   float*           rc_verts;
   unsigned int     rc_nverts;
   int*             rc_tris;
   float*           rc_trinorms;
   unsigned int     rc_ntris;
   float              rc_bmin[3];
   float              rc_bmax[3];

   Ogre::Vector3 VertA ;
   Ogre::Vector3 VertB ;
   Ogre::Vector3 VertC ;
   Ogre::Vector3 TriNorm ;
   int nVert=0 ;
   */



   //RecastCleanup() ;



   m_ctx=new rcContext(true) ;




   //
   // Step 1. Initialize build config.
   //

   // NOTE: this is one of the most important parts to get it right!!
    /*
      Perhaps the most important part of the above is setting the agent size with m_agentHeight and m_agentRadius, and the voxel cell size used, m_cellSize and m_cellHeight. In my project 32.0 units is 1 meter, so I've set the agent to 48 units high, and the cell sizes are quite large. The original cell sizes in the Recast/Detour demo were down around 0.3.
      */


   // cellsize (1.5, 1.0) was the most accurate at finding all the places we could go, but was also slow to generate.
   // Might be suitable for pre-generated meshes. Though it also produces a lot more polygons.

   // TODO clean this up, put in some more clear place, allow config file
   m_cellSize = /*9.0 ;//*/0.3;         //*
   m_cellHeight = /*6.0 ;//*/0.2;       //*
   m_agentMaxSlope = /*45*/20;          //*
   m_agentHeight = 2.5/*64.0;  1*/;        //*
   m_agentMaxClimb = 16;                //*
   m_agentRadius = /*16;*/0.5;          //*
   m_edgeMaxLen = 12/*512*/;
   m_edgeMaxError = 1.3;
   m_regionMinSize = 50;
   m_regionMergeSize = 20;
   m_vertsPerPoly = 6;
   m_detailSampleDist = 6;
   m_detailSampleMaxError = 1;
   m_keepInterResults = false;
   
   // Init build configuration from GUI
   memset(&m_cfg, 0, sizeof(m_cfg));
   m_cfg.cs = m_cellSize;
   m_cfg.ch = m_cellHeight;
   m_cfg.walkableSlopeAngle = m_agentMaxSlope;
   m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
   m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
   m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
   m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
   m_cfg.maxSimplificationError = m_edgeMaxError;
   m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);      // Note: area = size*size
   m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);   // Note: area = size*size
   m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
   m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
   m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;


   m_navMeshOffsetFromGround = m_cellHeight/5;//0.25;      // Distance above ground for drawing navmesh polygons
   m_navMeshEdgesOffsetFromGround = m_cellHeight/3;        // Distance above ground for drawing edges of navmesh (should be slightly higher than navmesh polygons)
   m_pathOffsetFromGround = m_agentHeight+m_navMeshOffsetFromGround; // Distance above ground for drawing path debug lines relative to cellheight (should be higher than navmesh polygons)

   m_navmeshNeighbourEdgeCol= Ogre::ColourValue(0.9,0.9,0.9);   // Light Grey
   m_navmeshOuterEdgeCol    = Ogre::ColourValue(0,0,0);         // Black
   m_navmeshGroundPolygonCol= Ogre::ColourValue(0,0.7,0);       // Green
   m_navmeshOtherPolygonCol = Ogre::ColourValue(0,0.175,0);     // Dark green
   m_pathCol                = Ogre::ColourValue(1,0,0);         // Red

   

   // Set the area where the navigation mesh will be build.
   // Using bounding box of source mesh and specified cell size
   const Ogre::AxisAlignedBox srcMeshBB = srcMesh->getBoundingBox();
   float tmpV[3];
   OgreVect3ToFloatA(srcMeshBB.getMinimum(), tmpV);
   rcVcopy(m_cfg.bmin, tmpV);
   OgreVect3ToFloatA(srcMeshBB.getMaximum(), tmpV);
   rcVcopy(m_cfg.bmax, tmpV);
   rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

   // Reset build times gathering.
   m_ctx->resetTimers();

   // Start the build process.
   m_ctx->startTimer(RC_TIMER_TOTAL);







   //
   // Step 2. Rasterize input polygon soup.
   //
   size_t meshVertexCount, meshIndexCount = 0;
   Ogre::Vector3* meshVertices = 0;
   unsigned long* meshIndices = 0;
   getMeshInformation(srcMesh->getMesh(), meshVertexCount, meshVertices, meshIndexCount, meshIndices);

   m_pLog->logMessage("Building navigation:");
   m_pLog->logMessage(" - " + Ogre::StringConverter::toString(m_cfg.width) + " x " + Ogre::StringConverter::toString(m_cfg.height) + " cells");
   m_pLog->logMessage(" - " + Ogre::StringConverter::toString(meshVertexCount/1000.0f) + " K verts, " + Ogre::StringConverter::toString(meshIndexCount/1000.0f) + " K tris");

   // DECLARE RECAST DATA BUFFERS USING THE INFO WE GRABBED ABOVE
   float *verts = new float[meshVertexCount*3];// *3 as verts holds x,y,&z for each verts in the array
   int *tris = new int[meshIndexCount];// tris in recast is really indices like ogre
   
   //set the reference node
   Ogre::SceneNode *rootNode = m_pSceneMgr->getRootSceneNode();

   //find the transform between the root node and this node
   Ogre::Matrix4 transform = rootNode->_getFullTransform().inverse() * srcMesh->getParentNode()->_getFullTransform();
   Ogre::Vector3 vertexPos;
   uint vertsIndex = 0;
   // copy all meshes verticies into single buffer and transform to world space relative to parentNode
   for (uint j = 0 ; j < meshVertexCount ; j++)
   {
       vertexPos = transform*meshVertices[j];
       verts[vertsIndex] = vertexPos.x;
       verts[vertsIndex+1] = vertexPos.y;
       verts[vertsIndex+2] = vertexPos.z;
       vertsIndex+=3;
   }

   for (uint j = 0 ; j < meshIndexCount ; j++)
   {
       tris[j] = meshIndices[j];
   }

   //delete[] meshVertices;
   //delete[] meshIndices;

   // calculate normals data for Recast - im not 100% sure where this is required
   // but it is used, Ogre handles its own Normal data for rendering, this is not related
   // to Ogre at all ( its also not correct lol )
   // TODO : fix this
   int ntris = meshIndexCount/3;
   int nverts = meshVertexCount;
   m_normals = new float[ntris*3];
   for (int i = 0; i < ntris*3; i += 3)
   {
       const float* v0 = &verts[tris[i]*3];
       const float* v1 = &verts[tris[i+1]*3];
       const float* v2 = &verts[tris[i+2]*3];
       float e0[3], e1[3];
       for (int j = 0; j < 3; ++j)
       {
           e0[j] = (v1[j] - v0[j]);
           e1[j] = (v2[j] - v0[j]);
       }
       float* n = &m_normals[i];
       n[0] = ((e0[1]*e1[2]) - (e0[2]*e1[1]));
       n[1] = ((e0[2]*e1[0]) - (e0[0]*e1[2]));
       n[2] = ((e0[0]*e1[1]) - (e0[1]*e1[0]));

       float d = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
       if (d > 0)
       {
           d = 1.0f/d;
           n[0] *= d;
           n[1] *= d;
           n[2] *= d;
       }
   }


   // Allocate voxel heightfield where we rasterize our input data to.
   m_solid = rcAllocHeightfield();
   if (!m_solid)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
      return false;
   }
   if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
      return false;
   }
   
   // Allocate array that can hold triangle area types.
   // If you have multiple meshes you need to process, allocate
   // an array which can hold the max number of triangles you need to process.
   m_triareas = new unsigned char[ntris];
   if (!m_triareas)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
      return false;
   }
   
   // Find triangles which are walkable based on their slope and rasterize them.
   // If your input data is multiple meshes, you can transform them here, calculate
   // the are type for each of the meshes and rasterize them.
   memset(m_triareas, 0, ntris*sizeof(unsigned char));
   rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
   rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb);

   if (!m_keepInterResults)
   {
      delete [] m_triareas;
      m_triareas = 0;
   }





   //
   // Step 3. Filter walkables surfaces.
   //
   
   // Once all geoemtry is rasterized, we do initial pass of filtering to
   // remove unwanted overhangs caused by the conservative rasterization
   // as well as filter spans where the character cannot possibly stand.
   rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
   rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
   rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);








   //
   // Step 4. Partition walkable surface to simple regions.
   //

   // Compact the heightfield so that it is faster to handle from now on.
   // This will result more cache coherent data as well as the neighbours
   // between walkable cells will be calculated.
   m_chf = rcAllocCompactHeightfield();
   if (!m_chf)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
      return false;
   }
   if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
      return false;
   }
   
   if (!m_keepInterResults)
   {
      rcFreeHeightField(m_solid);
      m_solid = 0;
   }


   // Erode the walkable area by agent radius.
   if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
      return false;
   }

   // (Optional) Mark areas.
   //const ConvexVolume* vols = m_geom->getConvexVolumes();
   //for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
   //   rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);
   
   // Prepare for region partitioning, by calculating distance field along the walkable surface.
   if (!rcBuildDistanceField(m_ctx, *m_chf))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
      return false;
   }

   // Partition the walkable surface into simple regions without holes.
   if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
      return false;
   }








   //
   // Step 5. Trace and simplify region contours.
   //
   
   // Create contours.
   m_cset = rcAllocContourSet();
   if (!m_cset)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
      return false;
   }
   if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
      return false;
   }

   if (m_cset->nconts == 0)
   {
       // In case of errors see: http://groups.google.com/group/recastnavigation/browse_thread/thread/a6fbd509859a12c8
       // You should probably tweak the parameters (at the top of this method)
           m_pLog->logMessage("ERROR: No contours created (Recast)!");
    }





   //
   // Step 6. Build polygons mesh from contours.
   //
   
   // Build polygon navmesh from the contours.
   m_pmesh = rcAllocPolyMesh();
   if (!m_pmesh)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
      return false;
   }
   if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
      return false;
   }
   








   //
   // Step 7. Create detail mesh which allows to access approximate height on each polygon.
   //
   
   m_dmesh = rcAllocPolyMeshDetail();
   if (!m_dmesh)
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
      return false;
   }

   if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
   {
      m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
      return false;
   }

   if (!m_keepInterResults)
   {
      rcFreeCompactHeightfield(m_chf);
      m_chf = 0;
      rcFreeContourSet(m_cset);
      m_cset = 0;
   }

   // At this point the navigation mesh data is ready, you can access it from m_pmesh.
   // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
   







   //
   // (Optional) Step 8. Create Detour data from Recast poly mesh.
   //
   
   // The GUI may allow more max points per polygon than Detour can handle.
   // Only build the detour navmesh if we do not exceed the limit.


   if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
   {
      m_pLog->logMessage("Detour 1000");

      unsigned char* navData = 0;
      int navDataSize = 0;

      
      // Update poly flags from areas.
      for (int i = 0; i < m_pmesh->npolys; ++i)
      {
         if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
         {
            m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
            m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
         }
      }
      

      // Set navmesh params
      dtNavMeshCreateParams params;
      memset(&params, 0, sizeof(params));
      params.verts = m_pmesh->verts;
      params.vertCount = m_pmesh->nverts;
      params.polys = m_pmesh->polys;
      params.polyAreas = m_pmesh->areas;
      params.polyFlags = m_pmesh->flags;
      params.polyCount = m_pmesh->npolys;
      params.nvp = m_pmesh->nvp;
      params.detailMeshes = m_dmesh->meshes;
      params.detailVerts = m_dmesh->verts;
      params.detailVertsCount = m_dmesh->nverts;
      params.detailTris = m_dmesh->tris;
      params.detailTriCount = m_dmesh->ntris;

      // no off mesh connections yet
      m_offMeshConCount=0 ;
      params.offMeshConVerts = m_offMeshConVerts ;
      params.offMeshConRad = m_offMeshConRads ;
      params.offMeshConDir = m_offMeshConDirs ;
      params.offMeshConAreas = m_offMeshConAreas ;
      params.offMeshConFlags = m_offMeshConFlags ;
      params.offMeshConUserID = m_offMeshConId ;
      params.offMeshConCount = m_offMeshConCount ;

      params.walkableHeight = m_agentHeight;
      params.walkableRadius = m_agentRadius;
      params.walkableClimb = m_agentMaxClimb;
      rcVcopy(params.bmin, m_pmesh->bmin);
      rcVcopy(params.bmax, m_pmesh->bmax);
      params.cs = m_cfg.cs;
      params.ch = m_cfg.ch;

      
      m_pLog->logMessage("Detour 2000");

      if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
      {
         m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
         return false;
      }

      m_pLog->logMessage("Detour 3000");
      
      m_navMesh = dtAllocNavMesh();
      if (!m_navMesh)
      {
         dtFree(navData);
         m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
         return false;
      }

      m_pLog->logMessage("Detour 4000");
      
      dtStatus status;
      
      status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
      if (dtStatusFailed(status))
      {
         dtFree(navData);
         m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
         return false;
      }

      m_pLog->logMessage("Detour 5000");
      
      m_navQuery = dtAllocNavMeshQuery();
      status = m_navQuery->init(m_navMesh, 2048);

      m_pLog->logMessage("Detour 5500");

      if (dtStatusFailed(status))
      {
         m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
         return false;
      }

      m_pLog->logMessage("Detour 6000");
   }
   
   m_ctx->stopTimer(RC_TIMER_TOTAL);

   
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


   // cleanup stuff we don't need
//   delete [] rc_verts ;
//   delete [] rc_tris ;
//   delete [] rc_trinorms ;

   //CreateRecastPolyMesh(*m_pmesh) ;   // Debug render it

   m_pLog->logMessage("NavMeshBuild End");
   return true;
}






#include <math.h>


/**
 * Now for the pathfinding code. 
 * This takes a start point and an end point and, if possible, generates a list of lines in a path. It might fail if the start or end points aren't near any navmesh polygons, or if the path is too long, or it can't make a path, or various other reasons. So far I've not had problems though.
 *
 * nTarget: The index number for the slot in which the found path is to be stored
 * nPathSlot: Number identifying the target the path leads to
 *
 * Return codes:
 *  0   found path
 *  -1  Couldn't find polygon nearest to start point
 *  -2  Couldn't find polygon nearest to end point
 *  -3  Couldn't create a path
 *  -4  Couldn't find a path
 *  -5  Couldn't create a straight path
 *  -6  Couldn't find a straight path
**/
int OgreRecastDemo::FindPath(float* pStartPos, float* pEndPos, int nPathSlot, int nTarget)
{
   dtStatus status ;
   float pExtents[3]={32.0f, 32.0f, 32.0f} ; // size of box around start/end points to look for nav polygons
   dtPolyRef StartPoly ;
   float StartNearest[3] ;
   dtPolyRef EndPoly ;
   float EndNearest[3] ;
   dtPolyRef PolyPath[MAX_PATHPOLY] ;
   int nPathCount=0 ;
   float StraightPath[MAX_PATHVERT*3] ;
   int nVertCount=0 ;


   // setup the filter
   dtQueryFilter Filter;
   Filter.setIncludeFlags(0xFFFF) ;
   Filter.setExcludeFlags(0) ;
   Filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f) ;

   // find the start polygon
   status=m_navQuery->findNearestPoly(pStartPos, pExtents, &Filter, &StartPoly, StartNearest) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -1 ; // couldn't find a polygon

   // find the end polygon
   status=m_navQuery->findNearestPoly(pEndPos, pExtents, &Filter, &EndPoly, EndNearest) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -2 ; // couldn't find a polygon

   status=m_navQuery->findPath(StartPoly, EndPoly, StartNearest, EndNearest, &Filter, PolyPath, &nPathCount, MAX_PATHPOLY) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -3 ; // couldn't create a path
   if(nPathCount==0) return -4 ; // couldn't find a path

   status=m_navQuery->findStraightPath(StartNearest, EndNearest, PolyPath, nPathCount, StraightPath, NULL, NULL, &nVertCount, MAX_PATHVERT) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -5 ; // couldn't create a path
   if(nVertCount==0) return -6 ; // couldn't find a path

   // At this point we have our path.  Copy it to the path store
   int nIndex=0 ;
   for(int nVert=0 ; nVert<nVertCount ; nVert++)
   {
      m_PathStore[nPathSlot].PosX[nVert]=StraightPath[nIndex++] ;
      m_PathStore[nPathSlot].PosY[nVert]=StraightPath[nIndex++] ;
      m_PathStore[nPathSlot].PosZ[nVert]=StraightPath[nIndex++] ;

      //sprintf(m_chBug, "Path Vert %i, %f %f %f", nVert, m_PathStore[nPathSlot].PosX[nVert], m_PathStore[nPathSlot].PosY[nVert], m_PathStore[nPathSlot].PosZ[nVert]) ;
      //m_pLog->logMessage(m_chBug);
   }
   m_PathStore[nPathSlot].MaxVertex=nVertCount ;
   m_PathStore[nPathSlot].Target=nTarget ;

   return nVertCount ;

}

int OgreRecastDemo::FindPath(Ogre::Vector3 startPos, Ogre::Vector3 endPos, int nPathSlot, int nTarget)
{
    float start[3];
    float end[3];
    OgreVect3ToFloatA(startPos, start);
    OgreVect3ToFloatA(endPos, end);

    return FindPath(start,end,nPathSlot,nTarget);
}







/**
 * Debug drawing functionality:
**/

void OgreRecastDemo::drawNavMesh() {
    CreateRecastPolyMesh(*m_pmesh);
}

void OgreRecastDemo::CreateRecastPolyMesh(const struct rcPolyMesh& mesh, bool colorRegions)
{
   const int nvp = mesh.nvp; 
   const float cs = mesh.cs;
   const float ch = mesh.ch;
   const float* orig = mesh.bmin;

   m_flDataX=mesh.npolys ;
   m_flDataY=mesh.nverts ;

   // When drawing regions choose different random colors for each region
   Ogre::ColourValue* regionColors = NULL;
   if(colorRegions) {
       regionColors = new Ogre::ColourValue[mesh.maxpolys];
       for (int i = 0; i < mesh.maxpolys; ++i) {
           regionColors[i] = Ogre::ColourValue(Ogre::Math::RangeRandom(0,1), Ogre::Math::RangeRandom(0,1), Ogre::Math::RangeRandom(0,1), 1);
       }
   }
   
   // create scenenodes
   m_pRecastSN=m_pSceneMgr->getRootSceneNode()->createChildSceneNode("RecastSN") ;

   int nIndex=0 ;
   m_nAreaCount=mesh.npolys;


   if(m_nAreaCount)
   {

      // start defining the manualObject with the navmesh planes
      m_pRecastMOWalk = m_pSceneMgr->createManualObject("RecastMOWalk");
      m_pRecastMOWalk->begin("recastdebug", Ogre::RenderOperation::OT_TRIANGLE_LIST) ;
      for (int i = 0; i < mesh.npolys; ++i) // go through all polygons
         if (mesh.areas[i] == SAMPLE_POLYAREA_GROUND)
         {
            const unsigned short* p = &mesh.polys[i*nvp*2];

            unsigned short vi[3];
            for (int j = 2; j < nvp; ++j) // go through all verts in the polygon
            {
               if (p[j] == RC_MESH_NULL_IDX) break;
               vi[0] = p[0];
               vi[1] = p[j-1];
               vi[2] = p[j];
               for (int k = 0; k < 3; ++k) // create a 3-vert triangle for each 3 verts in the polygon.
               {
                  const unsigned short* v = &mesh.verts[vi[k]*3];
                  const float x = orig[0] + v[0]*cs;
                  const float y = orig[1] + (v[1]/*+1*/)*ch;
                  const float z = orig[2] + v[2]*cs;

                  m_pRecastMOWalk->position(x, y+m_navMeshOffsetFromGround, z);
                  if(colorRegions) {
                      m_pRecastMOWalk->colour(regionColors[mesh.regs[i]]);  // Assign vertex color
                  } else {
                      if (mesh.areas[i] == SAMPLE_POLYAREA_GROUND)
                         m_pRecastMOWalk->colour(m_navmeshGroundPolygonCol);
                      else
                         m_pRecastMOWalk->colour(m_navmeshOtherPolygonCol);
                  }

               }
               m_pRecastMOWalk->triangle(nIndex, nIndex+1, nIndex+2) ;
               nIndex+=3 ;
            }
         }
      m_pRecastMOWalk->end() ;
      m_pRecastSN->attachObject(m_pRecastMOWalk) ;



      // Define manualObject with the navmesh edges between neighbouring polygons
      m_pRecastMONeighbour = m_pSceneMgr->createManualObject("RecastMONeighbour");
      m_pRecastMONeighbour->begin("recastdebug", Ogre::RenderOperation::OT_LINE_LIST) ;

      for (int i = 0; i < mesh.npolys; ++i)
      {
         const unsigned short* p = &mesh.polys[i*nvp*2];
         for (int j = 0; j < nvp; ++j)
         {
            if (p[j] == RC_MESH_NULL_IDX) break;
            if (p[nvp+j] == RC_MESH_NULL_IDX) continue;
            int vi[2];
            vi[0] = p[j];
            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
               vi[1] = p[0];
            else
               vi[1] = p[j+1];
            for (int k = 0; k < 2; ++k)
            {
               const unsigned short* v = &mesh.verts[vi[k]*3];
               const float x = orig[0] + v[0]*cs;
               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
               const float z = orig[2] + v[2]*cs;
               //dd->vertex(x, y, z, coln);
               m_pRecastMONeighbour->position(x, y+m_navMeshEdgesOffsetFromGround, z) ;
               m_pRecastMONeighbour->colour(m_navmeshNeighbourEdgeCol) ;

            }
         }
      }

      m_pRecastMONeighbour->end() ;
      m_pRecastSN->attachObject(m_pRecastMONeighbour) ;
      

      // Define manualObject with navmesh outer edges (boundaries)
      m_pRecastMOBoundary = m_pSceneMgr->createManualObject("RecastMOBoundary");
      m_pRecastMOBoundary->begin("recastdebug", Ogre::RenderOperation::OT_LINE_LIST) ;

      for (int i = 0; i < mesh.npolys; ++i)
      {
         const unsigned short* p = &mesh.polys[i*nvp*2];
         for (int j = 0; j < nvp; ++j)
         {
            if (p[j] == RC_MESH_NULL_IDX) break;
            if (p[nvp+j] != RC_MESH_NULL_IDX) continue;
            int vi[2];
            vi[0] = p[j];
            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
               vi[1] = p[0];
            else
               vi[1] = p[j+1];
            for (int k = 0; k < 2; ++k)
            {
               const unsigned short* v = &mesh.verts[vi[k]*3];
               const float x = orig[0] + v[0]*cs;
               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
               const float z = orig[2] + v[2]*cs;
               //dd->vertex(x, y, z, colb);

               m_pRecastMOBoundary->position(x, y+m_navMeshEdgesOffsetFromGround, z) ;
               m_pRecastMOBoundary->colour(m_navmeshOuterEdgeCol);
            }
         }
      }

      m_pRecastMOBoundary->end() ;
      m_pRecastSN->attachObject(m_pRecastMOBoundary) ;




   }// end areacount

   if(regionColors)
       delete[] regionColors;

}

void OgreRecastDemo::CreateRecastPathLine(int nPathSlot)
{
   // Remove previously drawn line
   if(m_pRecastMOPath)
   {
      m_pRecastSN->detachObject("RecastMOPath") ;
      m_pSceneMgr->destroyManualObject(m_pRecastMOPath) ;
      m_pRecastMOPath=NULL ;
   }

   // Create new manualobject for the line
   m_pRecastMOPath = m_pSceneMgr->createManualObject("RecastMOPath");
   m_pRecastMOPath->begin("recastdebug", Ogre::RenderOperation::OT_LINE_STRIP) ;

   
   int nVertCount=m_PathStore[nPathSlot].MaxVertex ;
   for(int nVert=0 ; nVert<nVertCount ; nVert++)
   {
      m_pRecastMOPath->position(m_PathStore[nPathSlot].PosX[nVert], m_PathStore[nPathSlot].PosY[nVert]+m_pathOffsetFromGround, m_PathStore[nPathSlot].PosZ[nVert]) ;
      m_pRecastMOPath->colour(m_pathCol);   // Assign vertex color

      //sprintf(m_chBug, "Line Vert %i, %f %f %f", nVert, m_PathStore[nPathSlot].PosX[nVert], m_PathStore[nPathSlot].PosY[nVert], m_PathStore[nPathSlot].PosZ[nVert]) ;
      //m_pLog->logMessage(m_chBug);
   }


   m_pRecastMOPath->end() ;
   m_pRecastSN->attachObject(m_pRecastMOPath) ;
}






/**
  * Helpers
  **/

void OgreRecastDemo::OgreVect3ToFloatA(const Ogre::Vector3 vect, float* result)
{
    result[0] = vect[0];
    result[1] = vect[1];
    result[2] = vect[2];
};

void OgreRecastDemo::FloatAToOgreVect3(const float* vect, Ogre::Vector3 &result)
{
    result.x = vect[0];
    result.y = vect[1];
    result.z = vect[2];
}


void OgreRecastDemo::getMeshInformation(const Ogre::MeshPtr mesh,
                        size_t &vertex_count,
                        Ogre::Vector3* &vertices,
                        size_t &index_count,
                        unsigned long* &indices,
                        const Ogre::Vector3 &position,
                        const Ogre::Quaternion &orient,
                        const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;

    vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }
        // Add the indices
        index_count += submesh->indexData->indexCount;
    }

    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }

        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        if ( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
                                          static_cast<unsigned long>(offset);
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
};




void OgreRecastDemo::getManualMeshInformation(const Ogre::ManualObject *manual,
                        size_t &vertex_count,
                        Ogre::Vector3* &vertices,
                        size_t &index_count,
                        unsigned long* &indices,
                        const Ogre::Vector3 &position,
                        const Ogre::Quaternion &orient,
                        const Ogre::Vector3 &scale)
{
        std::vector<Ogre::Vector3> returnVertices;
        std::vector<unsigned long> returnIndices;
        unsigned long thisSectionStart = 0;
        for (int i=0; i<manual->getNumSections(); i++)
        {
                Ogre::ManualObject::ManualObjectSection * section = manual->getSection(i);
                Ogre::RenderOperation * renderOp = section->getRenderOperation();

                std::vector<Ogre::Vector3> pushVertices;
                //Collect the vertices
                {
                        const Ogre::VertexElement * vertexElement = renderOp->vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
                        Ogre::HardwareVertexBufferSharedPtr vertexBuffer = renderOp->vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());

                        char * verticesBuffer = (char*)vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
                        float * positionArrayHolder;

                        thisSectionStart = pushVertices.size();

                        pushVertices.reserve(renderOp->vertexData->vertexCount);

                        for (unsigned int j=0; j<renderOp->vertexData->vertexCount; j++)
                        {
                                vertexElement->baseVertexPointerToElement(verticesBuffer + j * vertexBuffer->getVertexSize(), &positionArrayHolder);
                                Ogre::Vector3 vertexPos = Ogre::Vector3(positionArrayHolder[0],
                                                                                                        positionArrayHolder[1],
                                                                                                        positionArrayHolder[2]);

                                vertexPos = (orient * (vertexPos * scale)) + position;

                                pushVertices.push_back(vertexPos);
                        }

                        vertexBuffer->unlock();
                }
                //Collect the indices
                {
                        if (renderOp->useIndexes)
                        {
                                Ogre::HardwareIndexBufferSharedPtr indexBuffer = renderOp->indexData->indexBuffer;

                                if (indexBuffer.isNull() || renderOp->operationType != Ogre::RenderOperation::OT_TRIANGLE_LIST)
                                {
                                        //No triangles here, so we just drop the collected vertices and move along to the next section.
                                        continue;
                                }
                                else
                                {
                                        returnVertices.reserve(returnVertices.size() + pushVertices.size());
                                        returnVertices.insert(returnVertices.end(), pushVertices.begin(), pushVertices.end());
                                }

                                unsigned int * pLong = (unsigned int*)indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
                                unsigned short * pShort = (unsigned short*)pLong;

                                returnIndices.reserve(returnIndices.size() + renderOp->indexData->indexCount);

                                for (int j = 0; j < renderOp->indexData->indexCount; j++)
                                {
                                        unsigned long index;
                                        //We also have got to remember that for a multi section object, each section has
                                        //different vertices, so the indices will not be correct. To correct this, we
                                        //have to add the position of the first vertex in this section to the index

                                        //(At least I think so...)
                                        if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
                                                index = (unsigned long)pLong[j] + thisSectionStart;
                                        else
                                                index = (unsigned long)pShort[j] + thisSectionStart;

                                        returnIndices.push_back(index);
                                }

                                indexBuffer->unlock();
                        }
                }
        }

        //Now we simply return the data.
        index_count = returnIndices.size();
        vertex_count = returnVertices.size();
        vertices = new Ogre::Vector3[vertex_count];
        for (unsigned long i = 0; i<vertex_count; i++)
        {
                vertices[i] = returnVertices[i];
        }
        indices = new unsigned long[index_count];
        for (unsigned long i = 0; i<index_count; i++)
        {
                indices[i] = returnIndices[i];
        }

        //All done.
        return;
}


static float frand()
{
        return (float)rand()/(float)RAND_MAX;
}

Ogre::Vector3 OgreRecastDemo::getRandomNavMeshPoint()
{
    // setup the filter
    dtQueryFilter Filter;
    Filter.setIncludeFlags(0xFFFF) ;
    Filter.setExcludeFlags(0) ;
    Filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f) ;

    float resultPoint[3];
    dtPolyRef resultPoly;
    m_navQuery->findRandomPoint(&Filter, frand, &resultPoly, resultPoint);

    return Ogre::Vector3(resultPoint[0], resultPoint[1], resultPoint[2]);
}

Ogre::String OgreRecastDemo::getPathFindErrorMsg(int errorCode)
{
    Ogre::String code = Ogre::StringConverter::toString(errorCode);
    switch(errorCode) {
        case 0:
                return code +" -- No error.";
        case -1:
                return code +" -- Couldn't find polygon nearest to start point.";
        case -2:
                return code +" -- Couldn't find polygon nearest to end point.";
        case -3:
                return code +" -- Couldn't create a path.";
        case -4:
                return code +" -- Couldn't find a path.";
        case -5:
                return code +" -- Couldn't create a straight path.";
        case -6:
                return code +" -- Couldn't find a straight path.";
        default:
                return code + " -- Unknown detour error code.";
    }
}

