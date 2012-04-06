#ifndef __OgreRecastDemo_h_
#define __OgreRecastDemo_h_

#include "OgreRecast.h"
#include <Ogre.h>


// Detour/Recast stuff
class OgreRecastDemo
{
public:

   OgreRecastDemo(Ogre::SceneManager* sceneMgr);

   void RecastCleanup();
   void testPathFind();
   bool NavMeshBuild(Ogre::Entity* srcMesh);
   int FindPath(float* pStartPos, float* pEndPos, int nPathSlot, int nTarget) ;
   void CreateRecastPolyMesh(const struct rcPolyMesh& mesh, bool colorRegions=true);
   void CreateRecastPathLine(int nPathSlot);
   void drawNavMesh(void);

   /**
     * Convenience function for converting between Ogre::Vector3
     * and float* used by recast
    **/
   void OgreVect3ToFloatA(const Ogre::Vector3 vect, float* result);


   /**
     * Retrieve vertex data from a mesh
     * From http://www.ogre3d.org/tikiwiki/RetrieveVertexData
     *
     * This example is taken from monster's OgreODE project. The full source can be found under ogreaddons/ogreode in the Ogre SVN.
     * It has been adopted, so that it can be used separately. Just copy/paste it into your own project.
     *
     * Note that this code assumes sizeof(long) == sizeof(uint32_t), which is not true on AMD64 Linux.
    **/
   void getMeshInformation(const Ogre::MeshPtr mesh,
                           size_t &vertex_count,
                           Ogre::Vector3* &vertices,
                           size_t &index_count,
                           unsigned long* &indices,
                           const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                           const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                           const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);

   void getManualMeshInformation(const Ogre::ManualObject *manual,
                           size_t &vertex_count,
                           Ogre::Vector3* &vertices,
                           size_t &index_count,
                           unsigned long* &indices,
                           const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                           const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                           const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);

   unsigned char* m_triareas;
   rcHeightfield* m_solid;
   rcCompactHeightfield* m_chf;
   rcContourSet* m_cset;
   rcPolyMesh* m_pmesh;
   rcConfig m_cfg;   
   rcPolyMeshDetail* m_dmesh;

   class InputGeom* m_geom;
   class dtNavMesh* m_navMesh;
   class dtNavMeshQuery* m_navQuery;
   unsigned char m_navMeshDrawFlags;

   rcContext* m_ctx;

   float m_cellSize;
   float m_cellHeight;
   float m_agentHeight;
   float m_agentRadius;
   float m_agentMaxClimb;
   float m_agentMaxSlope;
   float m_regionMinSize;
   float m_regionMergeSize;
   float m_edgeMaxLen;
   float m_edgeMaxError;
   float m_vertsPerPoly;
   float m_detailSampleDist;
   float m_detailSampleMaxError;
   bool m_keepInterResults ;

   // Off-Mesh connections.  Not used yet.
   static const int MAX_OFFMESH_CONNECTIONS = 256;
   float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*3*2];
   float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
   unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
   unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
   unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
   unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
   int m_offMeshConCount;

   // helper debug drawing stuff
   int m_nAreaCount ;
   Ogre::ManualObject* m_pRecastMOWalk ;
   Ogre::ManualObject* m_pRecastMONeighbour ;
   Ogre::ManualObject* m_pRecastMOBoundary ;
   Ogre::ManualObject* m_pRecastMOPath ;
   Ogre::SceneNode*      m_pRecastSN ;

   float m_flTestStart[3] ;
   float m_flTestEnd[3] ;

   float *m_normals;
   int m_flDataX;
   int m_flDataY;

   float m_pathOffsetFromGround;
   float m_navMeshOffsetFromGround;
   float m_navMeshEdgesOffsetFromGround;

   Ogre::ColourValue m_navmeshNeighbourEdgeCol;
   Ogre::ColourValue m_navmeshOuterEdgeCol;
   Ogre::ColourValue m_navmeshGroundPolygonCol;
   Ogre::ColourValue m_navmeshOtherPolygonCol;
   Ogre::ColourValue m_pathCol;

   /**
    * Stores all created paths
   **/
   PATHDATA m_PathStore[MAX_PATHSLOT];


   Ogre::LogManager* m_pLog;
   Ogre::SceneManager* m_pSceneMgr;
};

#endif // #ifndef __OgreRecastDemo_h_
