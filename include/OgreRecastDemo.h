#ifndef __OgreRecastDemo_h_
#define __OgreRecastDemo_h_

#include "OgreRecast.h"
#include <Ogre.h>


/**
  * This class serves as a wrapper between Ogre and Recast/Detour
  * It's not a full wrapper, but instead offers the main features needed
  * to integrate the Ogre demo with Recast and Detour.
  **/
class OgreRecastDemo
{
public:

   OgreRecastDemo(Ogre::SceneManager* sceneMgr);

   /**
     * Cleanup recast parameters and variables.
     * This does not clean up the objects related to debug drawing.
     **/
   void RecastCleanup();

   /**
     * Build a navigation mesh from the specified Ogre::Mesh as source.
     * Currently only one mesh is supported, which means your level map must consist of only
     * one mesh.
     * Recast will construct a navmesh using some configuration parameters, which are currently
     * just set inside this method, but should be extracted to somewhere else in the future.
     * The most important parameters to set are cellsize, agentHeight and agentRadius.
     **/
   bool NavMeshBuild(Ogre::Entity* srcMesh);

   /**
    * Find a path beween start point and end point and, if possible, generates a list of lines in a path.
    * It might fail if the start or end points aren't near any navmesh polygons, or if the path is too long,
    * or it can't make a path, or various other reasons.
    *
    * nPathSlot: The index number for the slot in which the found path is to be stored.
    * nTarget: Number identifying the target the path leads to. Recast does nothing with this, but you can give them
    *   meaning in your own application.
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
   int FindPath(float* pStartPos, float* pEndPos, int nPathSlot, int nTarget);

   /**
     * Same as above, but works with Ogre::Vector3 points.
     **/
   int FindPath(Ogre::Vector3 startPos, Ogre::Vector3 endPos, int nPathSlot, int nTarget);

   /**
     * Draw the nav mesh for debug purposes. The navmesh is converted to an Ogre::Mesh and
     * put inside the scene for rendering.
     **/
   void drawNavMesh(void);

   /**
     * Calculate visual Ogre meshes to visualize the recast navigation mesh for debugging.
     *
     * Convert the calculated navmesh into an Ogre::ManualObject mesh, and put it in the scene.
     * A scenenode with name "RecastSN" will be created in the root of the scenegraph. This
     * scenenode is referenced by the m_pRecastSN member variable.
     *
     * Within this scenenode a mesh with name "RecastMOWalk" will be added, which is stored in
     * member variable m_pRecastMOWalk. This mesh will represents the faces of the segments in
     * the navmesh. Not the lines indicating the edges of the navmesh.
     *
     * The manual object referenced by member variable m_pRecastMONeighbour and with name
     * "RecastMONeighbour" is also added to the same scenenode. This object contains the lines
     * between neighbouring segments of the navmesh. These are all except the outer edges of
     * the navmesh.
     *
     * Finally, the manual object referenced by member variable m_pRecastMOBoundary and with name
     * "RecastMOBoundary" is added to the scene node. It is a collection of lines that represent
     * the outer edges of the navmesh, being the ones that do not have any neighbouring segments.
     **/
   void CreateRecastPolyMesh(const struct rcPolyMesh& mesh, bool colorRegions=true);

   /**
     * Create an Ogre::ManualObject mesh to visually debug a path on the navmesh found
     * using detour. The path stored in the specified slot number is visualized, and the
     * result is stored under the m_pRecastMOPath member variable and has the name "RecastMOPath".
     **/
   void CreateRecastPathLine(int nPathSlot);

   /**
     * Returns a random point on the navmesh.
     **/
   Ogre::Vector3 getRandomNavMeshPoint();

   /**
     * Convenience function for converting between Ogre::Vector3
     * and float* used by recast.
    **/
   static void OgreVect3ToFloatA(const Ogre::Vector3 vect, float* result);

   /**
     * Convenience function for converting between float* used by recast
     * and Ogre::Vector3.
    **/
   static void FloatAToOgreVect3(const float* vect, Ogre::Vector3 &result);

   /**
     * Translate error code of detour findPath into a readable explanation.
     **/
   Ogre::String getPathFindErrorMsg(int errorCode);


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

   /**
     * getMeshInformation for manual meshes.
     **/
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

   /**
     * Offset that the debug path is drawn from the ground.
     **/
   float m_pathOffsetFromGround;

   /**
     * Offset that the debug navmesh is drawn from the ground.
     **/
   float m_navMeshOffsetFromGround;

   /**
     * Offset that the navmesh edges are drawn from the ground.
     **/
   float m_navMeshEdgesOffsetFromGround;

   /**
     * Colours used for the various debug drawing objects.
     **/
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
