#ifndef __OgreRecast_h_
#define __OgreRecast_h_

#include "OgreRecastDefinitions.h"
#include <Ogre.h>
#include "RecastInputGeom.h"

/**
  * Configuration parameters for recast navmesh building.
  * A lot of the descripions of the parameters are not mine but come from the very
  * useful CritterAI page (http://www.critterai.org/nmgen_config).
  * For more detail and pictures, have a look there.
  * Some other descriptions come from stevesp's doxygen API docs for recast itself
  * (http://www.stevefsp.org/projects/rcndoc/prod/structrcConfig.html).
  *
  * Some settings are derived from easier to set parameters, those are denoted by
  * the _ that follows their setter and getter. The easier to set parameters correspond
  * to the settings available in the demo that comes with the recast library.
  * You can overwrite those values by calling the setters preceeded with _.
  * Otherwise it suffices to set a value for each setter without a preceding _.
  **/
class OgreRecastConfigParams
{
public:
    /**
      * Initialize some default recast parameters
      **/
    OgreRecastConfigParams(void)
        : cellSize(0.3),
          cellHeight(0.2),
          agentMaxSlope(20),
          agentHeight(2.5),
          agentMaxClimb(1),
          agentRadius(0.5),
          edgeMaxLen(12),
          edgeMaxError(1.3),
          regionMinSize(50),
          regionMergeSize(20),
          vertsPerPoly(DT_VERTS_PER_POLYGON),   // (=6)
          detailSampleDist(6),
          detailSampleMaxError(1),
          keepInterResults(false)
    { eval(); }


///////////////////////
// GUI SETTINGS:
///////////////////////

    /*****************
      * Rasterization
     *****************/
    /**
      * @see{cellSize}
      **/
    inline void setCellSize(Ogre::Real cellSize) { this->cellSize = cellSize; eval(); }
    /**
      * @see{cellHeight}
      **/
    inline void setCellHeight(Ogre::Real cellHeight) { this->cellHeight = cellHeight; eval(); }
    /*****************
      * Agent
     *****************/
    /**
      * @see{agentHeight}
      **/
    inline void setAgentHeight(Ogre::Real agentHeight) { this->agentHeight = agentHeight; eval(); }
    /**
      * @see{agentRadius}
      **/
    inline void setAgentRadius(Ogre::Real agentRadius) { this->agentRadius = agentRadius; eval(); }
    /**
      * @see{agentMaxClimb}
      **/
    inline void setAgentMaxClimb(Ogre::Real agentMaxClimb) { this->agentMaxClimb = agentMaxClimb; eval(); }
    /**
      * @see{agentMaxSlope}
      **/
    inline void setAgentMaxSlope(Ogre::Real agentMaxSlope) { this->agentMaxSlope = agentMaxSlope; }
    /*****************
      * Region
     *****************/
    /**
      * @see{regionMinSize}
      **/
    inline void setRegionMinSize(Ogre::Real regionMinSize) { this->regionMinSize = regionMinSize; eval(); }
    /**
      * @see{regionMergeSize}
      **/
    inline void setRegionMergeSize(Ogre::Real regionMergeSize) { this->regionMergeSize = regionMergeSize; eval(); }
// TODO Add "monotone partitioning" option to call rcBuildRegionsMonotone in single navmesh building.

    /*****************
      * Polygonization
     *****************/
    /**
      * @see{edgeMaxLen}
      **/
    inline void setEdgeMaxLen(Ogre::Real edgeMaxLength) { this->edgeMaxLen = edgeMaxLength; eval(); }
    /**
      * @see{edgeMaxError}
      **/
    inline void setEdgeMaxError(Ogre::Real edgeMaxError) { this->edgeMaxError = edgeMaxError;}
    /**
      * @see{vertsPerPoly}
      **/
    inline void setVertsPerPoly(int vertsPerPoly) { this->vertsPerPoly = vertsPerPoly; }
    /*****************
      * Detail mesh
     *****************/
    /**
      * @see{detailSampleDist}
      **/
    inline void setDetailSampleDist(Ogre::Real detailSampleDist) { this->detailSampleDist = detailSampleDist; eval(); }
    /**
      * @see{detailSampleMaxError}
      **/
    inline void setDetailSampleMaxError(Ogre::Real detailSampleMaxError) { this->detailSampleMaxError = detailSampleMaxError; eval(); }

    /**
      * @see{keepInterResults}
      **/
    inline void setKeepInterResults(bool keepInterResults) { this->keepInterResults = keepInterResults; }

    /**
      * @see{_walkableHeight}
      **/
    inline void _setWalkableHeight(int walkableHeight) { this->_walkableHeight = walkableHeight; }
    /**
      * @see{_walkableClimb}
      **/
    inline void _setWalkableClimb(int walkableClimb) { this->_walkableClimb = walkableClimb; }
    /**
      * @see{_walkableRadius}
      **/
    inline void _setWalkableRadius(int walkableRadius) { this->_walkableRadius = walkableRadius; }
    /**
      * @see{_maxEdgeLen}
      **/
    inline void _setMaxEdgeLen(int maxEdgeLen) { this->_maxEdgeLen = maxEdgeLen; }
    /**
      * @see{_minRegionArea}
      **/
    inline void _setMinRegionArea(int minRegionArea) { this->_minRegionArea = minRegionArea; }
    /**
      * @see{_mergeRegionArea}
      **/
    inline void _setMergeRegionArea(int mergeRegionArea) { this->_mergeRegionArea = mergeRegionArea; }
    /**
      * @see{_detailSampleDist}
      **/
    inline void _setDetailSampleDist(Ogre::Real detailSampleDist) { this->_detailSampleDist = detailSampleDist; }
    /**
      * @see{_detailSampleMaxError}
      **/
    inline void _setDetailSampleMaxError(Ogre::Real detailSampleMaxError) { this->_detailSampleMaxError = detailSampleMaxError; }



    /**
      * @see{cellSize}
      **/
    inline Ogre::Real getCellSize(void) { return cellSize; }
    /**
      * @see{cellHeight}
      **/
    inline Ogre::Real getCellHeight(void) { return cellHeight; }
    /**
      * @see{agentMaxSlope}
      **/
    inline Ogre::Real getAgentMaxSlope(void) { return agentMaxSlope; }
    /**
      * @see{agentHeight}
      **/
    inline Ogre::Real getAgentHeight(void) { return agentHeight; }
    /**
      * @see{agentMaxClimb}
      **/
    inline Ogre::Real getAgentMaxClimb(void) { return agentMaxClimb; }
    /**
      * @see{agentRadius}
      **/
    inline Ogre::Real getAgentRadius(void) { return agentRadius; }
    /**
      * @see{edgeMaxLen}
      **/
    inline Ogre::Real getEdgeMaxLen(void) { return edgeMaxLen; }
    /**
      * @see{edgeMaxError}
      **/
    inline Ogre::Real getEdgeMaxError(void) { return edgeMaxError; }
    /**
      * @see{regionMinSize}
      **/
    inline Ogre::Real getRegionMinSize(void) { return regionMinSize; }
    /**
      * @see{regionMergeSize}
      **/
    inline Ogre::Real getRegionMergeSize(void) { return regionMergeSize; }
    /**
      * @see{vertsPerPoly}
      **/
    inline int getVertsPerPoly(void) { return vertsPerPoly; }
    /**
      * @see{detailSampleDist}
      **/
    inline Ogre::Real getDetailSampleDist(void) { return detailSampleDist; }
    /**
      * @see{detailSampleMaxError}
      **/
    inline Ogre::Real getDetailSampleMaxError(void) { return detailSampleMaxError; }

    /**
      * @see{keepInterResults}
      **/
    inline bool getKeepInterResults(void) { return keepInterResults; }

    /**
      * @see{_walkableHeight}
      **/
    inline int _getWalkableheight(void) { return _walkableHeight; }
    /**
      * @see{_walkableClimb}
      **/
    inline int _getWalkableClimb(void) { return _walkableClimb; }
    /**
      * @see{_walkableRadius}
      **/
    inline int _getWalkableRadius(void) { return _walkableRadius; }
    /**
      * @see{_maxEdgeLen}
      **/
    inline int _getMaxEdgeLen(void) { return _maxEdgeLen; }
    /**
      * @see{_minRegionArea}
      **/
    inline int _getMinRegionArea(void) { return _minRegionArea; }
    /**
      * @see{_mergeRegionArea}
      **/
    inline int _getMergeRegionArea(void) { return _mergeRegionArea; }
    /**
      * @see{_detailSampleDist}
      **/
    inline int _getDetailSampleDist(void) { return (int)_detailSampleDist; }
    /**
      * @see{detailSampleMaxError}
      **/
    inline int _getDetailSampleMaxError(void) { return (int)_detailSampleMaxError; }



private:
    /**
      * Derive non-directly set parameters
      * This is the default behaviour and these parameters can be overridden using
      * _ setters.
      **/
    inline void eval(void) {
        _walkableHeight = (int)ceilf(agentHeight / cellHeight);
        _walkableClimb = (int)floorf(agentMaxClimb / cellHeight);
        _walkableRadius = (int)ceilf(agentRadius / cellSize);
        _maxEdgeLen = (int)(edgeMaxLen / cellSize);
        _minRegionArea = (int)rcSqr(regionMinSize);      // Note: area = size*size
        _mergeRegionArea = (int)rcSqr(regionMergeSize);   // Note: area = size*size
        _detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
        _detailSampleMaxError = cellHeight * detailSampleMaxError;
    }

    /**
      * Cellsize (cs) is the width and depth resolution used when sampling the source geometry.
      * The width and depth of the cell columns that make up voxel fields.
      * Cells are laid out on the width/depth plane of voxel fields. Width is associated with the x-axis of the source geometry. Depth is associated with the z-axis.
      * A lower value allows for the generated meshes to more closely match the source geometry, but at a higher processing and memory cost.
      *
      * The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu].
      * cs and ch define voxel/grid/cell size. So their values have significant side effects on all parameters defined in voxel units.
      * The minimum value for this parameter depends on the platform's floating point accuracy, with the practical minimum usually around 0.05.
      **/
    Ogre::Real cellSize;

    /**
      * Cellheight (ch) is the height resolution used when sampling the source geometry. The height of the voxels in voxel fields.
      * Height is associated with the y-axis of the source geometry.
      * A smaller value allows for the final meshes to more closely match the source geometry at a potentially higher processing cost.
      * (Unlike cellSize, using a lower value for cellHeight does not significantly increase memory use.)
      *
      * The y-axis cell size to use for fields. [Limit: > 0] [Units: wu].
      * cs and ch define voxel/grid/cell size. So their values have significant side effects on all parameters defined in voxel units.
      * The minimum value for this parameter depends on the platform's floating point accuracy, with the practical minimum usually around 0.05.
      *
      * Setting ch lower will result in more accurate detection of areas the agent can still pass under, as min walkable height is discretisized
      * in number of cells. Also walkableClimb's precision is affected by ch in the same way, along with some other parameters.
      **/
    Ogre::Real cellHeight;

    /**
      * The maximum slope that is considered traversable (in degrees).
      * [Limits: 0 <= value < 90]
      * The practical upper limit for this parameter is usually around 85 degrees.
      *
      * Also called maxTraversableSlope
      **/
    Ogre::Real agentMaxSlope;

    /**
      * The height of an agent. Defines the minimum height that
      * agents can walk under. Parts of the navmesh with lower ceilings
      * will be pruned off.
      *
      * This parameter serves at setting walkableHeight (minTraversableHeight) parameter, precision of this parameter is determined by cellHeight (ch).
      **/
    Ogre::Real agentHeight;

    /**
      * The Maximum ledge height that is considered to still be traversable.
      * This parameter serves at setting walkableClimb (maxTraversableStep) parameter, precision of this parameter is determined by cellHeight (ch).
      * [Limit: >=0]
      * Allows the mesh to flow over low lying obstructions such as curbs and up/down stairways. The value is usually set to how far up/down an agent can step.
      **/
    Ogre::Real agentMaxClimb;

    /**
      * The radius on the xz (ground) plane of the circle that describes the agent (character) size.
      * Serves at setting walkableRadius (traversableAreaBorderSize) parameter, the precision of walkableRadius is affected by cellSize (cs).
      *
      * This parameter is also used by DetourCrowd to determine the area other agents have to avoid in order not to collide with an agent.
      * The distance to erode/shrink the walkable area of the heightfield away from obstructions.
      * [Limit: >=0]
      *
      * In general, this is the closest any part of the final mesh should get to an obstruction in the source geometry. It is usually set to the maximum agent radius.
      * While a value of zero is legal, it is not recommended and can result in odd edge case issues.
      *
      **/
    Ogre::Real agentRadius;

    /**
      * The maximum allowed length for contour edges along the border of the mesh.
      * [Limit: >=0]
      * Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature.
      * Serves at setting maxEdgeLen, the precision of maxEdgeLen is affected by cellSize (cs).
      **/
    Ogre::Real edgeMaxLen;

    /**
      * The maximum distance a simplfied contour's border edges should deviate the original raw contour. (edge matching)
      * [Limit: >=0] [Units: wu]
      * The effect of this parameter only applies to the xz-plane.
      *
      * Also called maxSimplificationError or edgeMaxDeviation
      * The maximum distance the edges of meshes may deviate from the source geometry.
      * A lower value will result in mesh edges following the xz-plane geometry contour more accurately at the expense of an increased triangle count.
      * A value to zero is not recommended since it can result in a large increase in the number of polygons in the final meshes at a high processing cost.
      **/
    Ogre::Real edgeMaxError;

    /**
      * The minimum number of cells allowed to form isolated island areas (size).
      * [Limit: >=0]
      * Any regions that are smaller than this area will be marked as unwalkable. This is useful in removing useless regions that can sometimes form on geometry such as table tops, box tops, etc.
      * Serves at setting minRegionArea, which will be set to the square of this value (the regions are square, thus area=size*size)
      **/
    Ogre::Real regionMinSize;

    /**
      * Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
      * [Limit: >=0] [Units: vx]
      * Serves at setting MergeRegionArea, which will be set to the square of this value (the regions are square, thus area=size*size)
      **/
    Ogre::Real regionMergeSize;

    /**
      * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
      * [Limit: >= 3]
      * If the mesh data is to be used to construct a Detour navigation mesh, then the upper limit is limited to <= DT_VERTS_PER_POLYGON (=6).
      *
      * Also called maxVertsPerPoly
      * The maximum number of vertices per polygon for polygons generated during the voxel to polygon conversion process.
      * Higher values increase processing cost, but can also result in better formed polygons in the final meshes. A value of around 6 is generally adequate with diminishing returns for higher values.
      **/
    int vertsPerPoly;

    /**
      * Sets the sampling distance to use when generating the detail mesh.
      * (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
      *
      * Also called contourSampleDistance
      * Sets the sampling distance to use when matching the detail mesh to the surface of the original geometry.
      * Impacts how well the final detail mesh conforms to the surface contour of the original geometry. Higher values result in a detail mesh which conforms more closely to the original geometry's surface at the cost of a higher final triangle count and higher processing cost.
      * Setting this argument to less than 0.9 disables this functionality.
      **/
    Ogre::Real detailSampleDist;

    /**
      * The maximum distance the detail mesh surface should deviate from heightfield data.
      * (For height detail only.) [Limit: >=0] [Units: wu]
      *
      * Also called contourMaxDeviation
      * The maximum distance the surface of the detail mesh may deviate from the surface of the original geometry.
      * The accuracy is impacted by contourSampleDistance.
      * The value of this parameter has no meaning if contourSampleDistance is set to zero.
      * Setting the value to zero is not recommended since it can result in a large increase in the number of triangles in the final detail mesh at a high processing cost.
      * Stronly related to detailSampleDist (contourSampleDistance).
      **/
    Ogre::Real detailSampleMaxError;

    /**
      * Determines whether intermediary results are stored in OgreRecast class or whether they are removed after navmesh creation.
      **/
    bool keepInterResults;


    /**
      * Minimum height in number of (voxel) cells that the ceiling needs to be
      * for an agent to be able to walk under. Related to cellHeight (ch) and
      * agentHeight.
      *
      * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable.
      * [Limit: >= 3] [Units: vx]
      * Permits detection of overhangs in the source geometry that make the geometry below un-walkable. The value is usually set to the maximum agent height.
      *
      * Also called minTraversableHeight
      * This value should be at least two times the value of cellHeight in order to get good results.
      **/
    int _walkableHeight;

    /**
      * Maximum ledge height that is considered to still be traversable, in number of cells (height).
      * [Limit: >=0] [Units: vx].
      * Allows the mesh to flow over low lying obstructions such as curbs and up/down stairways. The value is usually set to how far up/down an agent can step.
      *
      * Also called maxTraversableStep
      * Represents the maximum ledge height that is considered to still be traversable.
      * Prevents minor deviations in height from improperly showing as obstructions. Permits detection of stair-like structures, curbs, etc.
      **/
    int _walkableClimb;

    /**
      * The distance to erode/shrink the walkable area of the heightfield away from obstructions, in cellsize units.
      * [Limit: >=0] [Units: vx]
      * In general, this is the closest any part of the final mesh should get to an obstruction in the source geometry. It is usually set to the maximum agent radius.
      * While a value of zero is legal, it is not recommended and can result in odd edge case issues.
      *
      * Also called traversableAreaBorderSize
      * Represents the closest any part of a mesh can get to an obstruction in the source geometry.
      * Usually this value is set to the maximum bounding radius of agents utilizing the meshes for navigation decisions.
      *
      * This value must be greater than the cellSize to have an effect.
      * The actual border will be larger around ledges if ledge clipping is enabled. See the clipLedges parameter for more information.
      * The actual border area will be larger if smoothingTreshold is > 0. See the smoothingThreshold parameter for more information.
      **/
    int _walkableRadius;


    /**
      * The maximum allowed length for contour edges along the border of the mesh.
      * [Limit: >=0] [Units: vx].
      * Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature.
      *
      * Also called maxEdgeLength
      * The maximum length of polygon edges that represent the border of meshes.
      * More vertices will be added to border edges if this value is exceeded for a particular edge.
      * In certain cases this will reduce the number of long thin triangles.
      * A value of zero will disable this feature.
      **/
    int _maxEdgeLen;

    /**
      * The minimum number of cells allowed to form isolated island areas.
      * [Limit: >=0] [Units: vx].
      * Any regions that are smaller than this area will be marked as unwalkable.
      * This is useful in removing useless regions that can sometimes form on geometry such as table tops, box tops, etc.
      *
      * Also called minUnconnectedRegionSize
      * The minimum region size for unconnected (island) regions.
      * The value is in voxels.
      * Regions that are not connected to any other region and are smaller than this size will be culled before mesh generation. I.e. They will no longer be considered traversable.
      **/
    int _minRegionArea;

    /**
      * Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
      * [Limit: >=0] [Units: vx]
      *
      * Also called mergeRegionSize or mergeRegionArea
      * Any regions smaller than this size will, if possible, be merged with larger regions.
      * Value is in voxels.
      * Helps reduce the number of small regions. This is especially an issue in diagonal path regions where inherent faults in the region generation algorithm can result in unnecessarily small regions.
      * Small regions are left unchanged if they cannot be legally merged with a neighbor region. (E.g. Merging will result in a non-simple polygon.)
      **/
    int _mergeRegionArea;

    /**
      * Sets the sampling distance to use when generating the detail mesh.
      * (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
      *
      * Also called contourSampleDistance
      * Sets the sampling distance to use when matching the detail mesh to the surface of the original geometry.
      * Impacts how well the final detail mesh conforms to the surface contour of the original geometry. Higher values result in a
      * detail mesh which conforms more closely to the original geometry's surface at the cost of a higher final triangle count and higher processing cost.
      * Setting this argument to less than 0.9 disables this functionality.
      *
      * The difference between this parameter and edge matching (edgeMaxError) is that this parameter operates on the height rather than the xz-plane.
      * It also matches the entire detail mesh surface to the contour of the original geometry. Edge matching only matches edges of meshes to the contour of the original geometry.
      **/
    Ogre::Real _detailSampleDist;

    /**
      * The maximum distance the detail mesh surface should deviate from heightfield data.
      * (For height detail only.) [Limit: >=0] [Units: wu]
      *
      * Also called contourMaxDeviation
      * The maximum distance the surface of the detail mesh may deviate from the surface of the original geometry.
      * The accuracy is impacted by contourSampleDistance (detailSampleDist).
      * The value of this parameter has no meaning if contourSampleDistance is set to zero.
      * Setting the value to zero is not recommended since it can result in a large increase in the number of triangles in the final detail mesh at a high processing cost.
      * This parameter has no impact if contourSampleDistance is set to zero.
      **/
    Ogre::Real _detailSampleMaxError;
};


// Advance declarations, needed for expressing friendship relation
class OgreDetourTileCache;
class OgreDetourCrowd;



/**
  * This class serves as a wrapper between Ogre and Recast/Detour
  * It's not a full wrapper, but instead offers the main features needed
  * to integrate the Ogre demo with Recast and Detour.
  **/
class OgreRecast
{
public:
    /**
      * Use static geometry for debug drawing the navmesh.
      * This is only useful when drawing tiled navmeshes (detourTileCache) and when there are
      * a lot of tiles, otherwise this will result in a too high batchcount.
      **/
    static bool STATIC_GEOM_DEBUG;

    /**
      * Set to true to print verbose messages about debug drawing.
      **/
    static bool VERBOSE;

    /**
      * Initialize an OgreRecast module to generate single navmeshes or store tiled navmeshes generated by OgreDetourTileCache.
      * You can specify custom parameters for navmesh building (which are also used by OgreDetourTileCache and some by DetourCrowd).
      * Not specifying parameters will result in defaults being used.
      **/
    OgreRecast(Ogre::SceneManager* sceneMgr, OgreRecastConfigParams configParams = OgreRecastConfigParams());

    /**
      * Should be called every frame. Used for updating navmesh debug drawing when using static geometry
      * (STATIC_GEOM_DEBUG).
      **/
    void update(void);

    /**
      * The agent radius for which this navmesh is built.
      **/
    float getAgentRadius(void);

    /**
      * The agent height for which this navmesh is built.
      **/
    float getAgentHeight(void);

    /**
      * The amount with which the drawn debug path is offset from the ground
      **/
    float getPathOffsetFromGround(void);

    /**
      * The amount with which the drawn debug navmesh polys are offset from the ground.
      **/
    float getNavmeshOffsetFromGround(void);

   /**
     * Cleanup recast parameters and variables.
     * This does not clean up the objects related to debug drawing.
     **/
   void RecastCleanup();

   /**
     * Configure navbuild parameters for this module.
     * Sets m_cfg and other parameters.
     **/
   void configure(OgreRecastConfigParams params);

   /**
     * Build a navigation mesh from the specified list of Ogre::Entities as source.
     * It is required that all supplied entities are attached to a scenenode in the scene
     * before calling this method.
     *
     * Recast will construct a navmesh using some configuration parameters, which are currently
     * just set inside this method, but should be extracted to somewhere else in the future.
     * The most important parameters to set are cellsize, agentHeight and agentRadius.
     **/
   bool NavMeshBuild(std::vector<Ogre::Entity*> srcMeshesA);

   /**
     * Build a navmesh from the specified input geometry.
     * @see{OgreRecast::NavMeshBuild(std::vector<Ogre::Entity*>)}
     **/
   bool NavMeshBuild(InputGeom* input);

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
     * Find a path between start and end position, works with Ogre::Vector3 points.
     * @see{OgreRecast::FindPath(float*, float*, int, int)}
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
   void CreateRecastPolyMesh(const Ogre::String name, const unsigned short *verts, const int nverts,
                             const unsigned short *polys, const int npolys, const unsigned char *areas,
                             const int maxpolys, const unsigned short *regions, const int nvp,
                             const float cs, const float ch, const float *orig, bool colorRegions=true);


   /**
     * Remove debug drawn navmesh for navmesh tile with specified (compressed detourtile) reference.
     **/
   void removeDrawnNavmesh(unsigned int tileRef);

   /**
     * Create an Ogre::ManualObject mesh to visually debug a path on the navmesh found
     * using detour. The path stored in the specified slot number is visualized, and the
     * result is stored under the m_pRecastMOPath member variable and has the name "RecastMOPath".
     **/
   void CreateRecastPathLine(int nPathSlot);

   /**
     * Find a point on the navmesh closest to the specified point position, within predefined
     * bounds.
     * Returns true if such a point is found (returned as resultPt), returns false
     * if no point is found. When false is returned, resultPt is not altered.
     **/
   bool findNearestPointOnNavmesh(Ogre::Vector3 position, Ogre::Vector3 &resultPt);

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
     * The configuration of the recast navmesh.
     **/
   rcConfig getConfig(void);



   // helper debug drawing stuff
   Ogre::ManualObject* m_pRecastMOWalk ;
   Ogre::ManualObject* m_pRecastMONeighbour ;
   Ogre::ManualObject* m_pRecastMOBoundary ;
   Ogre::ManualObject* m_pRecastMOPath ;
   Ogre::SceneNode*      m_pRecastSN ;

   Ogre::SceneManager* m_pSceneMgr;


protected:
   /**
     * Draw the specified recast poly mesh to scene for debugging.
     **/
   void drawPolyMesh(const struct rcPolyMesh &mesh, bool colorRegions=true);


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
   int m_vertsPerPoly;
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
   Ogre::StaticGeometry *m_sg;
   bool m_rebuildSg;

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

private:
   /**
     * Retrieve the vertices from a manual object, even if they are not referenced by faces.
     * Does not retrieve faces, as it is intended to retrieve line drawings.
     **/
   std::vector<Ogre::Vector3> getManualObjectVertices(Ogre::ManualObject *man);



   // Friend OgreDetourTileCache so it can access the navmesh of this component
   friend class OgreDetourTileCache;

   // Friend OgreDetourCrowd so it can access the navmesh of this component
   friend class OgreDetourCrowd;
};

#endif // #ifndef __OgreRecast_h_
