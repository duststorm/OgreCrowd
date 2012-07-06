#ifndef RECASTINPUTGEOM_H
#define RECASTINPUTGEOM_H

#include <Ogre.h>
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "RecastConvexHull.h"


struct rcChunkyTriMeshNode
{
        float bmin[2], bmax[2];
        int i, n;
};

/**
  * Spatial subdivision structure that structures triangles
  * in axis-aligned boxes of a fixed size.
  * This allows to quickly retrieve the triangles in a specific box,
  * at the cost of a small pre-process step and extra memory usage.
  **/
struct rcChunkyTriMesh
{
        inline rcChunkyTriMesh() : nodes(0), tris(0) {};
        inline ~rcChunkyTriMesh() { delete [] nodes; delete [] tris; }

        rcChunkyTriMeshNode* nodes;
        int nnodes;
        int* tris;
        int ntris;
        int maxTrisPerChunk;
};


/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
                                                   int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

/// Returns the chunk indices which overlap the input segment.
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, float p[2], float q[2], int* ids, const int maxIds);





/**
  * Helper class to manage input geometry used as input for recast
  * to build a navmesh.
  * Adds extra features such as creating off-mesh connections between
  * points in the geometry, raycasting to polygon level and bounding box
  * intersection tests.
  * It also allows to add or remove temporary obstacles to the geometry and
  * add convex shapes as extra obstacles.
  *
  * This class handles the conversion of Ogre::Entities to recast compatible
  * input format.
  **/
class InputGeom
{
public:
    InputGeom(std::vector<Ogre::Entity*> srcMeshes);
    InputGeom(Ogre::Entity* srcMesh);

    InputGeom(std::vector<Ogre::Entity*> srcMeshes, const Ogre::AxisAlignedBox &tileBounds);

    InputGeom(Ogre::TerrainGroup *terrainGroup, std::vector<Ogre::Entity*> srcMeshes = std::vector<Ogre::Entity*>());

    /**
      * Output inputGeom to obj wavefront file.
      **/
    void writeObj(Ogre::String filename);

    /**
      * Retrieve a bounding box for the entire inputGeom, in world space
      * coordinates.
      **/
    Ogre::AxisAlignedBox getBoundingBox(void);

    /**
      * Convenience funtion to calculate the bounding box of an entity in
      * world coordinates.
      * Entity needs to be added to the scene before calling this function.
      **/
    static Ogre::AxisAlignedBox getWorldSpaceBoundingBox(Ogre::MovableObject *ent);

    /**
      * Apply the specified rotation to all vertices of this geometry.
      * Note that this does not affect the entities from which the inputGeom was
      * originally built!
      * Rotation will happen around specified pivot point (because inputGeom
      * is in world-space coordinates and has no explicit reference to its
      * origin or center point). Default value for pivot point is the world origin.
      * Usually you specify the world position of the entity that this inputGeom was
      * built from as pivot point (or the center of the bounding box in case of multiple
      * entities).
      *
      * In this implementation, bounding boxes are not recalculated after a rotation (
      * this is a little more difficult to do efficiently than it might seem).
      * You cannot continuously rotate the same AABB because it will keep growing in size.
      * You can either calculate a bounding box that fits the model in all possible rotations,
      * or recalculate a bounding box from all verts (inefficient), or use a bounding box or
      * convex hull from the physics engine to speed it up.
      **/
    void applyOrientation(Ogre::Quaternion orientation, Ogre::Vector3 pivot = Ogre::Vector3::ZERO);

    /**
      * Move all vertices of this inputGeom with the offset specified by the
      * translation vector.
      **/
    void move(Ogre::Vector3 translate);

    ~InputGeom();

    float* getVerts(void);
    int getVertCount(void);
    int* getTris(void);
    int getTriCount(void);
    float* getNormals(void);

    float* getMeshBoundsMin(void);
    float* getMeshBoundsMax(void);

    bool isEmpty(void);

    void debugMesh(Ogre::SceneManager *sceneMgr);

    static const int MAX_VOLUMES = 256;


    /**
      * Retrieve vertex data from a mesh
      * From http://www.ogre3d.org/tikiwiki/RetrieveVertexData
      *
      * This example is taken from monster's OgreODE project. The full source can be found under ogreaddons/ogreode in the Ogre SVN.
      * It has been adopted, so that it can be used separately. Just copy/paste it into your own project.
      *
      * Note that this code assumes sizeof(long) == sizeof(uint32_t), which is not true on AMD64 Linux.
     **/
    static void getMeshInformation(const Ogre::MeshPtr mesh,
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
    static void getManualMeshInformation(const Ogre::ManualObject *manual,
                            size_t &vertex_count,
                            Ogre::Vector3* &vertices,
                            size_t &index_count,
                            unsigned long* &indices,
                            const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                            const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                            const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);

    static Ogre::ManualObject* drawConvexVolume(ConvexVolume *vol, Ogre::SceneManager* sceneMgr, Ogre::ColourValue color=Ogre::ColourValue(0.5, 0.5, 0.5));

    inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }

    bool raycastMesh(float* src, float* dst, float& tmin);

    /**
      * See OgreDetourTileCache::hitTestObstacle, but here it serves for
      * finding convexVolumes.
      **/
    int hitTestConvexVolume(const float* sp, const float* sq);

    ConvexVolume* getConvexVolume(int volIdx);


    /// @name Off-Mesh connections.
    ///@{
    int getOffMeshConnectionCount() const { return m_offMeshConCount; }
    const float* getOffMeshConnectionVerts() const { return m_offMeshConVerts; }
    const float* getOffMeshConnectionRads() const { return m_offMeshConRads; }
    const unsigned char* getOffMeshConnectionDirs() const { return m_offMeshConDirs; }
    const unsigned char* getOffMeshConnectionAreas() const { return m_offMeshConAreas; }
    const unsigned short* getOffMeshConnectionFlags() const { return m_offMeshConFlags; }
    const unsigned int* getOffMeshConnectionId() const { return m_offMeshConId; }
    void addOffMeshConnection(const float* spos, const float* epos, const float rad,
                              unsigned char bidir, unsigned char area, unsigned short flags);
    void deleteOffMeshConnection(int i);
    ///@}

    /// @name Box Volumes.
    ///@{
    int getConvexVolumeCount() const { return m_volumeCount; }
    const ConvexVolume* const* getConvexVolumes() const { return m_volumes; }
    int addConvexVolume(ConvexVolume *vol);
    bool deleteConvexVolume(int i, ConvexVolume** = NULL);
    void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
    int getConvexVolumeId(ConvexVolume *convexHull);
    ///@}


    /**
      * Create a convex hull that fits around the points
      * in this geometry.
      * This type of convex hull is in fact a 2D shape on the
      * xz plane (on the ground) and with a certain height, and
      * at a certain distance above the ground (min 0, which means
      * on the ground).
      *
      * Offset determines the offset of the hull from the
      * geometry. 0 offset means the convex hull wraps
      * tightly around the mesh.
      *
      * Convex hulls can be used as temporary or dynamic obstacles
      * on a navmesh.
      * You probably only want to create convex hulls from single
      * entities or a few entities that are close together.
      **/
    ConvexVolume* getConvexHull(Ogre::Real offset = 0.0f);

private:
    /**
      * Calculate max and min bounds of this geometry.
      **/
    void calculateExtents(void);

    /**
      * Build chunky tri mesh.
      * Only needed for building tiled navmeshes.
      **/
    void buildChunkyTriMesh(void);

    /**
      * Convert triangles and verticies of ogre entities
      * to recast internal geometry format.
      **/
    void convertOgreEntities(void);

    void convertOgreEntities(Ogre::AxisAlignedBox &tileBounds);

    float* verts;
    int nverts;
    int* tris;
    int ntris;
    float* normals;

    float* bmin;
    float* bmax;

    std::vector<Ogre::Entity*> mSrcMeshes;
    Ogre::SceneNode *mReferenceNode;
    Ogre::TerrainGroup *mTerrainGroup;

    /**
      * Optimized structures that stores triangles in axis aligned boxes of uniform size
      * (tiles). Allows quick access to a part of the geometry, but requires more memory to store.
      **/
    rcChunkyTriMesh *m_chunkyMesh;


    /// @name Off-Mesh connections.
    ///@{
    static const int MAX_OFFMESH_CONNECTIONS = 256;
    float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*3*2];
    float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
    unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
    unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
    int m_offMeshConCount;
    ///@}

    /// @name Convex Volumes (temporary) added to this geometry.
    ///@{
    ConvexVolume* m_volumes[MAX_VOLUMES];
    int m_volumeCount;
    ///@}

};

#endif // RECASTINPUTGEOM_H
