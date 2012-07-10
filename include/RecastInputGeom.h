#ifndef RECASTINPUTGEOM_H
#define RECASTINPUTGEOM_H

#include <Ogre.h>
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "RecastConvexHull.h"


/**
  * One node or chunk of the chunky tri mesh.
  * Contains a 2D xz plane bounding box.
  * n is the number of tris contained in this chunk.
  * i is the starting index of the tris contained in this chunk.
  * The actual tris are in the chunkyMesh object itself, in linear
  * order per node, so that each node only needs the begin position
  * and tri count to reference its tris.
  **/
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
        inline ~rcChunkyTriMesh() { if(nodes) delete [] nodes; if(tris) delete [] tris; }

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
    /**
      * Create recast compatible inputgeom from the specified entities. The entities have to be added to the
      * scene before this call, as we need to calculate the world coordinates of the entity.
      * Vertices and faces of the specified source entities are stored in this inputGeom, individual entity
      * grouping and origin points are lost.
      **/
    InputGeom(std::vector<Ogre::Entity*> srcMeshes);

    /**
      * @see{InputGeom(std::vector<Ogre::Entity*>)}
      * The same, only for a single entity.
      **/
    InputGeom(Ogre::Entity* srcMesh);

    /**
      * Construct inputGeom from the specified entities, only geometry that falls within the specified bounds
      * is stored. Note: this is not done optimally, only bounding box intersections are used. But tile building
      * is further optimized due to the use of the chunky tri mesh structure that is built within this inputGeom.
      * Further this constructor is the same as @see{InputGeom(std::vector<Ogre::Entity*>)}
      * Make sure to adapt your tileBounds fall together with the tilecache bounds so that they cover exactly the
      * tiles you want to rebuild!! Don't call this with an arbitrary bounding box!
      * Use OgreDetourTileCache::getTileAlignedBox() instead.
      **/
    InputGeom(std::vector<Ogre::Entity*> srcMeshes, const Ogre::AxisAlignedBox &tileBounds);

    /**
      * Construct inputGeom from terrain geometry and specified entities.
      * The entity part is the same as @see{InputGeom(std::vector<Ogre::Entity*>)}
      * Terrain geometry is added from highest LOD level of terrain.
      **/
    InputGeom(Ogre::TerrainGroup *terrainGroup, std::vector<Ogre::Entity*> srcMeshes = std::vector<Ogre::Entity*>());

    /**
      * Create inputGeom from terrain and entity polys that fall within specified bounding box, for rebuilding only tiles that fall within the box.
      * For terrain, only the x-z bounding plane of the box is looked at (height is ignored), and only the necessary tris from the terrain are copied into inputGeom.
      * For entities only simple bounding box tests happen for determining whether an entity should be added in its entirety to this inputGeom or not.
      * Make sure to adapt your tileBounds fall together with the tilecache bounds so that they cover exactly the tiles you want to rebuild.
      **/
    InputGeom(const Ogre::AxisAlignedBox &tileBounds, Ogre::TerrainGroup *terrainGroup, std::vector<Ogre::Entity*> srcMeshes = std::vector<Ogre::Entity*>());

    /**
      * Output inputGeom to obj wavefront file.
      * This can be used to test your inputGeom in the original recast demo (which loads .obj files).
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

    /**
      * Retrieves the vertices stored within this inputGeom. The verts are an array of floats in which each
      * subsequent three floats are in order the x, y and z coordinates of a vert. The size of this array is
      * always a multiple of three and is exactly 3*getVertCount().
      **/
    float* getVerts(void);

    /**
      * The number of vertices stored in this inputGeom.
      **/
    int getVertCount(void);

    /**
      * Retrieves the tris stored in this inputGeom.
      * A tri is defined by a sequence of three indexes which refer to an index position in the getVerts() array.
      * Similar to getVerts, the size of this array is a multitude of 3 and is exactly 3*getTriCount().
      **/
    int* getTris(void);

    /**
      * The number of triangles stored in this inputGeom.
      **/
    int getTriCount(void);

    /**
      * Retrieve the normals calculated for this inputGeom. Note that the normals are not exact and are not meant for rendering,
      * but they are good enough for navmesh calculation. Each normal corresponds to one vertex from getVerts() with the same index.
      * The size of the normals array is 3*getVertCount().
      **/
    float* getNormals(void);

    /**
      * The axis aligned bounding box minimum of this input Geom.
      **/
    float* getMeshBoundsMin(void);

    /**
      * The axis aligned bounding box maximum of this input Geom.
      **/
    float* getMeshBoundsMax(void);

    /**
      * Determines whether this inputGeom has no geometry stored.
      * Returns true if this inputGeom has no geometry.
      **/
    bool isEmpty(void);

    /**
      * Use this to verify whether the generated geometry in this inputGeom matches the geometry in your scene.
      * Draws all inputGeom vertices as points in the scene.
      **/
    void debugMesh(Ogre::SceneManager *sceneMgr);

    /**
      * Maximum number of convex volume obstacles that can be added to this inputGeom.
      **/
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

    /**
      * Debug function for drawing a convex hull as lines in the scene.
      * Returns the manual object added to the scene.
      * It's possible to specify a custom color for the drawn lines, default is grey.
      **/
    static Ogre::ManualObject* drawConvexVolume(ConvexVolume *vol, Ogre::SceneManager* sceneMgr, Ogre::ColourValue color=Ogre::ColourValue(0.5, 0.5, 0.5));

    /**
      * Debug function for drawing a bounding box as lines in the scene.
      * Returns the manual object added to the scene.
      * It's possible to specify a custom color for the drawn lines, default is grey.
      **/
    static Ogre::ManualObject* drawBoundingBox(Ogre::AxisAlignedBox box, Ogre::SceneManager *sceneMgr, Ogre::ColourValue color=Ogre::ColourValue(0.5, 0.5, 0.5));

    /**
      * The chunky tri mesh generated for this inputGeom.
      * Chunky tri meshes are only used when building tiled navmeshes, and are not essential,
      * just more optimized.
      **/
    inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }

    /**
      * Raycast this inputGeometry.
      **/
    bool raycastMesh(float* src, float* dst, float& tmin);

    /**
      * See OgreDetourTileCache::hitTestObstacle, but here it serves for
      * finding convexVolumes.
      **/
    int hitTestConvexVolume(const float* sp, const float* sq);

    /**
      * Retrieve the convex volume obstacle with specified index from this inputGeom.
      **/
    ConvexVolume* getConvexVolume(int volIdx);


// TODO off-mesh connections not implemented yet
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
    // Not implemented
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

    /**
      * Convert ogre entities whose bounding box intersects the specified bounds
      * to inputGeom.
      **/
    void convertOgreEntities(const Ogre::AxisAlignedBox &tileBounds);

    /**
      * Recast input vertices
      **/
    float* verts;

    /**
      * Number of verts
      * The actual size of the verts array is 3*nverts
      **/
    int nverts;

    /**
      * Recast input tris
      * Tris are index references to verts array
      **/
    int* tris;

    /**
      * The number of tris
      * The actual size of the tris array is 3*ntris
      **/
    int ntris;

    /**
      * Normals calculated for verts
      * Normals are not entirely accurate but good enough for recast use.
      * Size of the normals array is 3*nverts
      **/
    float* normals;

    /**
      * Axis aligned bounding box of this inputGeom minimum.
      **/
    float* bmin;

    /**
      * Axis aligned bounding box of this inputGeom maximum.
      **/
    float* bmax;

    /**
      * Ogre entities this inputGeom was constructed from.
      **/
    std::vector<Ogre::Entity*> mSrcMeshes;

    /**
      * Reference node to which the absolute coordinates of the verts in this inputGeom was calculated.
      * Is usually the scene rootnode.
      **/
    Ogre::SceneNode *mReferenceNode;

    /**
      * Terrain pages from which this inputGeom was constructed.
      **/
    Ogre::TerrainGroup *mTerrainGroup;

    /**
      * Optimized structures that stores triangles in axis aligned boxes of uniform size
      * (tiles). Allows quick access to a part of the geometry, but requires more memory to store.
      **/
    rcChunkyTriMesh *m_chunkyMesh;


// Not implemented yet
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
