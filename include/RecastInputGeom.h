#ifndef RECASTINPUTGEOM_H
#define RECASTINPUTGEOM_H

#include <Ogre.h>

static const int MAX_CONVEXVOL_PTS = 12;
struct ConvexVolume
{
        float verts[MAX_CONVEXVOL_PTS*3];
        float hmin, hmax;
        int nverts;
        int area;
};

struct rcChunkyTriMeshNode
{
        float bmin[2], bmax[2];
        int i, n;
};

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
    ~InputGeom();

    float* getVerts(void);
    int getVertCount(void);
    int* getTris(void);
    int getTriCount(void);
    float* getNormals(void);

    float* getMeshBoundsMin(void);
    float* getMeshBoundsMax(void);

    bool isEmpty(void);

    void debugMesh(void);


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

    inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }

    bool raycastMesh(float* src, float* dst, float& tmin);


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
    const ConvexVolume* getConvexVolumes() const { return m_volumes; }
    void addConvexVolume(const float* verts, const int nverts,
                         const float minh, const float maxh, unsigned char area);
    void deleteConvexVolume(int i);
    void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
    ///@}

private:
    /**
      * Calculate max and min bounds of this geometry.
      **/
    void calculateExtents(void);

    /**
      * Convert triangles and verticies of ogre entities
      * to recast internal geometry format.
      **/
    void convertOgreEntities(void);

    float* verts;
    int nverts;
    int* tris;
    int ntris;
    float* normals;

    float* bmin;
    float* bmax;

    std::vector<Ogre::Entity*> mSrcMeshes;
    Ogre::SceneNode *mReferenceNode;

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

    /// @name Convex Volumes.
    ///@{
    static const int MAX_VOLUMES = 256;
    ConvexVolume m_volumes[MAX_VOLUMES];
    int m_volumeCount;
    ///@}
};

#endif // RECASTINPUTGEOM_H
