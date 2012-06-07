#ifndef RECASTINPUTGEOM_H
#define RECASTINPUTGEOM_H

#include <Ogre.h>

class InputGeom
{
public:
    InputGeom(std::vector<Ogre::Entity*> srcMeshes);

    float* getVerts(void);
    int getVertCount(void);
    int* getTris(void);
    int getTriCount(void);
    float* getNormals(void);

    float* getMeshBoundsMin(void);
    float* getMeshBoundsMax(void);

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

private:
    /**
      * Calculate max and min bounds of this geometry.
      **/
    void calculateExtents(void);

    float* verts;
    int nverts;
    int* tris;
    int ntris;
    float* normals;

    float* bmin;
    float* bmax;

    std::vector<Ogre::Entity*> mSrcMeshes;
    Ogre::SceneNode *mReferenceNode;
};

#endif // RECASTINPUTGEOM_H
