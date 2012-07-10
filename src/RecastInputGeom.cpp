#include "RecastInputGeom.h"
#include "OgreRecast.h"
#include <OgreStreamSerialiser.h>
#include <float.h>
#include <cstdio>

InputGeom::InputGeom(std::vector<Ogre::Entity*> srcMeshes)
    : mSrcMeshes(srcMeshes),
      mTerrainGroup(0),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      m_offMeshConCount(0),
      m_volumeCount(0),
      m_chunkyMesh(0),
      normals(0),
      verts(0),
      tris(0)
{
    if (srcMeshes.empty())
        return;


    // Convert Ogre::Entity source meshes to a format that recast understands

    //set the reference node
    Ogre::Entity* ent = srcMeshes[0];
    mReferenceNode = ent->getParentSceneNode()->getCreator()->getRootSceneNode();


    // Set the area where the navigation mesh will be build.
    // Using bounding box of source mesh and specified cell size
    calculateExtents();


    // Convert ogre geometry (vertices, triangles and normals)
    convertOgreEntities();


// TODO You don't need to build this in single navmesh mode
    buildChunkyTriMesh();
}

void InputGeom::buildChunkyTriMesh()
{
    m_chunkyMesh = new rcChunkyTriMesh;
    if (!m_chunkyMesh)
    {
        Ogre::LogManager::getSingletonPtr()->logMessage("buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
        return;
    }
    if (!rcCreateChunkyTriMesh(getVerts(), getTris(), getTriCount(), 256, m_chunkyMesh))
    {
        Ogre::LogManager::getSingletonPtr()->logMessage("buildTiledNavigation: Failed to build chunky mesh.");
        return;
    }
}

InputGeom::InputGeom(Ogre::Entity* srcMesh)
    : mTerrainGroup(0),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      m_offMeshConCount(0),
      m_volumeCount(0),
      m_chunkyMesh(0)
{
    if (! srcMesh)
        return;

    mSrcMeshes.push_back(srcMesh);


    // Convert Ogre::Entity source meshes to a format that recast understands

    //set the reference node
    mReferenceNode = srcMesh->getParentSceneNode()->getCreator()->getRootSceneNode();


    // Set the area where the navigation mesh will be build.
    // Using bounding box of source mesh and specified cell size
    calculateExtents();


    // Convert ogre geometry (vertices, triangles and normals)
    convertOgreEntities();


// TODO You don't need to build this in single navmesh mode
    buildChunkyTriMesh();
}


// TODO make sure I don't forget destructing some members
InputGeom::~InputGeom()
{
    if(m_chunkyMesh)
        delete m_chunkyMesh;

    if(verts)
        delete[] verts;
    if(normals)
        delete[] normals;
    if(tris)
        delete[] tris;
    if(bmin)
        delete[] bmin;
    if(bmax)
        delete[] bmax;
}

// Used to query scene to get input geometry of a tile directly
// TODO only bounds segmentation that happens at the moment is on entity bounding box, improve this?
// Tile bounds need to be in world space coordinates
InputGeom::InputGeom(std::vector<Ogre::Entity*> srcMeshes, const Ogre::AxisAlignedBox &tileBounds)
    : mSrcMeshes(srcMeshes),
      mTerrainGroup(0),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      m_offMeshConCount(0),
      m_volumeCount(0),
      m_chunkyMesh(0),
      normals(0),
      verts(0),
      tris(0)
{
    if (srcMeshes.empty())
        return;


    // Convert Ogre::Entity source meshes to a format that recast understands

    //set the reference node
    Ogre::Entity* ent = srcMeshes[0];
    mReferenceNode = ent->getParentSceneNode()->getCreator()->getRootSceneNode();


    // Set the area where the navigation mesh will be build to the tileBounds parameter.
    bmin = new float[3];
    bmax = new float[3];
    OgreRecast::OgreVect3ToFloatA(tileBounds.getMinimum(), bmin);
    OgreRecast::OgreVect3ToFloatA(tileBounds.getMaximum(), bmax);


    // Convert ogre geometry (vertices, triangles and normals)
    convertOgreEntities(tileBounds);


// TODO You don't need to build this in single navmesh mode
    buildChunkyTriMesh();
}

InputGeom::InputGeom(Ogre::TerrainGroup *terrainGroup, std::vector<Ogre::Entity*> srcMeshes)
    : mSrcMeshes(srcMeshes),
      mTerrainGroup(0),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      m_offMeshConCount(0),
      m_volumeCount(0),
      m_chunkyMesh(0),
      normals(0),
      verts(0),
      tris(0)
{
    // PARTS OF THE FOLLOWING CODE WERE TAKEN AND MODIFIED FROM AN OGRE3D FORUM POST
    const int numNodes = srcMeshes.size();

    // Calculate bounds around all entities
    if(mSrcMeshes.size() > 0) {
        // Set reference node
        mReferenceNode = mSrcMeshes[0]->getParentSceneNode()->getCreator()->getRootSceneNode();

        // Calculate entity bounds
        // Set the area where the navigation mesh will be build.
        // Using bounding box of source mesh and specified cell size
        calculateExtents();
    } else {
        bmin = new float[3]; bmin[0] = FLT_MAX; bmin[1] = FLT_MAX; bmin[2] = FLT_MAX;
        bmax = new float[3]; bmax[0] = FLT_MIN; bmax[1] = FLT_MIN; bmax[2] = FLT_MIN;
    }

    // Calculate terrain bounds
    Ogre::TerrainGroup::TerrainIterator ti = terrainGroup->getTerrainIterator();
    Ogre::Terrain* trn;
    size_t trnCount = 0;
    while(ti.hasMoreElements())
    {
         trn = ti.getNext()->instance;
         Ogre::AxisAlignedBox bb = trn->getWorldAABB();
         Ogre::Vector3 min = bb.getMinimum();
         if(min.x < bmin[0])
             bmin[0]= min.x;
         if(min.y < bmin[1])
             bmin[1]= min.y;
         if(min.z < bmin[2])
             bmin[2]= min.z;

         Ogre::Vector3 max = bb.getMaximum();
         if(max.x > bmax[0])
             bmax[0]= max.x;
         if(max.y > bmax[1])
             bmax[1]= max.y;
         if(max.z > bmax[2])
             bmax[2]= max.z;

         trnCount++;
    }

    //if (trnCount == 0)
        // TODO return with error?

	size_t pagesTotal = trnCount;
    const size_t totalMeshes = pagesTotal + numNodes;

    nverts = 0;
    ntris = 0;
    size_t *meshVertexCount = new size_t[totalMeshes];
    size_t *meshIndexCount = new size_t[totalMeshes];
    Ogre::Vector3 **meshVertices = new Ogre::Vector3*[totalMeshes];
    unsigned long **meshIndices = new unsigned long*[totalMeshes];



    //---------------------------------------------------------------------------------
    // TERRAIN DATA BUILDING
    ti = terrainGroup->getTerrainIterator();
    trnCount = 0;
    while(ti.hasMoreElements())
    {
         trn = ti.getNext()->instance;

         // get height data, world size, map size
         float *mapptr = trn->getHeightData();
         float WorldSize = trn->getWorldSize();
         int MapSize = trn->getSize();
         // calculate where we need to move/place our vertices
         float DeltaPos = (WorldSize / 2.0f);

         // Determine world offset position for this terrain tile
         Ogre::AxisAlignedBox tileBox = trn->getWorldAABB();
         float DeltaX = tileBox.getMinimum().x;
         float DeltaZ = tileBox.getMaximum().z;

         float Scale = WorldSize / (float)(MapSize - 1);

         //////////////////////////////
         // THIS CODE WAS TAKEN FROM
         // AN OGRE FORUMS THREAD IN THE
         // NEW TERRAIN SCREENSHOTS THREAD
         // IN THE SHOWCASE FORUMS - I ONLY MODIFIED IT
         // TO BE ABLE TO WORK FOR RECAST AND IN THE CONTEXT OF
         // THIS DEMO APPLICATION

         // build vertices
         meshVertices[trnCount] = new Ogre::Vector3[(MapSize*MapSize)];

         int i = 0;
         int u = 0;
         int max = MapSize; // i think i needed this for something before but now it's obviously redundant
         int z = 0;
         for(int x = 0;; ++x)
         {
             // if we've reached the right edge, start over on the next line
             if(x == max)
             {
                 x = 0;
                 ++z;
             }
             // if we reached the bottom/end, we're done
             if(z == max)
                 break;

             // Calculate world coordinates for terrain tile vertex. Terrain vertices are defined in tile-local coordinates.
             // add the vertex to the buffer
             meshVertices[trnCount][u] = Ogre::Vector3((Scale * x) + DeltaX, mapptr[(MapSize * z) + x], (Scale * -z) + DeltaZ);
// TODO calculating a lower resolution LOD from the terrain is probably done by sampling a smaller amount of height points from the mapptr array

             i += 3;
             ++u;
         }


         size_t size = ((MapSize*MapSize)-(MapSize*2)) * 6;
         meshIndices[trnCount] = new unsigned long[size];
         // i will point to the 'indices' index to insert at, x points to the vertex # to use
         i = 0;
         for(int x = 0;;++x)
         {
             // skip rightmost vertices
             if((x+1)%MapSize == 0)
             {
                 ++x;
             }

             // make a square of 2 triangles
             meshIndices[trnCount][i] = x;
             meshIndices[trnCount][i+1] = x + 1;
             meshIndices[trnCount][i+2] = x + MapSize;

             meshIndices[trnCount][i+3] = x + 1;
             meshIndices[trnCount][i+4] = x + 1 + MapSize;
             meshIndices[trnCount][i+5] = x + MapSize;

             // if we just did the final square, we're done
             if(x+1+MapSize == (MapSize*MapSize)-1)
                 break;

             i += 6;
         }

         meshVertexCount[trnCount] = trn->getSize()*trn->getSize();
         meshIndexCount[trnCount] = size;

         nverts += meshVertexCount[trnCount];
         ntris += meshIndexCount[trnCount];

         if(trnCount < pagesTotal)
             ++trnCount;
    }




    //-----------------------------------------------------------------------------------------
    // ENTITY DATA BUILDING


    int i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++)
    {
        int ind = pagesTotal + i;
        getMeshInformation((*iter)->getMesh(), meshVertexCount[ind], meshVertices[ind], meshIndexCount[ind], meshIndices[ind]);

        //total number of verts
        nverts += meshVertexCount[ind];
        //total number of indices
        ntris += meshIndexCount[ind];

        i++;
    }


    //-----------------------------------------------------------------------------------------
    // DECLARE RECAST DATA BUFFERS USING THE INFO WE GRABBED ABOVE

    verts = new float[nverts*3];// *3 as verts holds x,y,&z for each verts in the array
    tris = new int[ntris];// tris in recast is really indices like ogre

    //convert index count into tri count
    ntris = ntris/3; //although the tris array are indices the ntris is actual number of triangles, eg. indices/3;


    //-----------------------------------------------------------------------------------------
    // RECAST TERRAIN DATA BUILDING

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    int vertsIndex = 0;
    int prevVerticiesCount = 0;
    int prevIndexCountTotal = 0;

    for (size_t i = 0 ; i < pagesTotal ; ++i)
    {
        //We don't need to transform terrain verts, they are already in world space!
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[i] ; ++j)
        {
            vertexPos = meshVertices[i][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[i] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[i][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[i];
        prevVerticiesCount += meshVertexCount[i];

    }



    //-----------------------------------------------------------------------------------------
    // RECAST TERRAIN ENTITY DATA BUILDING

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    // DO NOT RESET THESE VALUES
    // we need to keep the vert/index offset we have from the terrain generation above to make sure
    // we start referencing recast's buffers from the correct place, otherwise we just end up
    // overwriting our terrain data, which really is a pain ;)

    //set the reference node
    i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++)
    {
        int ind = pagesTotal + i;
        //find the transform between the reference node and this node
        Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * (*iter)->getParentSceneNode()->_getFullTransform();
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[ind] ; j++)
        {
            vertexPos = transform * meshVertices[ind][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[ind] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[ind][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[ind];
        prevVerticiesCount += meshVertexCount[ind];

        i++;
    }


// TODO fix this (memory leak)
/*
    //delete tempory arrays
    //TODO These probably could member varibles, this would increase performance slightly
    for(size_t i = 0; i < totalMeshes; ++i)
    {
        delete [] meshVertices[i];

    }
*/
    // first 4 were created differently, without getMeshInformation();
    // throws an exception if we delete the first 4
    // TODO - FIX THIS MEMORY LEAK - its only small, but its still not good
    for(size_t i  = pagesTotal; i < totalMeshes; ++i)
    {
        delete [] meshIndices[i];
    }

    delete [] meshVertices;
    delete [] meshVertexCount;
    delete [] meshIndices;
    delete [] meshIndexCount;



    //---------------------------------------------------------------------------------------------
    // RECAST **ONLY** NORMAL CALCS ( These are not used anywhere other than internally by recast)

    normals = new float[ntris*3];
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
        float* n = &normals[i];
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


    // Build chunky tri mesh from triangles, used for tiled navmesh construction
    buildChunkyTriMesh();
}


InputGeom::InputGeom(const Ogre::AxisAlignedBox &tileBounds, Ogre::TerrainGroup *terrainGroup, std::vector<Ogre::Entity*> srcMeshes)
    : mSrcMeshes(srcMeshes),
      mTerrainGroup(0),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      m_offMeshConCount(0),
      m_volumeCount(0),
      m_chunkyMesh(0),
      normals(0),
      verts(0),
      tris(0)
{
    // PARTS OF THE FOLLOWING CODE WERE TAKEN AND MODIFIED FROM AN OGRE3D FORUM POST

    // Set reference node for entities
    if(mSrcMeshes.size() > 0) {
        Ogre::Entity* ent = mSrcMeshes[0];
        mReferenceNode = ent->getParentSceneNode()->getCreator()->getRootSceneNode();
    }

    // Set the area where the navigation mesh will be build to the tileBounds parameter.
    bmin = new float[3];
    bmax = new float[3];
    OgreRecast::OgreVect3ToFloatA(tileBounds.getMinimum(), bmin);
    OgreRecast::OgreVect3ToFloatA(tileBounds.getMaximum(), bmax);

    // Calculate intersecting terrain pages
    Ogre::TerrainGroup::TerrainIterator ti = terrainGroup->getTerrainIterator();
    Ogre::Terrain* trn;
    std::vector<Ogre::Terrain*> terrainPages;
    while(ti.hasMoreElements())
    {
         trn = ti.getNext()->instance;
         Ogre::AxisAlignedBox bb = trn->getWorldAABB();

         if(tileBounds.intersects(bb))
             terrainPages.push_back(trn);
    }

    // Get intersecting entities
    std::vector<Ogre::Entity*> selectedEntities;
    Ogre::AxisAlignedBox bb;
    Ogre::Matrix4 transform;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        transform = mReferenceNode->_getFullTransform().inverse() * (*iter)->getParentSceneNode()->_getFullTransform();
        bb = (*iter)->getBoundingBox();
        bb.transform(transform);    // Transform to world coordinates
        if( bb.intersects(tileBounds) )
            selectedEntities.push_back(*iter);
    }
    mSrcMeshes.clear();
    mSrcMeshes = selectedEntities;

    //if (trnCount == 0)
        // TODO return with error?

    size_t numNodes = mSrcMeshes.size();
    size_t pagesTotal = terrainPages.size();
    const size_t totalMeshes = pagesTotal + numNodes;

    nverts = 0;
    ntris = 0;
    size_t *meshVertexCount = new size_t[totalMeshes];
    size_t *meshIndexCount = new size_t[totalMeshes];
    Ogre::Vector3 **meshVertices = new Ogre::Vector3*[totalMeshes];
    unsigned long **meshIndices = new unsigned long*[totalMeshes];



    //---------------------------------------------------------------------------------
    // TERRAIN DATA BUILDING
    size_t trnCount = 0;
    for(std::vector<Ogre::Terrain*>::iterator iter = terrainPages.begin(); iter != terrainPages.end(); iter++)
    {
         trn = *iter;

         // get height data, world size, map size
         float *mapptr = trn->getHeightData();
         float WorldSize = trn->getWorldSize();
         int MapSize = trn->getSize();
         // calculate where we need to move/place our vertices
         float DeltaPos = (WorldSize / 2.0f);

         // Determine world offset position for this terrain tile
         Ogre::AxisAlignedBox tileBox = trn->getWorldAABB();
         float DeltaX = tileBox.getMinimum().x;
         float DeltaZ = tileBox.getMaximum().z;

         float Scale = WorldSize / (float)(MapSize - 1);

         //////////////////////////////
         // THIS CODE WAS TAKEN FROM
         // AN OGRE FORUMS THREAD IN THE
         // NEW TERRAIN SCREENSHOTS THREAD
         // IN THE SHOWCASE FORUMS - I ONLY MODIFIED IT
         // TO BE ABLE TO WORK FOR RECAST AND IN THE CONTEXT OF
         // THIS DEMO APPLICATION

// TODO only retrieve vertices in x-z plane of tilebounds
         // build vertices
         meshVertices[trnCount] = new Ogre::Vector3[(MapSize*MapSize)];

         int i = 0;
         int u = 0;
         int max = MapSize; // i think i needed this for something before but now it's obviously redundant
         int z = 0;
         for(int x = 0;; ++x)
         {
             // if we've reached the right edge, start over on the next line
             if(x == max)
             {
                 x = 0;
                 ++z;
             }
             // if we reached the bottom/end, we're done
             if(z == max)
                 break;

             // Calculate world coordinates for terrain tile vertex. Terrain vertices are defined in tile-local coordinates.
             // add the vertex to the buffer
             meshVertices[trnCount][u] = Ogre::Vector3((Scale * x) + DeltaX, mapptr[(MapSize * z) + x], (Scale * -z) + DeltaZ);
// TODO calculating a lower resolution LOD from the terrain is probably done by sampling a smaller amount of height points from the mapptr array

             i += 3;
             ++u;
         }


         size_t size = ((MapSize*MapSize)-(MapSize*2)) * 6;
         meshIndices[trnCount] = new unsigned long[size];
         // i will point to the 'indices' index to insert at, x points to the vertex # to use
         i = 0;
         for(int x = 0;;++x)
         {
             // skip rightmost vertices
             if((x+1)%MapSize == 0)
             {
                 ++x;
             }

             // make a square of 2 triangles
             meshIndices[trnCount][i] = x;
             meshIndices[trnCount][i+1] = x + 1;
             meshIndices[trnCount][i+2] = x + MapSize;

             meshIndices[trnCount][i+3] = x + 1;
             meshIndices[trnCount][i+4] = x + 1 + MapSize;
             meshIndices[trnCount][i+5] = x + MapSize;

             // if we just did the final square, we're done
             if(x+1+MapSize == (MapSize*MapSize)-1)
                 break;

             i += 6;
         }

         meshVertexCount[trnCount] = trn->getSize()*trn->getSize();
         meshIndexCount[trnCount] = size;

         nverts += meshVertexCount[trnCount];
         ntris += meshIndexCount[trnCount];

         if(trnCount < pagesTotal)
             ++trnCount;
    }




    //-----------------------------------------------------------------------------------------
    // ENTITY DATA BUILDING


    int i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++)
    {
        int ind = pagesTotal + i;
        getMeshInformation((*iter)->getMesh(), meshVertexCount[ind], meshVertices[ind], meshIndexCount[ind], meshIndices[ind]);

        //total number of verts
        nverts += meshVertexCount[ind];
        //total number of indices
        ntris += meshIndexCount[ind];

        i++;
    }


    //-----------------------------------------------------------------------------------------
    // DECLARE RECAST DATA BUFFERS USING THE INFO WE GRABBED ABOVE

    verts = new float[nverts*3];// *3 as verts holds x,y,&z for each verts in the array
    tris = new int[ntris];// tris in recast is really indices like ogre

    //convert index count into tri count
    ntris = ntris/3; //although the tris array are indices the ntris is actual number of triangles, eg. indices/3;


    //-----------------------------------------------------------------------------------------
    // RECAST TERRAIN DATA BUILDING

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    int vertsIndex = 0;
    int prevVerticiesCount = 0;
    int prevIndexCountTotal = 0;

    for (size_t i = 0 ; i < pagesTotal ; ++i)
    {
        //We don't need to transform terrain verts, they are already in world space!
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[i] ; ++j)
        {
            vertexPos = meshVertices[i][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[i] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[i][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[i];
        prevVerticiesCount += meshVertexCount[i];

    }



    //-----------------------------------------------------------------------------------------
    // RECAST TERRAIN ENTITY DATA BUILDING

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    // DO NOT RESET THESE VALUES
    // we need to keep the vert/index offset we have from the terrain generation above to make sure
    // we start referencing recast's buffers from the correct place, otherwise we just end up
    // overwriting our terrain data, which really is a pain ;)

    //set the reference node
    i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++)
    {
        int ind = pagesTotal + i;
        //find the transform between the reference node and this node
        Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * (*iter)->getParentSceneNode()->_getFullTransform();
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[ind] ; j++)
        {
            vertexPos = transform * meshVertices[ind][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[ind] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[ind][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[ind];
        prevVerticiesCount += meshVertexCount[ind];

        i++;
    }


// TODO fix this (memory leak)
/*
    //delete tempory arrays
    //TODO These probably could member varibles, this would increase performance slightly
    for(size_t i = 0; i < totalMeshes; ++i)
    {
        delete [] meshVertices[i];

    }
*/
    // first 4 were created differently, without getMeshInformation();
    // throws an exception if we delete the first 4
    // TODO - FIX THIS MEMORY LEAK - its only small, but its still not good
    for(size_t i  = pagesTotal; i < totalMeshes; ++i)
    {
        delete [] meshIndices[i];
    }

    delete [] meshVertices;
    delete [] meshVertexCount;
    delete [] meshIndices;
    delete [] meshIndexCount;



    //---------------------------------------------------------------------------------------------
    // RECAST **ONLY** NORMAL CALCS ( These are not used anywhere other than internally by recast)

    normals = new float[ntris*3];
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
        float* n = &normals[i];
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


    // Build chunky tri mesh from triangles, used for tiled navmesh construction
    buildChunkyTriMesh();
}





void InputGeom::convertOgreEntities()
{
    //Convert all vertices and triangles to recast format
    const int numNodes = mSrcMeshes.size();
    size_t *meshVertexCount = new size_t[numNodes];
    size_t *meshIndexCount = new size_t[numNodes];
    Ogre::Vector3 **meshVertices = new Ogre::Vector3*[numNodes];
    unsigned long **meshIndices = new unsigned long*[numNodes];

    nverts = 0;
    ntris = 0;
    size_t i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        getMeshInformation((*iter)->getMesh(), meshVertexCount[i], meshVertices[i], meshIndexCount[i], meshIndices[i]);
        //total number of verts
        nverts += meshVertexCount[i];
        //total number of indices
        ntris += meshIndexCount[i];

        i++;
    }

    // DECLARE RECAST DATA BUFFERS USING THE INFO WE GRABBED ABOVE
    verts = new float[nverts*3];// *3 as verts holds x,y,&z for each verts in the array
    tris = new int[ntris];// tris in recast is really indices like ogre

    //convert index count into tri count
    ntris = ntris/3; //although the tris array are indices the ntris is actual number of triangles, eg. indices/3;

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    int vertsIndex = 0;
    int prevVerticiesCount = 0;
    int prevIndexCountTotal = 0;
    i = 0;
    for (std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        Ogre::Entity *ent = *iter;
        //find the transform between the reference node and this node
        Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[i] ; j++)
        {
            vertexPos = transform*meshVertices[i][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[i] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[i][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[i];
        prevVerticiesCount += meshVertexCount[i];

        i++;
    }

    //delete tempory arrays
    //TODO These probably could member varibles, this would increase performance slightly
    delete[] meshVertices;
    delete[] meshIndices;

    // calculate normals data for Recast - im not 100% sure where this is required
    // but it is used, Ogre handles its own Normal data for rendering, this is not related
    // to Ogre at all ( its also not correct lol )
    // TODO : fix this
    normals = new float[ntris*3];
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
        float* n = &normals[i];
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
}

void InputGeom::convertOgreEntities(const Ogre::AxisAlignedBox &tileBounds)
{
    // Select only entities that fall at least partly within tileBounds
    std::vector<Ogre::Entity*> selectedEntities;
    Ogre::AxisAlignedBox bb;
    Ogre::Matrix4 transform;
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        transform = mReferenceNode->_getFullTransform().inverse() * (*iter)->getParentSceneNode()->_getFullTransform();
        bb = (*iter)->getBoundingBox();
        bb.transform(transform);    // Transform to world coordinates
        if( bb.intersects(tileBounds) )
            selectedEntities.push_back(*iter);
    }
    mSrcMeshes.clear();
    mSrcMeshes = selectedEntities;


    convertOgreEntities();
}

void InputGeom::calculateExtents()
{
    Ogre::Entity* ent = mSrcMeshes[0];
    Ogre::AxisAlignedBox srcMeshBB = ent->getBoundingBox();
    Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();
    srcMeshBB.transform(transform);
    Ogre::Vector3 min = srcMeshBB.getMinimum();
    Ogre::Vector3 max = srcMeshBB.getMaximum();

    // Calculate min and max from all entities
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        Ogre::Entity* ent = *iter;

        //find the transform between the reference node and this node
        transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();

        Ogre::AxisAlignedBox srcMeshBB = ent->getBoundingBox();
        srcMeshBB.transform(transform);
        Ogre::Vector3 min2 = srcMeshBB.getMinimum();
        if(min2.x < min.x)
            min.x = min2.x;
        if(min2.y < min.y)
            min.y = min2.y;
        if(min2.z < min.z)
            min.z = min2.z;

        Ogre::Vector3 max2 = srcMeshBB.getMaximum();
        if(max2.x > max.x)
            max.x = max2.x;
        if(max2.y > max.y)
            max.y = max2.y;
        if(max2.z > max.z)
            max.z = max2.z;
    }

    if(!bmin)
        bmin = new float[3];
    if(!bmax)
        bmax = new float[3];
    OgreRecast::OgreVect3ToFloatA(min, bmin);
    OgreRecast::OgreVect3ToFloatA(max, bmax);
}

float* InputGeom::getMeshBoundsMax()
{
    return bmax;
}

float* InputGeom::getMeshBoundsMin()
{
    return bmin;
}

int InputGeom::getVertCount()
{
    return nverts;
}

int InputGeom::getTriCount()
{
    return ntris;
}

int* InputGeom::getTris()
{
    return tris;
}

float* InputGeom::getVerts()
{
    return verts;
}

bool InputGeom::isEmpty()
{
    return nverts <= 0 || ntris <= 0;
}

void InputGeom::getMeshInformation(const Ogre::MeshPtr mesh,
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




void InputGeom::getManualMeshInformation(const Ogre::ManualObject *manual,
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
    for (size_t i=0; i < manual->getNumSections(); i++)
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

                for (size_t j = 0; j < renderOp->indexData->indexCount; j++)
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

void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
                                                                         unsigned char bidir, unsigned char area, unsigned short flags)
{
        if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
        float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
        m_offMeshConRads[m_offMeshConCount] = rad;
        m_offMeshConDirs[m_offMeshConCount] = bidir;
        m_offMeshConAreas[m_offMeshConCount] = area;
        m_offMeshConFlags[m_offMeshConCount] = flags;
        m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
        rcVcopy(&v[0], spos);
        rcVcopy(&v[3], epos);
        m_offMeshConCount++;
}

void InputGeom::deleteOffMeshConnection(int i)
{
        m_offMeshConCount--;
        float* src = &m_offMeshConVerts[m_offMeshConCount*3*2];
        float* dst = &m_offMeshConVerts[i*3*2];
        rcVcopy(&dst[0], &src[0]);
        rcVcopy(&dst[3], &src[3]);
        m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
        m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
        m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
        m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
}


ConvexVolume* InputGeom::getConvexHull(Ogre::Real offset)
{
// TODO who manages created convexVolume objects, and maybe better return pointer
    return new ConvexVolume(this, offset);
}

int InputGeom::getConvexVolumeId(ConvexVolume *convexHull)
{
    for(int i = 0; i < m_volumeCount; i++) {
        if(m_volumes[i] == convexHull)
            return i;
    }

    return -1;
}

int InputGeom::addConvexVolume(ConvexVolume *vol)
{
    // The maximum number of convex volumes that can be added to the navmesh equals the max amount
    // of volumes that can be added to the inputGeom it is built from.
    if (m_volumeCount >= InputGeom::MAX_VOLUMES)
        return -1;

    m_volumes[m_volumeCount] = vol;
    m_volumeCount++;

    return m_volumeCount-1; // Return index of created volume
}

bool InputGeom::deleteConvexVolume(int i, ConvexVolume** removedVolume)
{
    if(i >= m_volumeCount || i < 0)
        return false;


    *removedVolume = m_volumes[i];
    m_volumeCount--;
    m_volumes[i] = m_volumes[m_volumeCount];

    return true;
}

ConvexVolume* InputGeom::getConvexVolume(int volIndex)
{
    if (volIndex < 0 || volIndex > m_volumeCount)
        return NULL;

	return m_volumes[volIndex];
}


static bool isectSegAABB(const float* sp, const float* sq,
                                                 const float* amin, const float* amax,
                                                 float& tmin, float& tmax)
{
        static const float EPS = 1e-6f;

        float d[3];
        d[0] = sq[0] - sp[0];
        d[1] = sq[1] - sp[1];
        d[2] = sq[2] - sp[2];
        tmin = 0.0;
        tmax = 1.0f;

        for (int i = 0; i < 3; i++)
        {
                if (fabsf(d[i]) < EPS)
                {
                        if (sp[i] < amin[i] || sp[i] > amax[i])
                                return false;
                }
                else
                {
                        const float ood = 1.0f / d[i];
                        float t1 = (amin[i] - sp[i]) * ood;
                        float t2 = (amax[i] - sp[i]) * ood;
                        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
                        if (t1 > tmin) tmin = t1;
                        if (t2 < tmax) tmax = t2;
                        if (tmin > tmax) return false;
                }
        }

        return true;
}

int InputGeom::hitTestConvexVolume(const float* sp, const float* sq)
{
    float tmin = FLT_MAX;
    int volMinIdx = -1;
    for (int i = 0; i < m_volumeCount; ++i)
    {
        const ConvexVolume* vol = m_volumes[i];

        float t0,t1;

        if (isectSegAABB(sp, sq, vol->bmin, vol->bmax, t0, t1))
        {
            if (t0 < tmin)
            {
                tmin = t0;
                volMinIdx = i;
            }
        }
    }
    return volMinIdx;
}


static bool intersectSegmentTriangle(const float* sp, const float* sq,
                                                                         const float* a, const float* b, const float* c,
                                                                         float &t)
{
        float v, w;
        float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
        rcVsub(ab, b, a);
        rcVsub(ac, c, a);
        rcVsub(qp, sp, sq);

        // Compute triangle normal. Can be precalculated or cached if
        // intersecting multiple segments against the same triangle
        rcVcross(norm, ab, ac);

        // Compute denominator d. If d <= 0, segment is parallel to or points
        // away from triangle, so exit early
        float d = rcVdot(qp, norm);
        if (d <= 0.0f) return false;

        // Compute intersection t value of pq with plane of triangle. A ray
        // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
        // dividing by d until intersection has been found to pierce triangle
        rcVsub(ap, sp, a);
        t = rcVdot(ap, norm);
        if (t < 0.0f) return false;
        if (t > d) return false; // For segment; exclude this code line for a ray test

        // Compute barycentric coordinate components and test if within bounds
        rcVcross(e, qp, ap);
        v = rcVdot(ac, e);
        if (v < 0.0f || v > d) return false;
        w = -rcVdot(ab, e);
        if (w < 0.0f || v + w > d) return false;

        // Segment/ray intersects triangle. Perform delayed division
        t /= d;

        return true;
}

bool InputGeom::raycastMesh(float* src, float* dst, float& tmin)
{
        float dir[3];
        rcVsub(dir, dst, src);

        // Prune hit ray.
        float btmin, btmax;
        if (!isectSegAABB(src, dst, getMeshBoundsMin(), getMeshBoundsMax(), btmin, btmax))
                return false;
        float p[2], q[2];
        p[0] = src[0] + (dst[0]-src[0])*btmin;
        p[1] = src[2] + (dst[2]-src[2])*btmin;
        q[0] = src[0] + (dst[0]-src[0])*btmax;
        q[1] = src[2] + (dst[2]-src[2])*btmax;

        int cid[512];
        const int ncid = rcGetChunksOverlappingSegment(m_chunkyMesh, p, q, cid, 512);
        if (!ncid)
                return false;

        tmin = 1.0f;
        bool hit = false;
        const float* verts = getVerts();

        for (int i = 0; i < ncid; ++i)
        {
                const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
                const int* tris = &m_chunkyMesh->tris[node.i*3];
                const int ntris = node.n;

                for (int j = 0; j < ntris*3; j += 3)
                {
                        float t = 1;
                        if (intersectSegmentTriangle(src, dst,
                                                                                 &verts[tris[j]*3],
                                                                                 &verts[tris[j+1]*3],
                                                                                 &verts[tris[j+2]*3], t))
                        {
                                if (t < tmin)
                                        tmin = t;
                                hit = true;
                        }
                }
        }

        return hit;
}




void InputGeom::debugMesh(Ogre::SceneManager *sceneMgr)
{
    // Debug navmesh points
    Ogre::ManualObject *manual = sceneMgr->createManualObject("InputGeomDebug");

    // Draw inputGeom as triangles
    manual->begin("recastdebug", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (int i = 0; i<ntris*3; i++) {
        int triIdx = tris[i];
        manual->position(verts[3*triIdx], verts[3*triIdx+1], verts[3*triIdx+2]);
    }

    /*
    // Alternative drawing method: draw only the vertices
    manual->begin("recastdebug", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (int i = 0; i<nverts*3; i+=3) {
        manual->position(verts[i], verts[i+1], verts[i+2]);
    }
    */

    manual->end();
    sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
}













// ChunkyTriMesh

struct BoundsItem
{
    float bmin[2];
    float bmax[2];
    int i;
};

static int compareItemX(const void* va, const void* vb)
{
    const BoundsItem* a = (const BoundsItem*)va;
    const BoundsItem* b = (const BoundsItem*)vb;
    if (a->bmin[0] < b->bmin[0])
        return -1;
    if (a->bmin[0] > b->bmin[0])
        return 1;
    return 0;
}

static int compareItemY(const void* va, const void* vb)
{
    const BoundsItem* a = (const BoundsItem*)va;
    const BoundsItem* b = (const BoundsItem*)vb;
    if (a->bmin[1] < b->bmin[1])
        return -1;
    if (a->bmin[1] > b->bmin[1])
        return 1;
    return 0;
}

static void calcExtends(const BoundsItem* items, const int /*nitems*/,
                        const int imin, const int imax,
                        float* bmin, float* bmax)
{
    bmin[0] = items[imin].bmin[0];
    bmin[1] = items[imin].bmin[1];

    bmax[0] = items[imin].bmax[0];
    bmax[1] = items[imin].bmax[1];

    for (int i = imin+1; i < imax; ++i)
    {
        const BoundsItem& it = items[i];
        if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
        if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];

        if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
        if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
    }
}

inline int longestAxis(float x, float y)
{
    return y > x ? 1 : 0;
}

static void subdivide(BoundsItem* items, int nitems, int imin, int imax, int trisPerChunk,
                      int& curNode, rcChunkyTriMeshNode* nodes, const int maxNodes,
                      int& curTri, int* outTris, const int* inTris)
{
    int inum = imax - imin;
    int icur = curNode;

    if (curNode > maxNodes)
        return;

    rcChunkyTriMeshNode& node = nodes[curNode++];

    if (inum <= trisPerChunk)
    {
        // Leaf
        calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

        // Copy triangles.
        node.i = curTri;
        node.n = inum;

        for (int i = imin; i < imax; ++i)
        {
            const int* src = &inTris[items[i].i*3];
            int* dst = &outTris[curTri*3];
            curTri++;
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
        }
    }
    else
    {
        // Split
        calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

        int	axis = longestAxis(node.bmax[0] - node.bmin[0],
                                   node.bmax[1] - node.bmin[1]);

        if (axis == 0)
        {
            // Sort along x-axis
            qsort(items+imin, inum, sizeof(BoundsItem), compareItemX);
        }
        else if (axis == 1)
        {
            // Sort along y-axis
            qsort(items+imin, inum, sizeof(BoundsItem), compareItemY);
        }

        int isplit = imin+inum/2;

        // Left
        subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
        // Right
        subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

        int iescape = curNode - icur;
        // Negative index means escape.
        node.i = -iescape;
    }
}

bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
                           int trisPerChunk, rcChunkyTriMesh* cm)
{
    int nchunks = (ntris + trisPerChunk-1) / trisPerChunk;

    cm->nodes = new rcChunkyTriMeshNode[nchunks*4];
    if (!cm->nodes)
        return false;

    cm->tris = new int[ntris*3];
    if (!cm->tris)
        return false;

    cm->ntris = ntris;

    // Build tree
    BoundsItem* items = new BoundsItem[ntris];
    if (!items)
        return false;

    for (int i = 0; i < ntris; i++)
    {
        const int* t = &tris[i*3];
        BoundsItem& it = items[i];
        it.i = i;
        // Calc triangle XZ bounds.
        it.bmin[0] = it.bmax[0] = verts[t[0]*3+0];
        it.bmin[1] = it.bmax[1] = verts[t[0]*3+2];
        for (int j = 1; j < 3; ++j)
        {
            const float* v = &verts[t[j]*3];
            if (v[0] < it.bmin[0]) it.bmin[0] = v[0];
            if (v[2] < it.bmin[1]) it.bmin[1] = v[2];

            if (v[0] > it.bmax[0]) it.bmax[0] = v[0];
            if (v[2] > it.bmax[1]) it.bmax[1] = v[2];
        }
    }

    int curTri = 0;
    int curNode = 0;
    subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks*4, curTri, cm->tris, tris);

    delete [] items;

    cm->nnodes = curNode;

    // Calc max tris per node.
    cm->maxTrisPerChunk = 0;
    for (int i = 0; i < cm->nnodes; ++i)
    {
        rcChunkyTriMeshNode& node = cm->nodes[i];
        const bool isLeaf = node.i >= 0;
        if (!isLeaf) continue;
        if (node.n > cm->maxTrisPerChunk)
            cm->maxTrisPerChunk = node.n;
    }

    return true;
}


inline bool checkOverlapRect(const float amin[2], const float amax[2],
                             const float bmin[2], const float bmax[2])
{
    bool overlap = true;
    overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
    overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
    return overlap;
}

int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm,
                               float bmin[2], float bmax[2],
                               int* ids, const int maxIds)
{
    // Traverse tree
    int i = 0;
    int n = 0;
    while (i < cm->nnodes)
    {
        const rcChunkyTriMeshNode* node = &cm->nodes[i];
        const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
        const bool isLeafNode = node->i >= 0;

        if (isLeafNode && overlap)
        {
            if (n < maxIds)
            {
                ids[n] = i;
                n++;
            }
        }

        if (overlap || isLeafNode)
            i++;
        else
        {
            const int escapeIndex = -node->i;
            i += escapeIndex;
        }
    }

    return n;
}



static bool checkOverlapSegment(const float p[2], const float q[2],
                                const float bmin[2], const float bmax[2])
{
    static const float EPSILON = 1e-6f;

    float tmin = 0;
    float tmax = 1;
    float d[2];
    d[0] = q[0] - p[0];
    d[1] = q[1] - p[1];

    for (int i = 0; i < 2; i++)
    {
        if (fabsf(d[i]) < EPSILON)
        {
            // Ray is parallel to slab. No hit if origin not within slab
            if (p[i] < bmin[i] || p[i] > bmax[i])
                return false;
        }
        else
        {
            // Compute intersection t value of ray with near and far plane of slab
            float ood = 1.0f / d[i];
            float t1 = (bmin[i] - p[i]) * ood;
            float t2 = (bmax[i] - p[i]) * ood;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }
    return true;
}

int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm,
                                  float p[2], float q[2],
                                  int* ids, const int maxIds)
{
    // Traverse tree
    int i = 0;
    int n = 0;
    while (i < cm->nnodes)
    {
        const rcChunkyTriMeshNode* node = &cm->nodes[i];
        const bool overlap = checkOverlapSegment(p, q, node->bmin, node->bmax);
        const bool isLeafNode = node->i >= 0;

        if (isLeafNode && overlap)
        {
            if (n < maxIds)
            {
                ids[n] = i;
                n++;
            }
        }

        if (overlap || isLeafNode)
            i++;
        else
        {
            const int escapeIndex = -node->i;
            i += escapeIndex;
        }
    }

    return n;
}


Ogre::ManualObject* InputGeom::drawConvexVolume(ConvexVolume *vol, Ogre::SceneManager* sceneMgr, Ogre::ColourValue color)
{
    // Define manualObject with convex volume faces
    Ogre::ManualObject *manual = sceneMgr->createManualObject();
    // Set material
    manual->begin("recastdebug", Ogre::RenderOperation::OT_LINE_LIST) ;

    for (int i = 0, j = vol->nverts-1; i < vol->nverts; j = i++)
    {
        const float* vi = &vol->verts[j*3];
        const float* vj = &vol->verts[i*3];

        manual->position(vj[0], vol->hmin, vj[2]);  manual->colour(color);
        manual->position(vi[0], vol->hmin, vi[2]);  manual->colour(color);
        manual->position(vj[0], vol->hmax, vj[2]);  manual->colour(color);
        manual->position(vi[0], vol->hmax, vi[2]);  manual->colour(color);
        manual->position(vj[0], vol->hmin, vj[2]);  manual->colour(color);
        manual->position(vj[0], vol->hmax, vj[2]);  manual->colour(color);
    }
/*
    // Create triangles
    for (int i = 0; i < vol->nverts-1; ++i) {   // Number of quads (is number of tris -1)
        // We ignore the y component and just draw a box between min height and max height

        // Triangle 1 of the quad
        // Bottom vertex
        manual->position(vol->verts[3*i], vol->hmin,vol->verts[3*i+2]);
        manual->colour(color);

        // Top vertex
        manual->position(vol->verts[3*i], vol->hmax,vol->verts[3*i+2]);
        manual->colour(color);

        // Next bottom vertex
        manual->position(vol->verts[3*i+3], vol->hmin,vol->verts[3*i+5]);
        manual->colour(color);


        // Triangle 2 of the quad
        // Bottom vertex
        manual->position(vol->verts[3*i], vol->hmin,vol->verts[3*i+2]);
        manual->colour(color);

        // Top vertex
        manual->position(vol->verts[3*i], vol->hmax,vol->verts[3*i+2]);
        manual->colour(color);

        // Next Top vertex
        manual->position(vol->verts[3*i+3], vol->hmax,vol->verts[3*i+5]);
        manual->colour(color);
    }
*/

    manual->end();
    sceneMgr->getRootSceneNode()->attachObject(manual); // Just attach it to root scenenode since its coordinates are in world-space
    return manual;
}


Ogre::AxisAlignedBox InputGeom::getWorldSpaceBoundingBox(Ogre::MovableObject *ent) {
    Ogre::SceneManager *sceneMgr = ent->getParentSceneNode()->getCreator();
    Ogre::Matrix4 transform = sceneMgr->getRootSceneNode()->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();
    Ogre::AxisAlignedBox bb = ent->getBoundingBox();
    bb.transform(transform);

    return bb;
}


Ogre::AxisAlignedBox InputGeom::getBoundingBox()
{
    Ogre::AxisAlignedBox bb;
    Ogre::Vector3 max;
    Ogre::Vector3 min;
    OgreRecast::FloatAToOgreVect3(bmin, min);
    OgreRecast::FloatAToOgreVect3(bmax, max);
    bb.setMaximum(max);
    bb.setMinimum(min);

    return bb;
}


void InputGeom::writeObj(Ogre::String filename)
{
    std::fstream fstr(filename.c_str(), std::ios::out);
//    Ogre::DataStreamPtr stream(OGRE_NEW Ogre::FileStreamDataStream(&fstr, false));
//    Ogre::StreamSerialiser streamWriter(stream);

    for(int i=0; i < nverts; i++) {
        Ogre::String line = "v "+Ogre::StringConverter::toString(verts[3*i])+" "+Ogre::StringConverter::toString(verts[3*i+1])+" "+Ogre::StringConverter::toString(verts[3*i+2]);
        fstr << line << std::endl;
//        streamWriter.write(&line);
    }
    for(int i=0; i < ntris; i++) {
        Ogre::String line = "f "+Ogre::StringConverter::toString(1+tris[3*i])+" "+Ogre::StringConverter::toString(1+tris[3*i+1])+" "+Ogre::StringConverter::toString(1+tris[3*i+2]);
        fstr << line << std::endl;
    }

    fstr.close();
}



void InputGeom::applyOrientation(Ogre::Quaternion orientation, Ogre::Vector3 pivot)
{
// TODO allow this or not?
    /*
    if(mTerrainGroup)
        return; // It makes no sense to do this if this inputGeom contains terrain!
    */


    // Apply transformation to all verts
    Ogre::Matrix4 transform = Ogre::Matrix4(orientation); // Convert quaternion into regular transformation matrix
    Ogre::Vector3 vert;
    for (int i = 0; i < nverts; i++) {
        // Obtain vertex (translated towards pivot point)
        vert.x = verts[3*i + 0] - pivot.x;
        vert.y = verts[3*i + 1] - pivot.y;
        vert.z = verts[3*i + 2] - pivot.z;

        // Apply rotation to vector
        vert = transform * vert;

        // Store the rotated vector and add translation away from pivot point again
        verts[3*i + 0] = vert.x + pivot.x;
        verts[3*i + 1] = vert.y + pivot.y;
        verts[3*i + 2] = vert.z + pivot.z;
    }


    // Transform extents
    // Abandoned for now: this means bounding boxes are not rotated any that you best
    // rotate only pretty symmetrical shapes
    // If this does not suffice you could use the bounding box or hull from the physics engine
    // Or fully recalculate the bounding box from all vertices
}

void InputGeom::move(Ogre::Vector3 translation)
{
    // Apply translation to all verts
    for (int i = 0; i < nverts; i++) {
        verts[3*i + 0] += translation.x;
        verts[3*i + 1] += translation.y;
        verts[3*i + 2] += translation.z;
    }


    // Transform extents
    bmin[0] += translation.x;
    bmin[1] += translation.y;
    bmin[2] += translation.z;

    bmax[0] += translation.x;
    bmax[1] += translation.y;
    bmax[2] += translation.z;
}


Ogre::ManualObject* InputGeom::drawBoundingBox(Ogre::AxisAlignedBox box, Ogre::SceneManager *sceneMgr, Ogre::ColourValue color)
{
    ConvexVolume *cv = new ConvexVolume(box);
    Ogre::ManualObject *result = InputGeom::drawConvexVolume(cv, sceneMgr, color);
    delete cv;

    return result;
}
