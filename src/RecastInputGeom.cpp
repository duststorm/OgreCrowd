#include "RecastInputGeom.h"
#include "OgreRecast.h"

InputGeom::InputGeom(std::vector<Ogre::Entity*> srcMeshes)
    : mSrcMeshes(srcMeshes),
    nverts(0),
    ntris(0),
    mReferenceNode(0)
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


    //Convert all vertices and triangles to recast format
    const int numNodes = srcMeshes.size();
    size_t *meshVertexCount = new size_t[numNodes];
    size_t *meshIndexCount = new size_t[numNodes];
    Ogre::Vector3 **meshVertices = new Ogre::Vector3*[numNodes];
    unsigned long **meshIndices = new unsigned long*[numNodes];

    nverts = 0;
    ntris = 0;
    size_t i = 0;
    for(std::vector<Ogre::Entity*>::iterator iter = srcMeshes.begin(); iter != srcMeshes.end(); iter++) {
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
    for (std::vector<Ogre::Entity*>::iterator iter = srcMeshes.begin(); iter != srcMeshes.end(); iter++) {
        Ogre::Entity *ent = *iter;
        //find the transform between the reference node and this node
        Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();
        Ogre::Vector3 vertexPos;
        for (uint j = 0 ; j < meshVertexCount[i] ; j++)
        {
            vertexPos = transform*meshVertices[i][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (uint j = 0 ; j < meshIndexCount[i] ; j++)
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

void InputGeom::calculateExtents()
{
    Ogre::Entity* ent = mSrcMeshes[0];
    const Ogre::AxisAlignedBox srcMeshBB = ent->getBoundingBox();
    Ogre::Matrix4 transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();
    Ogre::Vector3 min = transform * srcMeshBB.getMinimum();
    Ogre::Vector3 max = transform * srcMeshBB.getMaximum();

    // Calculate min and max from all entities
    for(std::vector<Ogre::Entity*>::iterator iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        Ogre::Entity* ent = *iter;

        //find the transform between the reference node and this node
        transform = mReferenceNode->_getFullTransform().inverse() * ent->getParentSceneNode()->_getFullTransform();

        const Ogre::AxisAlignedBox srcMeshBB = ent->getBoundingBox();
         Ogre::Vector3 min2 = transform * srcMeshBB.getMinimum();
         if(min2.x < min.x)
             min.x = min2.x;
         if(min2.y < min.y)
             min.y = min2.y;
         if(min2.z < min.z)
             min.z = min2.z;

         Ogre::Vector3 max2 = transform * srcMeshBB.getMaximum();
         if(max2.x > max.x)
             max.x = max2.x;
         if(max2.y > max.y)
             max.y = max2.y;
         if(max2.z > max.z)
             max.z = max2.z;
    }

    bmin = new float[3];
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



void InputGeom::debugMesh()
{
    // Debug navmesh points
    Ogre::SceneManager *sm = mSrcMeshes[0]->getParentSceneNode()->getCreator();
    Ogre::ManualObject *manual = sm->createManualObject("InputGeomDebug");
    manual->begin("dungeon", Ogre::RenderOperation::OT_POINT_LIST);
    for (int i = 0; i<nverts*3; i+=3) {
        manual->position(verts[i], verts[i+1], verts[i+2]);
    }

    for (int i = 0; i<ntris*3; i++) {
        manual->index(tris[i]);
    }

    manual->end();
    sm->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
}
