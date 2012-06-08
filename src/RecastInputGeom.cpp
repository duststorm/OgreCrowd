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


    // Convert ogre geometry (vertices, triangles and normals)
    convertOgreEntities();


    m_offMeshConCount = 0;
    m_volumeCount = 0;

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

InputGeom::~InputGeom()
{
    delete m_chunkyMesh;
    delete[] verts;
    delete[] normals;
    delete[] tris;
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

void InputGeom::addConvexVolume(const float* verts, const int nverts,
                                                                const float minh, const float maxh, unsigned char area)
{
        if (m_volumeCount >= MAX_VOLUMES) return;
        ConvexVolume* vol = &m_volumes[m_volumeCount++];
        memset(vol, 0, sizeof(ConvexVolume));
        memcpy(vol->verts, verts, sizeof(float)*3*nverts);
        vol->hmin = minh;
        vol->hmax = maxh;
        vol->nverts = nverts;
        vol->area = area;
}

void InputGeom::deleteConvexVolume(int i)
{
        m_volumeCount--;
        m_volumes[i] = m_volumes[m_volumeCount];
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