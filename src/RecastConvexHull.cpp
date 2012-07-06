#include "RecastConvexHull.h"
#include "RecastInputGeom.h"
#include "Recast/Recast.h"
#include "OgreRecastDefinitions.h"
#include "OgreRecast.h"

// Calculates convex hull on xz-plane of points on 'pts'
ConvexVolume::ConvexVolume(InputGeom* geom, float offset)
{
// TODO protect against too many vectors!

    int hullVertIndices[MAX_CONVEXVOL_PTS];
    float* pts = geom->getVerts();
    int npts = geom->getVertCount();

    // Find lower-leftmost point.
    int hull = 0;
    for (int i = 1; i < npts; ++i)
        if (ConvexVolume::cmppt(&pts[i*3], &pts[hull*3]))
            hull = i;
    // Gift wrap hull.
    int endpt = 0;
    int i = 0;
    do
    {
        hullVertIndices[i++] = hull;
        endpt = 0;
        for (int j = 1; j < npts; ++j)
            if (hull == endpt || ConvexVolume::left(&pts[hull*3], &pts[endpt*3], &pts[j*3]))
                endpt = j;
        hull = endpt;
    }
    while (endpt != hullVertIndices[0] && i < MAX_CONVEXVOL_PTS/2);   // TODO: number of hull points is limited, but in a naive way. In large meshes the best candidate points for the hull might not be selected
                                            // Leave the other half of the points for expanding the hull

    nverts = i;


    // Copy geometry vertices to convex hull
    for (int i = 0; i < nverts; i++)
        rcVcopy(&verts[i*3], &pts[hullVertIndices[i]*3]);



    area = SAMPLE_POLYAREA_DOOR;   // You can choose whatever flag you assing to the poly area

    // Find min and max height of convex hull
    hmin = geom->getMeshBoundsMin()[1];
    hmax = geom->getMeshBoundsMax()[1];

    // 3D mesh min and max bounds
    rcVcopy(bmin, geom->getMeshBoundsMin());
    rcVcopy(bmax, geom->getMeshBoundsMax());

//TODO offsetting is still broken for a lot of shapes! Fix this!
    // Offset convex hull if needed
    if(offset > 0.01f) {
        float offsetVerts[MAX_CONVEXVOL_PTS * 3]; // An offset hull is allowed twice the number of vertices
        int nOffsetVerts = rcOffsetPoly(verts, nverts, offset, offsetVerts, MAX_CONVEXVOL_PTS);

        if (nOffsetVerts <= 0)
            return;

        for(int i = 0; i < nOffsetVerts; i++)
            rcVcopy(&verts[i*3], &offsetVerts[i*3]);

        nverts = nOffsetVerts;

        // Modify the bounds with offset (except height)
        bmin[0] = bmin[0]-offset;
        bmin[2] = bmin[2]-offset;
        bmax[0] = bmax[0]+offset;
        bmax[2] = bmax[2]+offset;
    }

}


// Returns true if 'a' is more lower-left than 'b'.
bool ConvexVolume::cmppt(const float* a, const float* b)
{
    if (a[0] < b[0]) return true;
    if (a[0] > b[0]) return false;
    if (a[2] < b[2]) return true;
    if (a[2] > b[2]) return false;
    return false;
}

// Returns true if 'c' is left of line 'a'-'b'.
bool ConvexVolume::left(const float* a, const float* b, const float* c)
{
    const float u1 = b[0] - a[0];
    const float v1 = b[2] - a[2];
    const float u2 = c[0] - a[0];
    const float v2 = c[2] - a[2];
    return u1 * v2 - v1 * u2 < 0;
}


ConvexVolume::ConvexVolume(Ogre::AxisAlignedBox boundingBox, float offset)
{
    Ogre::Vector3 max = boundingBox.getMaximum();
    Ogre::Vector3 min = boundingBox.getMinimum();

    // Offset bounding box (except height)
    if(offset > 0.01f) {
        max = max + offset*Ogre::Vector3(1,0,1);
        min = min - offset*Ogre::Vector3(1,0,1);
    }

    // Create box verts (in clockwise fashion!!)
    verts[0]= min.x; verts[1]= min.y; verts[2]= max.z;
    verts[3]= max.x; verts[4]= max.y; verts[5]= max.z;
    verts[6]= max.x; verts[7]= max.y; verts[8]= min.z;
    verts[9]= min.x; verts[10]= min.y; verts[11]= min.z;
    nverts = 4; // For rcMarkConvexPoly the verts of the shape need to be in clockwise order

    // Set bounding box limits
    OgreRecast::OgreVect3ToFloatA(min, bmin);
    OgreRecast::OgreVect3ToFloatA(max, bmax);

    // Set height limits
    hmin = min.y;
    hmax = max.y;

    area = SAMPLE_POLYAREA_DOOR;   // You can choose whatever flag you assing to the poly area
}


void ConvexVolume::move(Ogre::Vector3 translate)
{
    // Offset all verts with translation vector
    for( int i = 0; i < nverts; i++) {
        verts[3*i +0] += translate.x;
        verts[3*i +1] += translate.y;
        verts[3*i +2] += translate.z;
    }

    // Recalculate bounds
    bmin[0] += translate.x;
    bmin[1] += translate.y;
    bmin[2] += translate.z;

    bmax[0] += translate.x;
    bmax[1] += translate.y;
    bmax[2] += translate.z;

    hmin += translate.y;
    hmax += translate.y;
}
