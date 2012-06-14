#include "RecastConvexHull.h"
#include "RecastInputGeom.h"
#include "Recast/Recast.h"
#include "OgreRecastDefinitions.h"

// Calculates convex hull on xz-plane of points on 'pts'
ConvexVolume::ConvexVolume(InputGeom* geom, float offset)
{
// TODO protect against too many vectors!

    int hullVertIndices[MAX_CONVEXVOL_PTS];
    float* pts = geom->verts;
    int npts = geom->nverts;

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
    while (endpt != hullVertIndices[0]);

    nverts = i;


    // Copy geometry vertices to convex hull
    for (int i = 0; i < nverts; i++)
        rcVcopy(&verts[i*3], &geom->verts[hullVertIndices[i]*3]);



    area = SAMPLE_POLYAREA_DOOR;   // You can choose whatever flag you assing to the poly area

    // Find min and max height of convex hull
    hmin = geom->bmin[1];
    hmax = geom->bmax[1];
/*
    hull->hmin = FLT_MAX; hull->hmax = 0;
    for (int i = 0; i < nbHullVerts; ++i)
        hull->hmin = rcMin(hull->hmin, hull->verts[i*3+1]);
// TODO set descent and height (see demo for details)
//    hull.hmin -= m_boxDescent;
//    hull.hmax = hull.hmin + m_boxHeight;
*/
    // 3D mesh min and max bounds
    rcVcopy(bmin, geom->bmin);
    rcVcopy(bmax, geom->bmax);

//TODO offset is still broken for a lot of shapes! Fix this!
    // Offset convex hull if needed
    if(offset > 0.01f) {
        float offsetVerts[2*MAX_CONVEXVOL_PTS * 3]; // An offset hull is allowed twice the number of vertices
        int nOffsetVerts = rcOffsetPoly(verts, nverts, offset, offsetVerts, MAX_CONVEXVOL_PTS*2);

        if (nOffsetVerts <= 0)
            return;

//TODO fix difference in max_pts for offset and non-offset hull
        for(int i = 0; i < nOffsetVerts; i++)
            rcVcopy(&verts[i*3], &offsetVerts[i*3]);
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
