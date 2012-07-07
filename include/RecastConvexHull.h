#ifndef RECASTCONVEXHULL_H
#define RECASTCONVEXHULL_H

#include <Ogre.h>

class InputGeom;    // Advance declaration

/**
  * The maximum amount of points that a convex hull
  * used for dynamic obstacles on a navmesh can consist of.
  * For performance reasons this cannot be too high.
  * But has to have at least twice the number of vertices that
  * the input has!!!
  **/
static const int MAX_CONVEXVOL_PTS = 90;    // TODO increase? or find better convex hull algorithm

/**
  * Volume describing a convex hull around geometry. Or a convex area to mark on the navmesh.
  * Can be used as a collision mesh for dynamic obstacles.
  *
  * Can also be used for marking areas of navmesh polygons with a specific flag.
  * For example, mark a convex area with area set to RC_NULL_AREA to make the area unwalkable (obstacle).
  * Area marking can also be used for marking more specific areas, such as swim (water),
  * mud, door, ...
  * Areas can assign weights to navmesh polygons that can be used during A* pathfinding
  * for determining the optimal path.
  *
  * Note that convex volume marking is not the only way of assigning flags to navmesh polygons.
  * Navmesh polygons can also be assigned a flag directly, the problem is that with automatic
  * navmesh generation you have no control over the exact size and position of each polygon on the
  * navmesh, making it sometimes impossible to mark an intended area precisely. That's why recast
  * offers a rcMarkConvexArea (and in the future hopefully dtMarkConvexArea) to allow you to
  * explicitly mark where a separate polygon should be in the navmesh, so that you can assign it
  * a custom area flag. Convex area marking is also referred to as cookie cutter algorithm.
  *
  * If you define a custom constructor for convexVolume objects, note that recast requires the
  * vertices of convex shapes to be wound in clockwise order (this is very important, or convex
  * area marking will not work)!
  *
  * Also note that recast requires convex volumes to be offset with the agent radius so the agent
  * does not collide with the edges.
  **/
class ConvexVolume
{
public:
    /**
      * Create a convex hull in 2D space (on the xz plane) from
      * the specified 3D points.
      **/
    ConvexVolume(InputGeom *geom, float offset = 0.0f);

    /**
      * Create a convex hull from a bounding box
      **/
    ConvexVolume(Ogre::AxisAlignedBox boundingBox, float offset = 0.0f);

    /**
      * Move this convex hull to a new world position offset with specified
      * translation vector.
      * Can be done pretty fast. Due to the 2D nature of this convex hull it's
      * not possible to apply an arbitrary rotation, however (though in theory
      * rotations around Y axis would work).
      **/
    void move(Ogre::Vector3 translate);

    /**
      * The vertices of this convex hull.
      * Vertices are stored as three subsequent values, in order x, y, z
      * Size of this array is always a multiple of 3, exactly 3*nverts
      **/
    float verts[MAX_CONVEXVOL_PTS*3];

    /**
      * Number of vertices in verts.
      * The size of the verts array is actually nverts*3
      **/
    int nverts;

    /**
      * Minimum and maximum height of this convex hull.
      **/
    float hmin, hmax;

    /**
      * Axis aligned boundig box minimum and maximum of this convex hull.
      **/
    float bmin[3], bmax[3];

    /**
      * Area flag for the navmesh polygon that will be marked with this convex volume.
      * For example, set to RC_NULL_AREA to make the area unwalkable (obstacle).
      * Area marking can also be used for marking more specific areas, such as swim (water),
      * mud, door, ...
      * Areas can assign weights to navmesh polygons that can be used during A* pathfinding
      * for determining the optimal path.
      **/
    int area;

private:
    /**
      * Compare two points. Returns true if they are equal.
      **/
    static inline bool cmppt(const float* a, const float* b);

    /**
      * isLeftOf comparison. Returns true if point c is left of line a-b.
      **/
    static inline bool left(const float* a, const float* b, const float* c);
};

#endif // RECASTCONVEXHULL_H
