#ifndef RECASTCONVEXHULL_H
#define RECASTCONVEXHULL_H

class InputGeom;

/**
  * The maximum amount of points that a convex hull
  * used for dynamic obstacles on a navmesh can consist of.
  * For performance reasons this cannot be too high.
  * But has to have at least twice the number of vertices that
  * the input has!!!
  **/
static const int MAX_CONVEXVOL_PTS = 90;    // TODO increase? or find better convex hull algorithm

/**
  * Volume describing a convex hull around
  * geometry. Can be used as a collision mesh
  * for dynamic obstacles.
  **/
// TODO also calculate and store height above the navmesh (boxDescent) and use boxHeight for hmax?
class ConvexVolume
{
public:
    /**
      * Create a convex hull in 2D space (on the xz plane) from
      * the specified 3D points.
      **/
    ConvexVolume(InputGeom *geom, float offset = 0.0f);

    float verts[MAX_CONVEXVOL_PTS*3];
    float hmin, hmax;
    float bmin[3], bmax[3];
    int nverts;
    int area;

private:
    static inline bool cmppt(const float* a, const float* b);
    static inline bool left(const float* a, const float* b, const float* c);
};

#endif // RECASTCONVEXHULL_H
