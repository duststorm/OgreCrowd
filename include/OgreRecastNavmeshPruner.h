/*
    OgreCrowd
    ---------

    Copyright (c) 2012 Jonas Hauquier

    Additional contributions by:

    - mkultra333
    - Paul Wilson

    Sincere thanks and to:

    - Mikko Mononen (developer of Recast navigation libraries)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/


#ifndef OGRERECASTNAVMESHPRUNER_H
#define OGRERECASTNAVMESHPRUNER_H

#include "OgreRecast.h"

/**
 * Tool to flood-fill the navmesh polygons starting at a specific polygon and marking
 * all reachable neighbours. This allows to prune off unreachable parts of the navmesh.
 *
 * Based on NavMeshPruneTool from the original recast sample.
 * The navmesh prune tool allows to disable unreachable polygons. There is currently no
 * way to discard the disabled polys, because the search is done on the final navmesh
 * data (when it is already rasterized) and it is really hard to modify.
 **/
class OgreRecastNavmeshPruner
{
public:
    OgreRecastNavmeshPruner(OgreRecast *recast, dtNavMesh *navMesh);
    bool floodNavmesh(Ogre::Vector3 startPoint);
    void clearSelection(void);
    void pruneSelected(void);

protected:
    void floodNavmesh(dtPolyRef start, unsigned char flag);
    void disableUnvisitedPolys(void);

    OgreRecast *mRecast;
    dtNavMesh *mNavmesh;
    class NavmeshFlags* mFlags;

};

#endif // OGRERECASTNAVMESHPRUNER_H
