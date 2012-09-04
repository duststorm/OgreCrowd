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

#include "Obstacle.h"

// Minimum distance an obstacle has to be moved before the obstacle is updated
const Ogre::Real Obstacle::SQUARED_DISTANCE_EPSILON = 0.1f * 0.1f;

// Minimum difference a new orientation has to have from the previous one for obstacle orientation to be updated
const Ogre::Real Obstacle::ORIENTATION_TOLERANCE_DEGREES = 2.0f;



Obstacle::Obstacle(OgreDetourTileCache *detourTileCache)
    : mDetourTileCache(detourTileCache),
      mSceneMgr(detourTileCache->m_recast->m_pSceneMgr)
{

}

Obstacle::~Obstacle()
{
}
