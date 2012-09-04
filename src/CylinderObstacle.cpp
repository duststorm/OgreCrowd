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

#include "CylinderObstacle.h"
#include "OgreRecastApplication.h"


CylinderObstacle::CylinderObstacle(Ogre::Vector3 position, OgreDetourTileCache *detourTileCache)
    : Obstacle(detourTileCache),
      mPosition(position),
      mEnt(0),
      mNode(0)
{
    // Create cylindrical obstacle in detourTileCache
    mObstacleRef = mDetourTileCache->addTempObstacle(mPosition);
    if (mObstacleRef) {
        mName = "CylinderObstacle_"+Ogre::StringConverter::toString(mObstacleRef);

        // Depict osbtacle as red cylinder
        mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(mName+"Node");
        mEnt = mSceneMgr->createEntity(mName, "Cylinder.mesh");
        mEnt->setMaterialName("Cylinder/Red");
        mNode->attachObject(mEnt);
        mNode->setPosition(mPosition);
        mNode->setScale(TEMP_OBSTACLE_RADIUS, TEMP_OBSTACLE_HEIGHT, TEMP_OBSTACLE_RADIUS);

//        mNode->setVisible(mDebugDraw);
        mEnt->setQueryFlags(OgreRecastApplication::OBSTACLE_MASK);  // add to query group for obstacles
    }

    // TODO have some way of notifying that obstacle creation failed? (exception maybe?)
}

CylinderObstacle::~CylinderObstacle()
{
    // Remove obstacle from detour tilecache
    mDetourTileCache->removeTempObstacle(mObstacleRef);

    mNode->removeAllChildren();
    mNode->getParentSceneNode()->removeChild(mNode);
    mSceneMgr->destroyEntity(mEnt);
    mSceneMgr->destroySceneNode(mNode);

    mNode = NULL;
    mEnt = NULL;
}

void CylinderObstacle::update(long time)
{
}


dtObstacleRef CylinderObstacle::getObstacleRef()
{
    return mObstacleRef;
}


Ogre::Entity* CylinderObstacle::getEntity()
{
    return mEnt;
}


void CylinderObstacle::updatePosition(Ogre::Vector3 position)
{
    // Modify position if larger than epsilon
    if ( mPosition.squaredDistance(position) > SQUARED_DISTANCE_EPSILON ) {
        mPosition = position;

        // Remove obstacle and re-add it at new location
        mDetourTileCache->removeTempObstacle(mObstacleRef);
        mObstacleRef = mDetourTileCache->addTempObstacle(mPosition);
    }
}

Ogre::Vector3 CylinderObstacle::getPosition()
{
    return mPosition;
}

void CylinderObstacle::updateOrientation(Ogre::Quaternion orientation)
{
    // Do nothing
    return;
}

Ogre::Quaternion CylinderObstacle::getOrientation()
{
    return Ogre::Quaternion::IDENTITY;
}
