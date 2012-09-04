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

#ifndef CYLINDEROBSTACLE_H
#define CYLINDEROBSTACLE_H

#include "Obstacle.h"

/**
  * A cylinder obstacle is the most simple of obstacles (and probably the fastest).
  * Cylinder obstacles are upright cylinders (cannot be rotated, only be moved) that have
  * a height, a radius and are placed at a specified position.
  * Cylindrical obstacles are ideal for symbolizing things like persons or pillars (in fact
  * detourCrowd agents are also cylinders) and are very fast to update on the navmesh.
  **/
class CylinderObstacle : public Obstacle
{
public:
    /**
      * Construct a simple cylindrical obstacle with radius TEMP_OBSTACLE_RADIUS,
      * at specified position.
      **/
    CylinderObstacle(Ogre::Vector3 position, OgreDetourTileCache *detourTileCache);
    ~CylinderObstacle();

    /**
      * @see{Obstacle::update()}
      **/
    virtual void update(long time);

    /**
      * The reference to the temp obstacle for this obstacle in the detourTileCache.
      **/
    virtual dtObstacleRef getObstacleRef(void);

    /**
      * @see{Obstacle::getEntity()}
      **/
    virtual Ogre::Entity* getEntity(void);

    /**
      * @see{Obstacle::getPosition()}
      **/
    virtual Ogre::Vector3 getPosition(void);

    /**
      * @see{Obstacle::getOrientation()}
      **/
    virtual Ogre::Quaternion getOrientation(void);

    /**
      * Not applicable to cylindrical obstacles. Does nothing.
      **/
    virtual void updateOrientation(Ogre::Quaternion orientation);

    /**
      * @see{Obstacle::updatePosition(Ogre::Vector3)}
      **/
    virtual void updatePosition(Ogre::Vector3 position);

protected:
    /**
      * Current position of this obstacle.
      **/
    Ogre::Vector3 mPosition;

    /**
      * The reference to the obstacle in the detourTileCache.
      **/
    dtObstacleRef mObstacleRef;

    /**
      * The entity representing this obstacle in the scene.
      **/
    Ogre::Entity *mEnt;

    /**
      * The scene node in which the obstacle entity is located.
      **/
    Ogre::SceneNode *mNode;

    /**
      * Name of this obstacle.
      **/
    Ogre::String mName;
};

#endif // CYLINDEROBSTACLE_H
