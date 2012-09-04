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

#ifndef CONVEXSHAPEOBSTACLE_H
#define CONVEXSHAPEOBSTACLE_H

#include "Obstacle.h"
#include "RecastConvexHull.h"

/**
  * Convex obstacles are a more complex type of temporary obstacles that can be used together with
  * the Detour Tile Cache.
  * In contrast to simple cylindrical obstacles they allow any type of geometry to be defined as an obstacle.
  * A convex obstacle relies on a simple convex hull of the obstacle it represents. It uses a simplified 2D
  * approximation of the convex hull with a height (the hull is on the x-z ground plane, and has one height
  * that is the same for each point, kind of like an upright cylinder where the sides don't have to be simply
  * round but can be any shape).
  * Convex obstacles can both be moved and given a different orientation (rotation). When rotating the convex
  * hull approximation has to be rebuilt, as it is only a simple 2D approximation instead of a full 3D hull.
  * Also note that the convex hull algorithm that is currently used is very fast but also very limited. Don't
  * feed it too much geometry.
  * Alternatively, when not rotating obstacles it's also possible to use the bounding box of the object instead
  * of calculating a hull (in case the bounding box approximates the obstacle well, for example in case of a
  * box or rectangular shape).
  **/
class ConvexShapeObstacle : public Obstacle
{
public:
    /**
      * Construct a convex shape obstacle on the specified position.
      * To make this work with detour tileCache a simplified convex hull is created
      * from the obstacle entity's geometry. The hull is offset with the specified
      * amount from the mesh (offset should generally be agent radius).
      * DetourTileCache parameter specifies the tilecache on which the obstacle will
      * be added.
      **/
    ConvexShapeObstacle(Ogre::Vector3 position, Ogre::Real offset, OgreDetourTileCache *detourTileCache);
    virtual ~ConvexShapeObstacle(); // Important that the destructor is virtual!

    /**
      * @see{Obstacle::update()}
      **/
    virtual void update(long time);

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
      * Update the orientation (mainly intended for rotation and position) of the obstacle
      * to the specified orientation.
      * Replaces the old rotation completely (this is not cumulative).
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
      * Current orientation of this obstacle.
      **/
    Ogre::Quaternion mOrientation;

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

    /**
      * 2D Convex hull with height calculated for this obstacle (for the current rotation).
      **/
    ConvexVolume *mConvexHull;

    /**
      * Inputgeom object containing the original geometry of this obstacle (used to generate convex hull from).
      **/
    InputGeom *mInputGeom;

    /**
      * Amount that the convex hull is offset from the inputGeom. Normally detour requires the hulls to be offset
      * with at least the radius of the agents.
      **/
    Ogre::Real mOffset;

    /**
      * Visual debug representation of the calculated convex hull, in the form of a line drawing.
      **/
    Ogre::ManualObject *mConvexHullDebug;
};

#endif // CONVEXSHAPEOBSTACLE_H
