#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "OgreDetourTileCache.h"

/**
  * Dynamic obstacle that can be put on the navmesh when using DetourTileCache.
  * Obstacles only mark polygons of the navmesh with a special flag (such as unwalkable)
  * and do not actually add new geometry to the navmesh input. This makes them faster,
  * but no new areas can be added to the navmesh, only areas can be marked or cut out.
  * Obstacles are a lot faster than rebuilding a navmesh tile with new geometry, however.
  **/
class Obstacle
{
public:
    /**
      * Construct an obstacle that will be added to the specified detour Tilecache.
      **/
    Obstacle(OgreDetourTileCache *detourTileCache);
    virtual ~Obstacle();

    /**
      * No use at the moment.
      * The future plan was to use it to do deferred updates of obstacle position/rotation.
      * Not sure whether it has much use, however.
      **/
    virtual void update(long time) = 0;

    /**
      * The entity that represents this obstacle in the scene.
      **/
    virtual Ogre::Entity* getEntity(void)=0;

    /**
      * Update the orientation (mainly intended for rotation and position) of the obstacle
      * to the specified orientation. Does not work for all types of obstacles (will have
      * no effect in that case).
      * Replaces previous rotation completely, not cumulative.
      **/
    virtual void updateOrientation(Ogre::Quaternion orientation) = 0;

    /**
      * Update position of the obstacle with this new position.
      * The obstacle position will be updated on the navmesh if the difference
      * in squared distance with the previous position is larger than SQUARED_DISTANCE_EPSILON.
      * Replaces old position completely, not cumulative.
      **/
    virtual void updatePosition(Ogre::Vector3 position) = 0;

    /**
      * The current position of the obstacle.
      **/
    virtual Ogre::Vector3 getPosition(void) = 0;

    /**
      * The current orientation of the obstacle.
      **/
    virtual Ogre::Quaternion getOrientation(void) = 0;

    /**
      * The minimum distance an obstacle has to be moved before the obstacle is updated on the
      * navmesh.
      **/
    static const Ogre::Real SQUARED_DISTANCE_EPSILON;

    /**
      * Delta tolerance in degrees with which a new orientation has to differ from the previous one
      * in order for the obstacle to be updated.
      **/
    static const Ogre::Real ORIENTATION_TOLERANCE_DEGREES;

protected:
    /**
      * The detour tile cache that manages this obstacle.
      **/
    OgreDetourTileCache *mDetourTileCache;

    /**
      * Scene manager on which the visual representation of this obstacle is drawn.
      **/
    Ogre::SceneManager *mSceneMgr;

};

#endif // OBSTACLE_H
