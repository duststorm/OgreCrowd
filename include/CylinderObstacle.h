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
