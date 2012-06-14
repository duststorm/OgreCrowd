#ifndef CONVEXSHAPEOBSTACLE_H
#define CONVEXSHAPEOBSTACLE_H

#include "Obstacle.h"
#include "RecastConvexHull.h"

class ConvexShapeObstacle : public Obstacle
{
public:
    ConvexShapeObstacle(Ogre::Vector3 position, Ogre::Real offset, OgreDetourTileCache *detourTileCache);
    virtual ~ConvexShapeObstacle(); // Important that the destructor is virtual!

    virtual void update(long time);

protected:
    Ogre::Vector3 mPosition;

    Ogre::Entity *mEnt;
    Ogre::SceneNode *mNode;
    Ogre::String mName;

    ConvexVolume *mConvexHull;
    InputGeom *mInputGeom;   // TODO or create pointer
    int mObstacleId;
    Ogre::ManualObject *mConvexHullDebug;
};

#endif // CONVEXSHAPEOBSTACLE_H
