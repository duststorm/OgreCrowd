#ifndef CYLINDEROBSTACLE_H
#define CYLINDEROBSTACLE_H

#include "Obstacle.h"

class CylinderObstacle : public Obstacle
{
public:
    CylinderObstacle(Ogre::Vector3 position, OgreDetourTileCache *detourTileCache);
    ~CylinderObstacle();

    virtual void update(long time);

    virtual dtObstacleRef getObstacleRef(void);

    virtual Ogre::Entity* getEntity(void);

protected:
    Ogre::Vector3 mPosition;
    dtObstacleRef mObstacleRef;

    Ogre::Entity *mEnt;
    Ogre::SceneNode *mNode;
    Ogre::String mName;
};

#endif // CYLINDEROBSTACLE_H
