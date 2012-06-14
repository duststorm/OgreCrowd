#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "OgreDetourTileCache.h"

class Obstacle
{
public:
    Obstacle(OgreDetourTileCache *detourTileCache);
    virtual ~Obstacle();

    virtual void update(long time) = 0;

protected:
    OgreDetourTileCache *mDetourTileCache;
    Ogre::SceneManager *mSceneMgr;

};

#endif // OBSTACLE_H
