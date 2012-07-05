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
