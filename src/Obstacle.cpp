#include "Obstacle.h"

Obstacle::Obstacle(OgreDetourTileCache *detourTileCache)
    : mDetourTileCache(detourTileCache),
      mSceneMgr(detourTileCache->m_recast->m_pSceneMgr)
{

}

Obstacle::~Obstacle()
{
}
