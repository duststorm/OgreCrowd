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
        mEnt->setQueryFlags(OgreRecastApplication::DEFAULT_MASK);  // exclude from navmesh queries
    }

    // TODO have some way of notifying that obstacle creation failed? (exception maybe?)
}

CylinderObstacle::~CylinderObstacle()
{
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
