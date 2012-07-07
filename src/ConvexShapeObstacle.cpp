#include "ConvexShapeObstacle.h"
#include "OgreRecastApplication.h"

ConvexShapeObstacle::ConvexShapeObstacle(Ogre::Vector3 position, Ogre::Real offset, OgreDetourTileCache *detourTileCache)
    : Obstacle(detourTileCache),
      mPosition(position),
      mEnt(0),
      mNode(0),
      mConvexHullDebug(0),
      mInputGeom(0),
      mOffset(offset)
{
    // Randomly place a box or a pot as obstacle
    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    if (Ogre::Math::RangeRandom(0,2) < 1.5) {
        mEnt = mSceneMgr->createEntity("Box.mesh");
    } else {
        // For more complex entities, convex hull building will consider only MAX_CONVEXVOL_PTS vertices and the result can be sub-optimal.
        // A more robust convex hull building algorithm might be preferred.
        mEnt = mSceneMgr->createEntity("Pot.mesh");
        mNode->setScale(0.3, 0.3, 0.3);
    }
    mNode->attachObject(mEnt);
    mNode->setPosition(mPosition);

    mOrientation = mNode->getOrientation();

    mEnt->setQueryFlags(OgreRecastApplication::OBSTACLE_MASK);  // add to query group for obstacles

    // Transfer entitiy geometry to recast compatible format
    if(mEnt->getMesh()->getName() == "Pot.mesh") {
        // Create a convex hull from the mesh geometry

        // Note that it is important to first add your entity to the scene before creating an inputGeom from it.
        // This is so that it can calculate the world space coordinates for the object, which are needed for recast.
        mInputGeom = new InputGeom(mEnt);

        // Create convex  obstacle in the detourTileCache
        // Create convex hull with agent radios offset around the object (this is important so agents don't walk through the edges of the obstacle!)
        mConvexHull = mInputGeom->getConvexHull(mOffset);
    } else {
        // Create a convex hull simply from the bounding box

        // The bounding box has to be transformed into world-space coordinates
        mConvexHull = new ConvexVolume(InputGeom::getWorldSpaceBoundingBox(mEnt), offset);
    }

    // WARNING: Watch out for memory leaks here! ConvexVolume objects are not managed by any system (except this class).
    mConvexHull->area = RC_NULL_AREA;   // Set area described by convex polygon to "unwalkable"
    // Add convex hull to detourTileCache as obstacle
    mDetourTileCache->addConvexShapeObstacle(mConvexHull);

    // Debug draw convex hull
// TODO add debug flag, grey lines around boxes should disappear when disabling debug drawing
    mConvexHullDebug = InputGeom::drawConvexVolume(mConvexHull, mSceneMgr);    // Debug convex volume

    //if(mObstacleId == -1)
        // TODO exception when something goes wrong!

    //mEnt->setVisible(false);       // TODO maybe make boxes semi-transparent in debug draw mode?
}

ConvexShapeObstacle::~ConvexShapeObstacle()
{
    // Remove obstacle from DetourTileCache
    mDetourTileCache->removeConvexShapeObstacle(mConvexHull);

    mNode->removeAllChildren();
    mNode->getParentSceneNode()->removeChild(mNode);
    mSceneMgr->destroyEntity(mEnt);
    mSceneMgr->destroySceneNode(mNode);

    mNode = NULL;
    mEnt = NULL;

    mConvexHullDebug->detachFromParent();
    mSceneMgr->destroyManualObject(mConvexHullDebug);

    delete mInputGeom;

    mConvexHullDebug = NULL;
}


void ConvexShapeObstacle::update(long time)
{
}


Ogre::Entity* ConvexShapeObstacle::getEntity()
{
    return mEnt;
}


void ConvexShapeObstacle::updatePosition(Ogre::Vector3 position)
{
    // Modify position if larger than epsilon
    if ( mPosition.squaredDistance(position) > SQUARED_DISTANCE_EPSILON ) {
        // Remove obstacle
        mDetourTileCache->removeConvexShapeObstacle(mConvexHull);

        // Transform hull to new location
        mConvexHull->move(position-mPosition);

        // Re-add hull as obstacle at new location
        mDetourTileCache->addConvexShapeObstacle(mConvexHull);

        mPosition = position;


        // Now also set the position to the visual obstacle entity
        mNode->setPosition(position);

        // Create new debug drawing of the convex hull
        mConvexHullDebug->detachFromParent();
        mSceneMgr->destroyManualObject(mConvexHullDebug);
        mConvexHullDebug = InputGeom::drawConvexVolume(mConvexHull, mSceneMgr);
    }
}

Ogre::Vector3 ConvexShapeObstacle::getPosition()
{
    return mPosition;
}

void ConvexShapeObstacle::updateOrientation(Ogre::Quaternion orientation)
{
    // Modify orientation if difference larger than epsilon
    if(! mOrientation.equals(orientation, Ogre::Degree(ORIENTATION_TOLERANCE_DEGREES))) {
        // Remove old obstacle from tilecache
        mDetourTileCache->removeConvexShapeObstacle(mConvexHull);

        // In case we didn't generate inputgeom yet (we previously generated hull directly from the bounding box), do it now
        if(!mInputGeom)
            mInputGeom = new InputGeom(mEnt);

        // Apply rotation to the inputGeometry and calculate a new 2D convex hull
        Ogre::Quaternion relativeOrientation = orientation * mOrientation.Inverse();    // Calculate relative rotation from current rotation to the specified one
        orientation.normalise();    // Make sure quaternion is normalized
        mInputGeom->applyOrientation(relativeOrientation, mPosition);   // Rotate around obstacle position (center or origin point)
        if (mConvexHull)
            delete mConvexHull;
        mConvexHull = mInputGeom->getConvexHull(mOffset);
        mConvexHull->area = RC_NULL_AREA;   // Be sure to set the proper area for the convex shape!

        // Add new hull as obstacle to tilecache
        mDetourTileCache->addConvexShapeObstacle(mConvexHull);

        mOrientation = orientation;


        // Now also set the rotation to the visual obstacle entity
        mNode->setOrientation(orientation);

        // Create new debug drawing of the convex hull
        mConvexHullDebug->detachFromParent();
        mSceneMgr->destroyManualObject(mConvexHullDebug);
        mConvexHullDebug = InputGeom::drawConvexVolume(mConvexHull, mSceneMgr);
    }
}


Ogre::Quaternion ConvexShapeObstacle::getOrientation()
{
    return mOrientation;
}
