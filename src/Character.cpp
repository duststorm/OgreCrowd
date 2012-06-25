#include "Character.h"
#include "OgreRecastApplication.h"

// Remember: this value should be squared and should be strictly >0 !
const Ogre::Real Character::DESTINATION_RADIUS = 0.1 * 0.1;
    // TODO it's also possible to calculate this relative to the agent radius


Character::Character(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position)
    : mName(name),
    mSceneMgr(sceneMgr),
    mDetourCrowd(detourCrowd),
    mEnt(NULL),
    mNode(NULL),
    mAgent(NULL),
    mAgentID(-1),
    mManualVelocity(Ogre::Vector3::ZERO),
    mDestination(detourCrowd->getLastDestination()),
    mStopped(false),
    mAgentControlled(true),
    mDetourTileCache(NULL),
    mTempObstacle(0),
    mClipTo(0),
    mRaySceneQuery(0)
{
    mAgentID = mDetourCrowd->addAgent(position);
    mAgent = mDetourCrowd->getAgent(mAgentID);
}

int Character::getAgentID()
{
    return mAgentID;
}

const dtCrowdAgent* Character::getAgent()
{
    return mAgent;
}

Ogre::Entity* Character::getEntity()
{
    return mEnt;
}

Ogre::SceneNode* Character::getNode()
{
    return mNode;
}

Ogre::Vector3 Character::getDestination()
{
    if (mAgentControlled)
        return mDestination;

    return Ogre::Vector3::ZERO;     // TODO this is not ideal
}

void Character::setPosition(Ogre::Vector3 position)
{
    if(!mAgentControlled) {
        getNode()->setPosition(position);
        return;
    }

    // Find position on navmesh
    if (!mDetourCrowd->m_recast->findNearestPointOnNavmesh(position, position))
        return;

    // Remove agent from crowd and re-add at position
    mDetourCrowd->removeAgent(mAgentID);
    mAgentID = mDetourCrowd->addAgent(position);

    getNode()->setPosition(position);
}

void Character::updateDestination(Ogre::Vector3 destination, bool updatePreviousPath)
{
    if(!mAgentControlled)
        return;

    // Find position on navmesh
    if(!mDetourCrowd->m_recast->findNearestPointOnNavmesh(destination, destination))
        return;

    mDetourCrowd->setMoveTarget(mAgentID, destination, updatePreviousPath);
    mDestination = destination;
    mStopped = false;
    mManualVelocity = Ogre::Vector3::ZERO;
}

Ogre::Vector3 Character::getPosition()
{
    return getNode()->getPosition();
}

void Character::updatePosition(Ogre::Real timeSinceLastFrame)
{
    if (mAgentControlled) {
        Ogre::Vector3 agentPos;
        OgreRecast::FloatAToOgreVect3(getAgent()->npos, agentPos);

        getNode()->setPosition(agentPos);
    } else {
        // Move character manually to new position
        if(getVelocity().isZeroLength())
            return;

        // Make other agents avoid first character by placing a temporary obstacle in its position
        mDetourTileCache->removeTempObstacle(mTempObstacle);   // Remove old obstacle
        getNode()->setPosition(getPosition() + timeSinceLastFrame * getVelocity());
        // TODO check whether this position is within navmesh
        mTempObstacle = mDetourTileCache->addTempObstacle(getPosition());   // Add new obstacle
    }

    // Clip position to terrain height
    if (mClipTo)
        clipToTerrainHeight();
}


void Character::clipToTerrain(Ogre::TerrainGroup *terrainGroup)
{
    mClipTo = terrainGroup;
}


void Character::clipToTerrainHeight()
{
    // Setup the scene query
    Ogre::Ray queryRay(getNode()->getPosition(), Ogre::Vector3::NEGATIVE_UNIT_Y);

    // Perform the scene query
    Ogre::TerrainGroup::RayResult result = mClipTo->rayIntersects(queryRay);
    if(result.hit) {
        Ogre::Real terrainHeight = result.position.y;

        Ogre::Vector3 pos = getNode()->getPosition();
        pos.y = terrainHeight;
        getNode()->setPosition(pos);
    } else {
        // Try querying terrain above character
        queryRay.setOrigin(getNode()->getPosition());
        queryRay.setDirection(Ogre::Vector3::UNIT_Y);

        // Perform scene query again
        result = mClipTo->rayIntersects(queryRay);
        if(result.hit) {
            Ogre::Real terrainHeight = result.position.y;

            Ogre::Vector3 pos = getNode()->getPosition();
            pos.y = terrainHeight;
            getNode()->setPosition(pos);
        }
    }
}


bool Character::destinationReached()
{
    Ogre::Vector3 pos = getPosition();
    Ogre::Vector3 dest = getDestination();
    Ogre::Real dist = pos.squaredDistance(dest);
    Ogre::Real rad = DESTINATION_RADIUS;
    bool res = dist < DESTINATION_RADIUS;
    return (getPosition().squaredDistance(getDestination()) <= Character::DESTINATION_RADIUS);
}

void Character::setDestination(Ogre::Vector3 destination)
{
    if (!mAgentControlled)
        return;

    mDestination = destination;
    mManualVelocity = Ogre::Vector3::ZERO;
    mStopped = false;
}

void Character::stop()
{
    if(!mAgentControlled) {
        mManualVelocity = Ogre::Vector3::ZERO;
        mStopped = true;
        return;
    }

    if(mDetourCrowd->stopAgent(getAgentID())) {
        mDestination = Ogre::Vector3::ZERO;     // TODO this is not ideal
        mManualVelocity = Ogre::Vector3::ZERO;
        mStopped = true;
    }
}

Ogre::Vector3 Character::getLookingDirection()
{
    return mNode->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;    // Character looks in negative Z direction
}

void Character::moveForward()
{
    Ogre::Vector3 lookDirection = getLookingDirection();
    lookDirection.normalise();

    setVelocity(getMaxSpeed() * lookDirection);
}

void Character::setVelocity(Ogre::Vector3 velocity)
{
    mManualVelocity = velocity;
    mStopped = false;
    mDestination = Ogre::Vector3::ZERO;     // TODO this is not ideal

    if(mAgentControlled)
        mDetourCrowd->requestVelocity(getAgentID(), mManualVelocity);
}

Ogre::Vector3 Character::getVelocity()
{
    if(mAgentControlled) {
        Ogre::Vector3 velocity;
        OgreRecast::FloatAToOgreVect3(getAgent()->nvel, velocity);
        return velocity;
    } else {
        return mManualVelocity;
    }
}

Ogre::Real Character::getSpeed()
{
    return getVelocity().length();
}

Ogre::Real Character::getMaxSpeed()
{
    return getAgent()->params.maxSpeed;
}

Ogre::Real Character::getMaxAcceleration()
{
    return getAgent()->params.maxAcceleration;
}

bool Character::isMoving()
{
    return !mStopped || getSpeed() != 0;
}

Ogre::Real Character::getAgentHeight(void)
{
    return mDetourCrowd->getAgentHeight();
}

Ogre::Real Character::getAgentRadius(void)
{
    return mDetourCrowd->getAgentRadius();
}

void Character::setAgentControlled(bool agentControlled)
{
    if (mAgentControlled != agentControlled) {
        if (agentControlled) {
            if(mTempObstacle)
                mDetourTileCache->removeTempObstacle(mTempObstacle);
            mTempObstacle = 0;
            mAgentID = mDetourCrowd->addAgent(getPosition());
            mDestination = mDetourCrowd->getLastDestination();
            mManualVelocity = Ogre::Vector3::ZERO;
            mStopped = true;
        } else {
            mTempObstacle = mDetourTileCache->addTempObstacle(getPosition());   // Add temp obstacle at current position
            mDetourCrowd->removeAgent(mAgentID);
            mAgentID = -1;
            mDestination = Ogre::Vector3::ZERO;     // TODO this is not ideal
            mStopped = false;
        }
        mAgentControlled = agentControlled;
    }
}

bool Character::isAgentControlled()
{
    return mAgentControlled;
}

void Character::setDetourTileCache(OgreDetourTileCache *dtTileCache)
{
    mDetourTileCache = dtTileCache;
}
