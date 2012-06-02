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
    mStopped(false)
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
    return mDestination;
}

void Character::setPosition(Ogre::Vector3 position)
{
    // Remove agent from crowd and re-add at position
    mDetourCrowd->removeAgent(mAgentID);
    mAgentID = mDetourCrowd->addAgent(position);

    getNode()->setPosition(position);
}

void Character::updateDestination(Ogre::Vector3 destination, bool updatePreviousPath)
{
    mDetourCrowd->setMoveTarget(mAgentID, destination, updatePreviousPath);
    mDestination = destination;
    mStopped = false;
    mManualVelocity = Ogre::Vector3::ZERO;
}

Ogre::Vector3 Character::getPosition()
{
    if(OgreRecastApplication::DEBUG_DRAW)
        return getNode()->getPosition() - (mDetourCrowd->m_recast->m_navMeshOffsetFromGround * Ogre::Vector3::UNIT_Y);
    else
        return getNode()->getPosition();
}

void Character::updatePosition()
{
    Ogre::Vector3 agentPos;
    OgreRecast::FloatAToOgreVect3(getAgent()->npos, agentPos);

    if(OgreRecastApplication::DEBUG_DRAW)
        agentPos.y = agentPos.y + mDetourCrowd->m_recast->m_navMeshOffsetFromGround; // Compensate for distance of navmesh above ground

    getNode()->setPosition(agentPos);
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
    mDestination = destination;
    mManualVelocity = Ogre::Vector3::ZERO;
    mStopped = false;
}

void Character::stop()
{
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
    setVelocity(getAgent()->params.maxSpeed * lookDirection);
}

void Character::setVelocity(Ogre::Vector3 velocity)
{
    mManualVelocity = velocity;
    mStopped = false;
    mDestination = Ogre::Vector3::ZERO;     // TODO this is not ideal

    mDetourCrowd->requestVelocity(getAgentID(), mManualVelocity);
}

Ogre::Vector3 Character::getVelocity()
{
    Ogre::Vector3 velocity;
    OgreRecast::FloatAToOgreVect3(getAgent()->nvel, velocity);

    return velocity;
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
