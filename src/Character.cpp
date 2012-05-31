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
    mDestination(position)
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
    // TODO updating individual also affects latest set destination in OgreDetourCrowd. Fix this there instead of hacking it in the application class.
    mDetourCrowd->setMoveTarget(mAgentID, destination, updatePreviousPath);
    mDestination = destination;
}

Ogre::Vector3 Character::getPosition()
{
    if(OgreRecastApplication::DEBUG_DRAW)
        return getNode()->getPosition() - (mDetourCrowd->m_recastDemo->m_navMeshOffsetFromGround * Ogre::Vector3::UNIT_Y);
    else
        return getNode()->getPosition();
}

void Character::updatePosition()
{
    Ogre::Vector3 agentPos;
    mDetourCrowd->m_recastDemo->FloatAToOgreVect3(getAgent()->npos, agentPos);

    if(OgreRecastApplication::DEBUG_DRAW)
        agentPos.y = agentPos.y + mDetourCrowd->m_recastDemo->m_navMeshOffsetFromGround; // Compensate for distance of navmesh above ground

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
}
