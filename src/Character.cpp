/*
    OgreCrowd
    ---------

    Copyright (c) 2012 Jonas Hauquier

    Additional contributions by:

    - mkultra333
    - Paul Wilson

    Sincere thanks and to:

    - Mikko Mononen (developer of Recast navigation libraries)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#include "Character.h"
#include "OgreRecastApplication.h"

// Remember: this value should be squared and should be strictly >0 !
const Ogre::Real Character::DESTINATION_RADIUS = 1 * 1;
    // TODO it's also possible to calculate this relative to the agent radius


Character::Character(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position)
    : mName(name),
    mSceneMgr(sceneMgr),
    mDetourCrowd(detourCrowd),
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
    mRaySceneQuery(0),
    mLookingDirection(Ogre::Vector3::UNIT_X)
{
// TODO maybe create mNode in this consructor instead of in subclasses
    load(position);
}

Character::~Character()
{
    unLoad();
}

int Character::getAgentID()
{
    return mAgentID;
}

const dtCrowdAgent* Character::getAgent()
{
    return mAgent;
}

Ogre::SceneNode* Character::getNode(void) const
{
    return mNode;
}

Ogre::Vector3 Character::getDestination() const
{
    if (mAgentControlled && isLoaded())
        return mDestination;

    return Ogre::Vector3::ZERO;     // TODO this is not ideal
}

void Character::setPosition(Ogre::Vector3 position)
{
    if(!mAgentControlled || !isLoaded()) {
        getNode()->setPosition(position);
        return;
    }

    // Find position on navmesh
    if (!mDetourCrowd->m_recast->findNearestPointOnNavmesh(position, position))
        return;

    // Remove agent from crowd and re-add at position
    mDetourCrowd->removeAgent(mAgentID);
    mAgentID = mDetourCrowd->addAgent(position);
    mAgent = mDetourCrowd->getAgent(mAgentID);

    if(getNode())   // TODO remove this check by initing mNode in this class' constructor
        getNode()->setPosition(position);
}

void Character::updateDestination(Ogre::Vector3 destination, bool updatePreviousPath)
{
    if(!mAgentControlled || !isLoaded())
        return;

    // Find position on navmesh
    if(!mDetourCrowd->m_recast->findNearestPointOnNavmesh(destination, destination))
        return;

    mDetourCrowd->setMoveTarget(mAgentID, destination, updatePreviousPath);
    mDestination = destination;
    mStopped = false;
    mManualVelocity = Ogre::Vector3::ZERO;
}

Ogre::Vector3 Character::getPosition() const
{
    return getNode()->getPosition();
}

void Character::updatePosition(Ogre::Real timeSinceLastFrame)
{
    if(!isLoaded())
        return;

    if (mAgentControlled) {
        if(getAgent()->active) {
            Ogre::Vector3 agentPos;
            OgreRecast::FloatAToOgreVect3(getAgent()->npos, agentPos);

            getNode()->setPosition(agentPos);
        }
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
    if(!isLoaded())
        return false;

    if(getPosition().squaredDistance(getDestination()) <= Character::DESTINATION_RADIUS)
        return true;


    return mDetourCrowd->destinationReached(getAgent(), Character::DESTINATION_RADIUS);
}

void Character::setDestination(Ogre::Vector3 destination)
{
    if (!mAgentControlled || !isLoaded())
        return;

    mDestination = destination;
    mManualVelocity = Ogre::Vector3::ZERO;
    mStopped = false;
}

void Character::stop()
{
    if(!mAgentControlled || !isLoaded()) {
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
    return mNode->getOrientation() * getRelativeLookingDirection();
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

    if(mAgentControlled && isLoaded())
        mDetourCrowd->requestVelocity(getAgentID(), mManualVelocity);
}

Ogre::Vector3 Character::getVelocity()
{
    if(!isLoaded())
        return Ogre::Vector3::ZERO;

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
    if(isLoaded())
        return getAgent()->params.maxSpeed;
    else
        return 0.0f;
}

Ogre::Real Character::getMaxAcceleration()
{
    if(isLoaded())
        return getAgent()->params.maxAcceleration;
    else
        return 0.0f;
}

bool Character::isMoving()
{
    return !mStopped || getSpeed() != 0;
}

Ogre::Real Character::getAgentHeight(void) const
{
    return mDetourCrowd->getAgentHeight();
}

Ogre::Real Character::getAgentRadius(void) const
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

void Character::load()
{
    if(isLoaded())
        return; // nothing to do

    load(getPosition());
}

void Character::load(Ogre::Vector3 position)
{
    if(isLoaded()) {
        setPosition(position);
    } else {
        mAgentID = mDetourCrowd->addAgent(position);
        mAgent = mDetourCrowd->getAgent(mAgentID);
    }

    setPosition(position);
    show();
}

void Character::unLoad()
{
    mDetourCrowd->removeAgent(getAgentID());
    mAgentID = -1;
    mAgent = NULL;

    hide();
}

void Character::show()
{
    if(getNode()) {
        getNode()->setVisible(true);
    }
}

void Character::hide()
{
    if(getNode())
        getNode()->setVisible(false);
}

Ogre::Vector3 Character::getRelativeLookingDirection()
{
    return mLookingDirection;
}

void Character::setRelativeLookingDirection(Ogre::Vector3 direction)
{
    mLookingDirection = direction;
}
