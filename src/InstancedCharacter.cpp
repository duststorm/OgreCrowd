#include "InstancedCharacter.h"
#include "OgreRecastApplication.h"  // TODO remove this dependency

InstancedCharacter::InstancedCharacter(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::InstanceManager* instanceMgr, bool debugDraw, Ogre::Vector3 position)
    : Character(name, sceneMgr, detourCrowd, position),
    mAnimState(NULL),
    mAnimSpeedScale(1),
    mDebugNode(NULL),
    mEnt(NULL),
    mDebugDraw(debugDraw),
    mInstanceManager(instanceMgr)
{
    mNode = sceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
    mEnt = mInstanceManager->createInstancedEntity("Examples/Instancing/ShaderBased/Robot");

    // Set looking direction for robot model
    setRelativeLookingDirection( Ogre::Vector3::UNIT_X );

    mEnt->setQueryFlags(OgreRecastApplication::DEFAULT_MASK);   // Exclude from ray queries

    mNode->attachObject(mEnt);
    mNode->setPosition(position);

    // Assign animation
    mAnimState= mEnt->getAnimationState("Walk");
    mAnimState->setEnabled(true);
    mAnimState->setLoop(true);

    Ogre::Vector3 bBoxSize = mEnt->getBoundingBox().getSize();

    Ogre::Real agentRadius = mDetourCrowd->getAgentRadius();
    Ogre::Real agentHeight = mDetourCrowd->getAgentHeight();

    // Set Height to match that of agent
    Ogre::Real scale = agentHeight/bBoxSize.y;
    mNode->setScale(scale, scale, scale);

    // Set animation speed scaling
    mAnimSpeedScale = 1;


    // Debug draw agent
    mDebugNode = mNode->createChildSceneNode(name+"AgentDebugNode");
    mDebugNode->setPosition(0, mDetourCrowd->m_recast->getNavmeshOffsetFromGround(), 0);
    Ogre::Entity* debugEnt = sceneMgr->createEntity(name+"AgentDebug", "Cylinder.mesh");
    debugEnt->setMaterialName("Cylinder/Wires/LightBlue");
    mDebugNode->attachObject(debugEnt);
    // Set marker scale to size of agent
    mDebugNode->setInheritScale(false);
    mDebugNode->setScale(agentRadius*2, agentHeight, agentRadius*2);
    debugEnt->setQueryFlags(OgreRecastApplication::DEFAULT_MASK);   // Exclude from ray queries
    mDebugNode->setVisible(mDebugDraw);
}

void InstancedCharacter::update(Ogre::Real timeSinceLastFrame)
{
    updatePosition(timeSinceLastFrame);

    if (mClipTo)
        clipToTerrainHeight();

    Ogre::Vector3 velocity = getVelocity(); // Current velocity of agent
    Ogre::Real speed = velocity.length();

    if(speed > 0.15) {
        mAnimState->setEnabled(true);
        mAnimState->addTime(mAnimSpeedScale * speed * timeSinceLastFrame);

        if(speed > 0/*0.8*/) {  // Avoid jitter (TODO keep this?)
            // Rotate to look in walking direction
            Ogre::Vector3 src = getLookingDirection();
            src.y = 0;  // Ignore y direction

            velocity.y = 0;
            velocity.normalise();
            mNode->rotate(src.getRotationTo(velocity));
        }
    } else {    // Assume character has stopped
        mAnimState->setEnabled(false);
        mAnimState->setTimePosition(0);
    }
}

Ogre::InstancedEntity* InstancedCharacter::getEntity()
{
    return mEnt;
}

void InstancedCharacter ::setDebugVisibility(bool visible)
{
    mDebugNode->setVisible(visible);
}

bool InstancedCharacter::getDebugVisibility()
{
    return mDebugDraw;
}

void InstancedCharacter::show()
{
    Character::show();

    mDebugNode->setVisible(getDebugVisibility());
}
