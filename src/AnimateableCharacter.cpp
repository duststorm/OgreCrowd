#include "AnimateableCharacter.h"
#include "OgreRecastApplication.h"

AnimateableCharacter::AnimateableCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, bool debugDraw, Ogre::Vector3 position)
    : Character(name, sceneMgr, detourCrowd, position),
    mAnimState(NULL),
    mAnimSpeedScale(1),
    mDebugNode(NULL)
{
    mNode = sceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
    mEnt = sceneMgr->createEntity(name, "Gamechar-male.mesh");

    // Assign random texture
    int i = (int)Ogre::Math::RangeRandom(0,14);
    if (i > 13)
        i = 13;
    mEnt->setMaterialName("GameChar_Male_Mat_"+Ogre::StringConverter::toString(i));

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
    //mNode->setScale((agentRadius*2)/bBoxSize.y, agentHeight/bBoxSize.y, (agentRadius*2)/bBoxSize.y);
    Ogre::Real scale = agentHeight/bBoxSize.y;
    mNode->setScale(scale, scale, scale);

    // Set animation speed scaling
    mAnimSpeedScale = 0.35*(scale*4);


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
    mDebugNode->setVisible(debugDraw);
}

void AnimateableCharacter::update(Ogre::Real timeSinceLastFrame)
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
            Ogre::Vector3 src = mNode->getOrientation() * -(Ogre::Vector3::UNIT_Z);    // Character looks in negative Z direction
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

void AnimateableCharacter::setDebugVisibility(bool visible)
{
    mDebugNode->setVisible(visible);
}
