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
#include <OgrePrerequisites.h>
#if OGRE_VERSION_MINOR >= 8

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

    // Assign random texture
    int i = (int)Ogre::Math::RangeRandom(0,14);
    if (i > 13)
        i = 13;
    mEnt = mInstanceManager->createInstancedEntity("Examples/Instancing/ShaderBased/Male_"+Ogre::StringConverter::toString(i));

    // Set looking direction for model
    setRelativeLookingDirection( - Ogre::Vector3::UNIT_Z );

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
    mAnimSpeedScale = 0.2;


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
                // TODO average direction over multiple velocity samples
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

void InstancedCharacter::randomizeAnimationPosition()
{
    mAnimState->setTimePosition( Ogre::Math::RangeRandom(0, mAnimState->getLength()) );
}

#endif
