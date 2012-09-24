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

#include "AnimateableCharacter.h"
#include "OgreRecastApplication.h"

AnimateableCharacter::AnimateableCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, bool debugDraw, Ogre::Vector3 position)
    : Character(name, sceneMgr, detourCrowd, position),
    mAnimState(NULL),
    mEnt(NULL),
    mAnimSpeedScale(1),
    mDebugNode(NULL),
    mDebugDraw(debugDraw)
{
    mNode = sceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
    mEnt = sceneMgr->createEntity(name, "Gamechar-male.mesh");

    // Set looking direction for this model
    setRelativeLookingDirection( -Ogre::Vector3::UNIT_Z );

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
    mDebugNode->setVisible(mDebugDraw);
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

Ogre::Entity* AnimateableCharacter::getEntity()
{
    return mEnt;
}

void AnimateableCharacter::setDebugVisibility(bool visible)
{
    mDebugDraw = visible;
    mDebugNode->setVisible(mDebugDraw);
}

bool AnimateableCharacter::getDebugVisibility()
{
    return mDebugDraw;
}

void AnimateableCharacter::show()
{
    Character::show();

    mDebugNode->setVisible(getDebugVisibility());
}

void AnimateableCharacter::randomizeAnimationPosition()
{
    mAnimState->setTimePosition( Ogre::Math::RangeRandom(0, mAnimState->getLength()) );
}
