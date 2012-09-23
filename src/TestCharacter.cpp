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

#include "TestCharacter.h"
#include "OgreRecastApplication.h"

TestCharacter::TestCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position)
    : Character(name, sceneMgr, detourCrowd, position),
    mEnt(NULL)
{
    // Depict agent as blue cylinder
    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
    mEnt = mSceneMgr->createEntity(name, "Cylinder.mesh");
    mEnt->setMaterialName("Cylinder/Blue");
    mNode->attachObject(mEnt);
    mNode->setPosition(position);

    // Set marker scale to size of agent
    mNode->setScale(detourCrowd->getAgentRadius()*2, detourCrowd->getAgentHeight(), detourCrowd->getAgentRadius()*2);

    mEnt->setQueryFlags(OgreRecastApplication::DEFAULT_MASK);   // Exclude from ray queries
}

Ogre::Entity* TestCharacter::getEntity()
{
    return mEnt;
}

void TestCharacter::update(Ogre::Real timeSinceLastFrame)
{
    updatePosition(timeSinceLastFrame);
}

void TestCharacter::setDebugVisibility(bool visible)
{
    return;
}
