#include "TestCharacter.h"
#include "OgreRecastApplication.h"

TestCharacter::TestCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position)
    : Character(name, sceneMgr, detourCrowd, position)
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

void TestCharacter::update(Ogre::Real timeSinceLastFrame)
{
    updatePosition(timeSinceLastFrame);
}

void TestCharacter::setDebugVisibility(bool visible)
{
    return;
}
