#ifndef TESTCHARACTER_H
#define TESTCHARACTER_H

#include "Character.h"
#include <Ogre.h>

/**
  * Simple representation of an agent.
  * Agents are represented as blue cylinders.
  **/
class TestCharacter : public Character
{
public:
    TestCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    virtual void update(Ogre::Real timeSinceLastFrame);
};

#endif // TESTCHARACTER_H
