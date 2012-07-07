#ifndef TESTCHARACTER_H
#define TESTCHARACTER_H

#include "Character.h"
#include <Ogre.h>

/**
  * Simple representation of an agent. This is the most simple way to show and debug
  * detour crowd agents.
  * Agents are represented as blue cylinders.
  **/
class TestCharacter : public Character
{
public:
    /**
      * Create a simple test character, the entities will be placed in the specified scene manager.
      * detourCrowd is the crowd manager in which an agent for this character will be created (make sure you don't create
      * more characters than MAX_AGENTS).
      * Position defines initial position the character has to be placed on (should be a valid position on the navmesh).
      **/
    TestCharacter(Ogre::String name, Ogre::SceneManager *sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    /**
      * @see{Character::update(Ogre::Real)}
      **/
    virtual void update(Ogre::Real timeSinceLastFrame);

    /**
      * @see{Character::setDebugVisibility(bool)}
      **/
    virtual void setDebugVisibility(bool visible);
};

#endif // TESTCHARACTER_H
