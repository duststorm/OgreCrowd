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
      * The entity that represents this character in the scene
      **/
    virtual Ogre::Entity* getEntity(void);

    /**
      * @see{Character::update(Ogre::Real)}
      **/
    virtual void update(Ogre::Real timeSinceLastFrame);

    /**
      * @see{Character::setDebugVisibility(bool)}
      **/
    virtual void setDebugVisibility(bool visible);

protected:
    /**
      * Main entity that represents this character.
      **/
    Ogre::Entity *mEnt;
};

#endif // TESTCHARACTER_H
