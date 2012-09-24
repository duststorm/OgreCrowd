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

#ifndef ANIMATEABLECHARACTER_H
#define ANIMATEABLECHARACTER_H

#include "Character.h"

/**
  * More complex character that shows an animated walking human to represent an agent.
  * This is probably closer to what you would use in a real scenario. This class can just
  * be replaced with your character class if you already have one and it mainly serves
  * as an example of how characters can be integrated with detour agents.
  **/
class AnimateableCharacter : public Character
{
public:
    /**
      * Create a new human character with specified name, the entities will be placed in the specified scene manager.
      * detourCrowd is the crowd manager in which an agent for this character will be created (make sure you don't create
      * more characters than MAX_AGENTS).
      * Set debugDraw to true to initially draw debug geometry (can be disabled afterwards too).
      * Position defines initial position the character has to be placed on (should be a valid position on the navmesh).
      **/
    AnimateableCharacter(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, bool debugDraw = false, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    /**
      * The entity that represents this character in the scene
      **/
    virtual Ogre::Entity* getEntity(void);

    /**
      * Update one tick in the render loop. Advances animation and character position.
      * In order for the agents to be updated, you first need to call the detourCrowd
      * update function.
      **/
    virtual void update(Ogre::Real timeSinceLastFrame);

    /**
      * @see{Character::setDebugVisibility(bool)}
      **/
    virtual void setDebugVisibility(bool visible);

    virtual bool getDebugVisibility(void);

    virtual void show(void);

    void randomizeAnimationPosition(void);

protected:
    bool mDebugDraw;

    /**
      * Main entity that represents this character.
      **/
    Ogre::Entity *mEnt;

    /**
      * Currently active animation state.
      **/
    Ogre::AnimationState* mAnimState;

    /**
      * Speed scaling factor of the animation playback.
      **/
    Ogre::Real mAnimSpeedScale;

    /**
      * Scenenode that stores all geometry related to
      * recast debug drawing. Can be made visible with
      * setDebugVisibility().
      **/
    Ogre::SceneNode *mDebugNode;
};

#endif // ANIMATEABLECHARACTER_H
