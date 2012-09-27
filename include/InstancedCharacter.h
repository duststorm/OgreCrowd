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

#ifndef INSTANCEDCHARACTER_H
#define INSTANCEDCHARACTER_H

#include <OgrePrerequisites.h>
#if OGRE_VERSION_MINOR >= 8
#include "Character.h"

class InstancedCharacter : public Character
{
public:
    InstancedCharacter(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::InstanceManager* instanceMgr, bool debugDraw = false, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    /**
      * Update one tick in the render loop. Advances animation and character position.
      * In order for the agents to be updated, you first need to call the detourCrowd
      * update function.
      **/
    virtual void update(Ogre::Real timeSinceLastFrame);

    /**
      * The instanced entity that represents this character in the scene
      **/
    virtual Ogre::InstancedEntity* getEntity(void);

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
    Ogre::InstancedEntity *mEnt;

    Ogre::InstanceManager* mInstanceManager;

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
#endif

#endif // INSTANCEDCHARACTER_H
