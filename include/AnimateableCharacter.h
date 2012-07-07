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
      * Update one tick in the render loop. Advances animation and character position.
      * In order for the agents to be updated, you first need to call the detourCrowd
      * update function.
      **/
    virtual void update(Ogre::Real timeSinceLastFrame);

    /**
      * @see{Character::setDebugVisibility(bool)}
      **/
    virtual void setDebugVisibility(bool visible);

protected:
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
