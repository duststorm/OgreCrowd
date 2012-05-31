#ifndef ANIMATEABLECHARACTER_H
#define ANIMATEABLECHARACTER_H

#include "Character.h"

class AnimateableCharacter : public Character
{
public:
    AnimateableCharacter(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    virtual void update(Ogre::Real timeSinceLastFrame);

protected:
    /**
      * Currently active animation state.
      **/
    Ogre::AnimationState* mAnimState;

    /**
      * Speed scaling factor of the animation playback.
      **/
    Ogre::Real mAnimSpeedScale;
};

#endif // ANIMATEABLECHARACTER_H
