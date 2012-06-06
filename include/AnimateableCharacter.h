#ifndef ANIMATEABLECHARACTER_H
#define ANIMATEABLECHARACTER_H

#include "Character.h"

class AnimateableCharacter : public Character
{
public:
    AnimateableCharacter(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, bool debugDraw = false, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    virtual void update(Ogre::Real timeSinceLastFrame);

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
