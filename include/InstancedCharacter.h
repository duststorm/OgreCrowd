#ifndef INSTANCEDCHARACTER_H
#define INSTANCEDCHARACTER_H

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

#endif // INSTANCEDCHARACTER_H
