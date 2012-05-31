#ifndef CHARACTER_H
#define CHARACTER_H

#include <Ogre.h>
#include "OgreDetourCrowd.h"
#include "DetourCrowd/DetourCrowd.h"

class OgreRecastApplication;    //Advance declaration

/**
  * An animeatable character that acts as crowd agent
  **/
class Character
{
public:
    Character(Ogre::String name, Ogre::SceneManager* sceneMgr, OgreDetourCrowd* detourCrowd, Ogre::Vector3 position = Ogre::Vector3::ZERO);

    /**
      * The delta offset an agent must be from its destination before considering the destination reached.
      * Set this as the squared value of the actual delta value (squared distance is calculated for perfocmance).
      **/
    static const Ogre::Real DESTINATION_RADIUS;

    /**
      * The scenenode this character is attached to
      **/
    virtual Ogre::SceneNode* getNode(void);

    /**
      * The entity that represents this character in the scene
      **/
    virtual Ogre::Entity* getEntity(void);

    /**
      * Update this character for drawing a new frame
      **/
    virtual void update(Ogre::Real timeSinceLastFrame) = 0;

    /**
      * Update the destination for this agent.
      * If updatePreviousPath is set to true the previous path will be reused instead
      * of calculating a completely new path, but this can only be used if the new
      * destination is close to the previous (eg. when chasing a moving entity).
      **/
    virtual void updateDestination(Ogre::Vector3 destination, bool updatePreviousPath= false);

    /**
     * The destination set for this character.
     **/
    virtual Ogre::Vector3 getDestination(void);

    /**
      * Place character at new position.
      * The character will start following the globally set destination in the detourCrowd,
      * unless you give it an individual destination using updateDestination().
      **/
    virtual void setPosition(Ogre::Vector3 position);

    /**
      * The current position of the agent.
      * Is only up to date once update() has been called in a frame.
      **/
    virtual Ogre::Vector3 getPosition(void);

    /**
      * Index ID identifying the agent of this character in the crowd
      **/
    virtual int getAgentID();

    /**
      * The agent that steers this character within the crowd
      **/
    virtual const dtCrowdAgent* getAgent();

    /**
      * Returns true when this character has reached its set destination.
      **/
    virtual bool destinationReached(void);

protected:
    /**
      * Update current position of this character to the current position of its agent.
      **/
    virtual void updatePosition(void);

    /**
      * Set destination member variable directly without updating the agent state.
      **/
    void setDestination(Ogre::Vector3 destination);

    /**
      * Scene manager that manages this character.
     **/
    Ogre::SceneManager *mSceneMgr;

    /**
      * Main entity that represents this character.
      **/
    Ogre::Entity *mEnt;

    /**
      * Node in which this character is.
      **/
    Ogre::SceneNode *mNode;

    /**
      * Name of this character.
      **/
    Ogre::String mName;

    /**
      * Crowd in which the agent of this character is.
      **/
    OgreDetourCrowd *mDetourCrowd;

    /**
      * The agent controlling this character.
      **/
    const dtCrowdAgent *mAgent;

    /**
      * ID of mAgent within the crowd.
      **/
    int mAgentID;

    /**
      * The current destination set for this agent.
      * Take care in properly setting this variable, as it is only updated properly when
      * using Character::updateDestination() to set an individual destination for this character.
      * After creation of a new character, or when updating the destination of all agents this
      * variable should be set externally using setDestination().
      **/
    Ogre::Vector3 mDestination;


    // Friend the application class to allow setDestinationForAllAgents to update character destination values
    friend class OgreRecastApplication;
};

#endif // CHARACTER_H
