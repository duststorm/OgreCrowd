#ifndef CHARACTER_H
#define CHARACTER_H

#include <Ogre.h>
#include "OgreDetourCrowd.h"
#include "DetourCrowd/DetourCrowd.h"
#include "OgreDetourTileCache.h"

class OgreRecastApplication;    //Advance declaration

/**
  * An animeatable character that acts as crowd agent
  **/
class Character
{
public:
    /**
      * Create a character with specified name, for which entities will be drawn on the specified scene manager.
      * detourCrowd specifies the detour crowd manager on which an agent for this character will be created
      * (make sure you don't create more characters than MAX_AGENTS).
      * Position defines initial position the character has to be placed on (should be a valid position on the navmesh).
      **/
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
      * The height of the agent for this character.
      **/
    virtual Ogre::Real getAgentHeight(void);

    /**
      * The radius of the agent for this character.
      **/
    virtual Ogre::Real getAgentRadius(void);

    /**
      * Update this character for drawing a new frame.
      * Updates one tick in the render loop.
      * In order for the agents to be updated, you first need to call the detourCrowd
      * update function.
      * What is updated specifically is up to a specific implementation of Character,
      * but at the very least the position in the scene should be updated to reflect
      * the detour agent position (possibly with additional physics engine clipping
      * and collision testing).
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

    /**
      * Request to set a manual velocity for this character, to control it
      * manually.
      * The set velocity will stay active, meaning that the character will
      * keep moving in the set direction, until you stop() it.
      * Manually controlling a character offers no absolute control as the
      * laws of acceleration and max speed still apply to an agent, as well
      * as the fact that it cannot steer off the navmesh or into other agents.
      * You will notice small corrections to steering when walking into objects
      * or the border of the navmesh (which is usually a wall or object).
      **/
    void setVelocity(Ogre::Vector3 velocity);

    /**
      * Manually control the character moving it forward.
      **/
    virtual void moveForward(void);

    /**
      * Stop any movement this character is currently doing. This means losing
      * the requested velocity or target destination.
      **/
    virtual void stop(void);

    /**
      * The current velocity (speed and direction) this character is traveling
      * at.
      **/
    virtual Ogre::Vector3 getVelocity(void);

    /**
      * The current speed this character is traveling at.
      **/
    virtual Ogre::Real getSpeed(void);

    /**
      * The maximum speed this character can attain.
      * This parameter is configured for the agent controlling this character.
      **/
    virtual Ogre::Real getMaxSpeed(void);

    /**
      * The maximum acceleration this character has towards its maximum speed.
      * This parameter is configured for the agent controlling this character.
      **/
    virtual Ogre::Real getMaxAcceleration(void);

    /**
      * Returns true if this character is moving.
      **/
    virtual bool isMoving(void);

    /**
      * The direction in which the character is currently looking.
      **/
    virtual Ogre::Vector3 getLookingDirection(void);

    /**
      * Set to true to show visual recast debugging geometry to represent
      * the agent of this character..
      * Will be initialized to OgreRecastApplication::getDebugDrawing()
      * at character construction.
      **/
    virtual void setDebugVisibility(bool visible) = 0;

    /**
      * Set whether this character is controlled by an agent or whether it
      * will position itself independently based on the requested velocity.
      * Set to true to let the character be controlled by an agent.
      * Set to false to manually control it without agent, you need to set
      * detourTileCache first.
      **/
    void setAgentControlled(bool agentControlled);

    /**
      * Determines whether this character is controlled by an agent.
      **/
    bool isAgentControlled(void);

    /**
      * Set detour tile cache component.
      * This is needed for controlling the agent manually without agent,
      * as it will use dtTileCache to add a temporary obstacle at its
      * current position to make other characters in the crowd avoid it.
      **/
    void setDetourTileCache(OgreDetourTileCache* dtTileCache);

    /**
      * Makes the character clip to the terrain height of the specified
      * terrain set. Specify NULL as terrainGroup to disable.
      * Height clipping only happens for the visiual character entity,
      * the position of the detour crowd agent that controls it is
      * not changed.
      **/
    void clipToTerrain(Ogre::TerrainGroup *terrainGroup);

protected:
    /**
      * Update current position of this character to the current position of its agent.
      **/
    virtual void updatePosition(Ogre::Real timeSinceLastFrame);

    /**
      * Set destination member variable directly without updating the agent state.
      * Usually you should call updateDestination() externally, unless you are controlling
      * the agents directly and need to update the corresponding character class to reflect
      * the change in state (see OgreRecastApplication friendship).
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
      * After updating the destination of all agents this variable should be set externally using
      * setDestination().
      **/
    Ogre::Vector3 mDestination;

    /**
      * Velocity set for this agent for manually controlling it.
      * If this is not zero then a manually set velocity is currently controlling the movement
      * of this character (not pathplanning towards a set destination).
      **/
    Ogre::Vector3 mManualVelocity;

    /**
      * True if this character is stopped.
      **/
    bool mStopped;

    /**
      * True if character is controlled by agent.
      * False if character is manually controlled without agent.
      **/
    bool mAgentControlled;

    /**
      * Detour Tile Cache component needed when controlling this character
      * manually without agent. It needs the dtTileCache to place a temporary
      * obstacle at its current position to make other characters in the crowd
      * avoid it.
      **/
    OgreDetourTileCache* mDetourTileCache;

    /**
      * Obstacle used when character is not being controlled by agent.
      * The temp obstacle is placed on the current position of this character
      * so that other agents in the crowd will not walk through it.
      **/
    dtTileRef mTempObstacle;

    /**
      * Helper to fix the height of the character to the terrain height after
      * a position update. Only the visual character entity's height will be
      * altered, not that of the detour crowd agent that controls it.
      **/
    virtual void clipToTerrainHeight(void);

    /**
      * The terraingroup the character height will be clipped to.
      * Set to NULL to disable this feature.
      **/
    Ogre::TerrainGroup *mClipTo;

    /**
      * Ray query used to query the terrain height when terrain clipping is
      * enabled. This value is stored as a member for optimization.
      **/
    Ogre::RaySceneQuery *mRaySceneQuery;


    // Friend the application class to allow setDestinationForAllAgents to update character destination values
    friend class OgreRecastApplication;
};

#endif // CHARACTER_H
