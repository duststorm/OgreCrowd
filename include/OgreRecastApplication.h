/*
-----------------------------------------------------------------------------
Filename:    OgreRecastApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __OgreRecastApplication_h_
#define __OgreRecastApplication_h_

#include "BaseApplication.h"
#include "OgreRecast.h"
#include "OgreRecastDemo.h"
#include "OgreDetourCrowd.h"
#include "Character.h"

class OgreRecastApplication : public BaseApplication
{
public:
    OgreRecastApplication(void);
    virtual ~OgreRecastApplication(void);

    /**
      * Calculate an intersection point of a ray with objects in the scene.
      **/
    bool rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject& foundMovable);

    /**
      * Retrieve a marker with specified name and optional material (to override default material).
      * Creates the marker if it does not exist yet.
      **/
    Ogre::SceneNode* getOrCreateMarker(Ogre::String name, Ogre::String materialName="");

    /**
      * Create a new character in the scene, backed with an agent in the crowd.
      * Character must have a unique name and will be positioned at specified position.
      **/
    Character* createCharacter(Ogre::String name, Ogre::Vector3 position);

    /**
      * Query flags used for ray intersection tests.
      * All scene entities but the navmesh should be in the default group.
      **/
    enum QueryFlags {
       DEFAULT_MASK = 0u,
       NAVMESH_MASK = 5u,
    };

    /**
      * Set to true to draw visual debug clues related to Recast/Detour navigation
      **/
    static const bool DEBUG_DRAW;

    /**
      * Set to true to use animated human characters, otherwise test cylinders will
      * be used to represent agents.
      **/
    static const bool HUMAN_CHARACTERS;


protected:
    /**
      * Initialise the scene and everything needed for pathfinding and steering.
      * Setup demo specific constructions.
      **/
    virtual void createScene(void);

    /**
      * Fix font loading.
      **/
    virtual void createFrameListener(void);

    /**
      * Handle mouse input.
      **/
    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

    /**
      * Handle keyboard input.
      **/
    virtual bool keyPressed( const OIS::KeyEvent &arg );

    /**
      * Update state for rendering a new frame.
      **/
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    /**
      * Update all agents to navigate to the new destination set by the end marker.
      * First agent well be repositioned at the begin marker.
      **/
    void UpdateAllAgentDestinations(void);

    /**
      * Find a new path between the markers and draw it as a line (only for debugging purposes).
      * Path is only drawn if DEBUG_DRAW is true.
      * Also makes begin marker visible.
      * Calculated path is stored at slot pathNb, its target is identified with id targetId.
      **/
    void drawPathBetweenMarkers(int pathNb, int targetId);

    /**
      * Set path and begin marker visible or not.
      * Path will only be set visible if DRAW_DEBUG is true.
      **/
    void setPathAndBeginMarkerVisibility(bool visibility);

    /**
      * Give each agent in the scene (except the first one) a different random destination.
      **/
    void setRandomTargetsForCrowd(void);

    /**
      * Set all agents in the scene except the first to follow the first agent.
      **/
    void setFollowTargetForCrowd(Ogre::Vector3 targetDestination);

    /**
      * Set the same destination for all agents in the scene.
      **/
    void setDestinationForAllAgents(Ogre::Vector3 destination, bool adjustExistingPath= false);

    /**
      * Find a new path using Recast and draw it as a line in the scene. The calculated path is stored
      * in slot pathNb for possible future reuse and its target is identified by targetId (how to use this
      * id is up to the developer, it's not used by recast or the demo app).
      **/
    void calculateAndDrawPath(Ogre::Vector3 beginPos, Ogre::Vector3 endPos, int pathNb, int targetId);

    /**
      * Current state the demo application is in.
      **/
    enum ApplicationState
    {
       SIMPLE_PATHFIND,
       CROWD_WANDER,
       FOLLOW_TARGET
    } mApplicationState;

private:
    /**
      * Ogre version of the code of the original Recast demo that comes with the recast distribution.
      **/
    OgreRecastDemo* mRecastDemo;

    /**
      * Ray scene intersection query object.
      **/
    Ogre::RaySceneQuery* mRayScnQuery;

    /**
      * Ogre wrapper around the detourCrowd library
      **/
    OgreDetourCrowd *mDetourCrowd;

    /**
      * Overlay label drawn on the screen. Shows current application state.
      **/
    OgreBites::Label *mLabelOverlay;

    /**
      * All characters in the scene that represent an agent.
      **/
    std::vector<Character*> mCharacters;

    /**
      * The destination for all agents that was last issued to DetourCrowd.
      * This is the destination that is implicitly given to all newly added agents
      * by detourCrowd. This value needs to be stored as it's hard to query this back
      * from detourCrowd or its agents.
      **/
    Ogre::Vector3 mLastSetDestination;
};




#endif // #ifndef __OgreRecastApplication_h_
