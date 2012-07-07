#ifndef OGREDETOURCROWD_H
#define OGREDETOURCROWD_H

#include "OgreRecastDefinitions.h"
#include "OgreRecast.h"
#include "DetourCrowd/DetourCrowd.h"
#include <vector>


/**
  * Ogre wrapper around DetourCrowd.
  * Controls a crowd of agents that can steer to avoid each other and follow
  * individual paths.
  *
  * This class is largely based on the CrowdTool used in the original recastnavigation
  * demo.
  **/
class OgreDetourCrowd
{
public:
    /**
      * Initialize a detour crowd that will manage agents on the specified
      * recast navmesh. It does not matter how this navmesh is constructed
      * (either with OgreRecast directly or with DetourTileCache).
      * Parameters such as agent dimensions will be taken from the specified
      * recast component.
      **/
    OgreDetourCrowd(OgreRecast *recast);
    ~OgreDetourCrowd(void);

    /**
      * Add an agent to the crowd
      * Returns ID of created agent (-1 if maximum agents is already created)
      **/
    int addAgent(const Ogre::Vector3 position);

    /**
      * Retrieve agent with specified ID from the crowd.
      **/
    const dtCrowdAgent* getAgent(int id);

    /**
      * Remove agent with specified ID from the crowd.
      **/
    void removeAgent(const int idx);

    /**
      * Set global destination or target for all agents in the crowd.
      * Setting adjust to true will try to adjust the current calculated path
      * of the agents slightly to end at the new destination, avoiding the need
      * to calculate a completely new path. This only works if the destination is
      * close to the previously set one, for example when chasing a moving entity.
      **/
    void setMoveTarget(Ogre::Vector3 position, bool adjust);

    /**
      * Set target or destination for an individual agent.
      * Setting adjust to true will try to adjust the current calculated path
      * of the agent slightly to end at the new destination, avoiding the need
      * to calculate a completely new path. This only works if the destination is
      * close to the previously set one, for example when chasing a moving entity.
      **/
    void setMoveTarget(int agentId, Ogre::Vector3 position, bool adjust);

    /**
      * Request a specified velocity for the agent with specified index.
      * Requesting a velocity means manually controlling an agent.
      * Returns true if the request was successful.
      **/
    bool requestVelocity(int agentId, Ogre::Vector3 velocity);

    /**
      * Cancels any request for the specified agent, making it stop.
      * Returns true if the request was successul.
      **/
    bool stopAgent(int agentId);

    /**
      * Helper that calculates the needed velocity to steer an agent to a target destination.
      * Parameters:
      *     position    is the current position of the agent
      *     target      is the target destination to reach
      *     speed       is the (max) speed the agent can travel at
      * Returns the calculated velocity.
      *
      * This function can be used together with requestMoveVelocity to achieve the functionality
      * of adjustMoveTarget function.
      **/
    static Ogre::Vector3 calcVel(Ogre::Vector3 position, Ogre::Vector3 target, Ogre::Real speed);

    /**
      * Update method for the crowd manager. Will calculate new positions for moving agents.
      * Call this method in your frameloop every frame to make your agents move.
      *
      * DetourCrowd uses sampling based local steering to calculate a velocity vector for each
      * agent. The calculation time of this is limited to the number of agents in the crowd and
      * the sampling amount (which is a constant).
      *
      * Additionally pathfinding tasks are queued and the number of computations is limited, to
      * limit the maximum amount of time spent for preparing a frame. This can have as consequence
      * that some agents will only start to move after a few frames, when their paths are calculated.
      **/
    void updateTick(const float dt);

    /**
      * The height of agents in this crowd. All agents in a crowd have the same height, and height is
      * determined by the agent height parameter with which the navmesh is build.
      **/
    Ogre::Real getAgentHeight(void);

    /**
      * The radius of agents in this crowd. All agents in a crowd have the same radius, and radius
      * determined by the agent radius parameter with which the navmesh is build.
      **/
    Ogre::Real getAgentRadius(void);

    /**
      * The number of (active) agents in this crowd.
      **/
    int getNbAgents(void);

    /**
      * The maximum number of agents that are allowed in this crowd.
      **/
    int getMaxNbAgents(void);

    /**
      * Get all (active) agents in this crowd.
      **/
    std::vector<dtCrowdAgent*> getActiveAgents(void);

    /**
      * Get the IDs of all (active) agents in this crowd.
      **/
    std::vector<int> getActiveAgentIds(void);

    /**
      * The last set destination for the crowd. This is the destination
      * that will be assigned to newly added agents.
      **/
    Ogre::Vector3 getLastDestination(void);

    /**
      * Reference to the DetourCrowd object that is wrapped.
      **/
    dtCrowd* m_crowd;

    /**
      * Reference to the Recast/Detour wrapper object for Ogre.
      **/
    OgreRecast *m_recast;

    /**
      * The latest set target or destination section in the recast navmesh.
      **/
    dtPolyRef m_targetRef;

    /**
      * The latest set target or destination position.
      **/
    float m_targetPos[3];

    /**
      * Max pathlength for calculated paths.
      **/
    static const int AGENT_MAX_TRAIL = 64;

    /**
      * Max number of agents allowed in this crowd.
      **/
    static const int MAX_AGENTS = 128;

    /**
      * Stores the calculated paths for each agent in the crowd.
      **/
    struct AgentTrail
    {
            float trail[AGENT_MAX_TRAIL*3];
            int htrail;
    };
    AgentTrail m_trails[MAX_AGENTS];

    /**
      * Debug info object used in the original recast/detour demo, not used in this
      * application.
      **/
    dtCrowdAgentDebugInfo m_agentDebug;

    /**
      * Parameters for obstacle avoidance of DetourCrowd steering.
      **/
    dtObstacleAvoidanceDebugData* m_vod;


    // Agent configuration parameters
    bool m_anticipateTurns;
    bool m_optimizeVis;
    bool m_optimizeTopo;
    bool m_obstacleAvoidance;
    bool m_separation;

    float m_obstacleAvoidanceType;
    float m_separationWeight;


protected:
    /**
      * Helper to calculate the needed velocity to steer an agent to a target destination.
      * Parameters:
      *     velocity    is the return parameter, the calculated velocity
      *     position    is the current position of the agent
      *     target      is the target destination to reach
      *     speed       is the (max) speed the agent can travel at
      *
      * This function can be used together with requestMoveVelocity to achieve the functionality
      * of the old adjustMoveTarget function.
      **/
    static void calcVel(float* velocity, const float* position, const float* target, const float speed);


private:
    /**
      * Number of (active) agents in the crowd.
      **/
    int m_activeAgents;
};

#endif // OGREDETOURCROWD_H
