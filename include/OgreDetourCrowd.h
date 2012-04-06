#ifndef OGREDETOURCROWD_H
#define OGREDETOURCROWD_H

#include "DetourCrowd/DetourCrowd.h"
#include "OgreRecast.h"
#include "OgreRecastDemo.h"


class OgreDetourCrowd
{
public:
    OgreDetourCrowd(OgreRecastDemo *recastDemo);
    ~OgreDetourCrowd(void);

    void addAgent(const Ogre::Vector3 position);
    void removeAgent(const int idx);
    void hilightAgent(Ogre::Entity* agent);
    void setMoveTarget(Ogre::Vector3 position, bool adjust);
    void updateTick(const float dt);

    dtCrowd* m_crowd;
    OgreRecastDemo *m_recastDemo;

    Ogre::Entity *m_highlightedAgent;

    dtPolyRef m_targetRef;
    float m_targetPos[3];

    static const int AGENT_MAX_TRAIL = 64;
    static const int MAX_AGENTS = 128;
    struct AgentTrail
    {
            float trail[AGENT_MAX_TRAIL*3];
            int htrail;
    };
    AgentTrail m_trails[MAX_AGENTS];

    dtCrowdAgentDebugInfo m_agentDebug;
    dtObstacleAvoidanceDebugData* m_vod;


    // Agent parameters
    bool m_anticipateTurns;
    bool m_optimizeVis;
    bool m_optimizeTopo;
    bool m_obstacleAvoidance;
    bool m_separation;

    float m_obstacleAvoidanceType;
    float m_separationWeight;
};

#endif // OGREDETOURCROWD_H
