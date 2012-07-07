#include "Ogre.h"
#include "OgreDetourCrowd.h"
#include "Detour/DetourCommon.h"


OgreDetourCrowd::OgreDetourCrowd(OgreRecast *recast)
    : m_crowd(0),
    m_recast(recast),
    m_targetRef(0),
    m_activeAgents(0)
{
    m_crowd = dtAllocCrowd();
    if(!m_crowd)
        Ogre::LogManager::getSingletonPtr()->logMessage("Error: Could not allocate crowd instance.");


    // Set default agent parameters
    m_anticipateTurns = true;
    m_optimizeVis = true;
    m_optimizeTopo = true;
    m_obstacleAvoidance = true;
    m_separation = false;

    m_obstacleAvoidanceType = 3.0f;
    m_separationWeight = 2.0f;


    memset(m_trails, 0, sizeof(m_trails));

    m_vod = dtAllocObstacleAvoidanceDebugData();
    m_vod->init(2048);

    memset(&m_agentDebug, 0, sizeof(m_agentDebug));
    m_agentDebug.idx = -1;
    m_agentDebug.vod = m_vod;



    dtNavMesh* nav = recast->m_navMesh;
    dtCrowd* crowd = m_crowd;
            if (nav && crowd && crowd->getAgentCount() == 0)
            {
                    crowd->init(MAX_AGENTS, m_recast->getAgentRadius(), nav);

                    // Make polygons with 'disabled' flag invalid.
                    crowd->getEditableFilter()->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);


                    // Create different avoidance settings presets. The crowd object can store multiple, identified by an index number.
                    // Setup local avoidance params to different qualities.
                    dtObstacleAvoidanceParams params;
                    // Use mostly default settings, copy from dtCrowd.
                    memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

                    // Low (11)
                    params.velBias = 0.5f;
                    params.adaptiveDivs = 5;
                    params.adaptiveRings = 2;
                    params.adaptiveDepth = 1;
                    crowd->setObstacleAvoidanceParams(0, &params);

                    // Medium (22)
                    params.velBias = 0.5f;
                    params.adaptiveDivs = 5;
                    params.adaptiveRings = 2;
                    params.adaptiveDepth = 2;
                    crowd->setObstacleAvoidanceParams(1, &params);

                    // Good (45)
                    params.velBias = 0.5f;
                    params.adaptiveDivs = 7;
                    params.adaptiveRings = 2;
                    params.adaptiveDepth = 3;
                    crowd->setObstacleAvoidanceParams(2, &params);

                    // High (66)
                    params.velBias = 0.5f;
                    params.adaptiveDivs = 7;
                    params.adaptiveRings = 3;
                    params.adaptiveDepth = 3;

                    crowd->setObstacleAvoidanceParams(3, &params);
            }
}

OgreDetourCrowd::~OgreDetourCrowd()
{
    dtFreeCrowd(m_crowd);
    dtFreeObstacleAvoidanceDebugData(m_vod);
}


Ogre::Real OgreDetourCrowd::getAgentHeight()
{
    return m_recast->getAgentHeight();
}

Ogre::Real OgreDetourCrowd::getAgentRadius()
{
    return m_recast->getAgentRadius();
}

void OgreDetourCrowd::updateTick(const float dt)
{
        dtNavMesh* nav = m_recast->m_navMesh;
        dtCrowd* crowd = m_crowd;
        if (!nav || !crowd) return;

//        TimeVal startTime = getPerfTime();

        crowd->update(dt, &m_agentDebug);

//        TimeVal endTime = getPerfTime();

        // Update agent trails
        for (int i = 0; i < crowd->getAgentCount(); ++i)
        {
                const dtCrowdAgent* ag = crowd->getAgent(i);
                AgentTrail* trail = &m_trails[i];
                if (!ag->active)
                        continue;
                // Update agent movement trail.
                trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
                dtVcopy(&trail->trail[trail->htrail*3], ag->npos);
        }

        m_agentDebug.vod->normalizeSamples();

        //m_crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
        //m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}


int OgreDetourCrowd::addAgent(const Ogre::Vector3 position)
{
        // Define parameters for agent in crowd
        dtCrowdAgentParams ap;
        memset(&ap, 0, sizeof(ap));
        ap.radius = getAgentRadius();
        ap.height = getAgentHeight();
        ap.maxAcceleration = 8.0f;
        ap.maxSpeed = (ap.height/2)*1.5f;//3.5
        ap.collisionQueryRange = ap.radius * 12.0f;
        ap.pathOptimizationRange = ap.radius * 30.0f;

        // Set update flags according to config
        ap.updateFlags = 0;
        if (m_anticipateTurns)
                ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
        if (m_optimizeVis)
                ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
        if (m_optimizeTopo)
                ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
        if (m_obstacleAvoidance)
                ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
        if (m_separation)
                ap.updateFlags |= DT_CROWD_SEPARATION;
        ap.obstacleAvoidanceType = (unsigned char)m_obstacleAvoidanceType;
        ap.separationWeight = m_separationWeight;

        float p[3];
        OgreRecast::OgreVect3ToFloatA(position, p);
        int idx = m_crowd->addAgent(p, &ap);
        if (idx != -1)
        {
            // If a move target is defined: move agent towards it
// TODO do we want to set newly added agent's destination to previously set target? or remove this behaviour?
                if (m_targetRef)
                        m_crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);

                // Init trail
                AgentTrail* trail = &m_trails[idx];
                for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
                        dtVcopy(&trail->trail[i*3], p);
                trail->htrail = 0;
        }

        m_activeAgents++;

        return idx;
}

int OgreDetourCrowd::getNbAgents()
{
    return m_activeAgents;
}

int OgreDetourCrowd::getMaxNbAgents()
{
    return m_crowd->getAgentCount();
}

std::vector<dtCrowdAgent*> OgreDetourCrowd::getActiveAgents()
{
    dtCrowdAgent** resultEntries = new dtCrowdAgent*[getMaxNbAgents()];
    int size = m_crowd->getActiveAgents(resultEntries,getMaxNbAgents());

    std::vector<dtCrowdAgent*> result(resultEntries, resultEntries + size);
    delete[] resultEntries;
    return result;
}

std::vector<int> OgreDetourCrowd::getActiveAgentIds(void)
{
    std::vector<int> result = std::vector<int>();

    const dtCrowdAgent* agent = NULL;
    for(int i=0; i<getMaxNbAgents(); i++) {
        agent = m_crowd->getAgent(i);
        if(agent->active)
            result.push_back(i);
    }

    return result;
}


void OgreDetourCrowd::removeAgent(const int idx)
{
        m_crowd->removeAgent(idx);

        m_activeAgents--;
}

const dtCrowdAgent* OgreDetourCrowd::getAgent(int id)
{
    return m_crowd->getAgent(id);
}


Ogre::Vector3 OgreDetourCrowd::calcVel(Ogre::Vector3 position, Ogre::Vector3 target, Ogre::Real speed)
{
    float pos[3];
    OgreRecast::OgreVect3ToFloatA(position, pos);

    float tgt[3];
    OgreRecast::OgreVect3ToFloatA(target, tgt);

    float res[3];
    calcVel(res, pos, tgt, speed);

    Ogre::Vector3 result;
    OgreRecast::FloatAToOgreVect3(res, result);

    return result;
}


void OgreDetourCrowd::calcVel(float* velocity, const float* position, const float* target, const float speed)
{
        dtVsub(velocity, target, position);
        velocity[1] = 0.0;
        dtVnormalize(velocity);
        dtVscale(velocity, velocity, speed);
}


void OgreDetourCrowd::setMoveTarget(Ogre::Vector3 position, bool adjust)
{
        // Find nearest point on navmesh and set move request to that location.
        dtNavMeshQuery* navquery = m_recast->m_navQuery;
        dtCrowd* crowd = m_crowd;
        const dtQueryFilter* filter = crowd->getFilter();
        const float* ext = crowd->getQueryExtents();
        float p[3];
        OgreRecast::OgreVect3ToFloatA(position, p);

        navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);

        // Adjust target using tiny local search. (instead of recalculating full path)
        if (adjust)
        {
                        for (int i = 0; i < crowd->getAgentCount(); ++i)
                        {
                                const dtCrowdAgent* ag = crowd->getAgent(i);
                                if (!ag->active) continue;
                                float vel[3];
                                calcVel(vel, ag->npos, p, ag->params.maxSpeed);
                                crowd->requestMoveVelocity(i, vel);
                        }
        }
        else
        {
                // Move target using path finder (recalculate a full new path)
                        for (int i = 0; i < crowd->getAgentCount(); ++i)
                        {
                                const dtCrowdAgent* ag = crowd->getAgent(i);
                                if (!ag->active) continue;
                                crowd->requestMoveTarget(i, m_targetRef, m_targetPos);
                        }
        }
}

void OgreDetourCrowd::setMoveTarget(int agentId, Ogre::Vector3 position, bool adjust)
{
    // TODO extract common method
    // Find nearest point on navmesh and set move request to that location.
    dtNavMeshQuery* navquery = m_recast->m_navQuery;
    dtCrowd* crowd = m_crowd;
    const dtQueryFilter* filter = crowd->getFilter();
    const float* ext = crowd->getQueryExtents();
    float p[3];
    OgreRecast::OgreVect3ToFloatA(position, p);

    navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);
    // ----

    if (adjust) {
        const dtCrowdAgent *ag = getAgent(agentId);
        float vel[3];
        calcVel(vel, ag->npos, p, ag->params.maxSpeed);
        crowd->requestMoveVelocity(agentId, vel);
    } else {
        m_crowd->requestMoveTarget(agentId, m_targetRef, m_targetPos);
    }
}

Ogre::Vector3 OgreDetourCrowd::getLastDestination()
{
    Ogre::Vector3 result;
    OgreRecast::FloatAToOgreVect3(m_targetPos, result);
    return result;
}

bool OgreDetourCrowd::requestVelocity(int agentId, Ogre::Vector3 velocity)
{
    if (!getAgent(agentId)->active)
        return false;

    float vel[3];
    OgreRecast::OgreVect3ToFloatA(velocity, vel);

    return m_crowd->requestMoveVelocity(agentId, vel);
}

bool OgreDetourCrowd::stopAgent(int agentId)
{
    float zeroVel[] = {0,0,0};
    return m_crowd->resetMoveTarget(agentId) && m_crowd->requestMoveVelocity(agentId, zeroVel);
}
