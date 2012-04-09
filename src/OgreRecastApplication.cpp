/*
-----------------------------------------------------------------------------
Filename:    OgreRecastApplication.cpp
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
#include "OgreRecastApplication.h"

//-------------------------------------------------------------------------------------
OgreRecastApplication::OgreRecastApplication(void)
        : mRecastDemo(0),
        mRayScnQuery(0),
        mDetourCrowd(0),
        mApplicationState(SIMPLE_PATHFIND),
        mMinSquaredDistanceToGoal(0.1)
{
}
//-------------------------------------------------------------------------------------
OgreRecastApplication::~OgreRecastApplication(void)
{
}

//-------------------------------------------------------------------------------------
void OgreRecastApplication::createScene(void)
{
    // Create navigateable dungeon
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight( "MainLight" );
    light->setPosition(20, 80, 50);

    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    mapE->setMaterialName("dungeon");
    Ogre::SceneNode* mapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MapNode");
    mapNode->attachObject(mapE);


    // RECAST (navmesh creation)
    // Create the navmesh and show it
    mRecastDemo = new OgreRecastDemo(mSceneMgr);
    if(mRecastDemo->NavMeshBuild(mapE)) {
        mRecastDemo->drawNavMesh();
    } else {
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
        return;
    }


    // DETOUR (pathfinding)
    // Do a pathing between two random points on the navmesh and draw the path
    int pathNb = 0;     // The index number for the slot in which the found path is to be stored
    int targetId = 0;   // Number identifying the target the path leads to
    Ogre::Vector3 beginPos = mRecastDemo->getRandomNavMeshPoint();
    Ogre::Vector3 endPos = mRecastDemo->getRandomNavMeshPoint();

    int ret = mRecastDemo->FindPath(beginPos, endPos, pathNb, targetId) ;
    if( ret >= 0)
        mRecastDemo->CreateRecastPathLine(pathNb) ; // Draw a line showing path at specified slot
    else
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a path. ("+mRecastDemo->getPathFindErrorMsg(ret)+")");


    //-------------------------------------------------
    // The rest of this method is specific to the demo

    // PLACE PATH BEGIN AND END MARKERS
    beginPos.y = beginPos.y + mRecastDemo->m_navMeshOffsetFromGround;
    endPos.y = endPos.y + mRecastDemo->m_navMeshOffsetFromGround;
    getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen")->setPosition(beginPos);
    getOrCreateMarker("EndPos", "Cylinder/Wires/Brown")->setPosition(endPos);


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);


    // SETUP RAY SCENE QUERYING
    // Add navmesh to separate querying group
    Ogre::SceneNode *navMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    navMeshNode->getAttachedObject("RecastMOWalk")->setQueryFlags(NAVMESH_MASK);
    // Exclude other meshes from navmesh queries
    navMeshNode->getAttachedObject("RecastMONeighbour")->setQueryFlags(DEFAULT_MASK);
    navMeshNode->getAttachedObject("RecastMOBoundary")->setQueryFlags(DEFAULT_MASK);
    mapE->setQueryFlags(DEFAULT_MASK);


    // CREATE CURSOR OVERLAY
    Ogre::Overlay *mCrosshair = Ogre::OverlayManager::getSingletonPtr()->getByName("GUI/Crosshair");
    mCrosshair->show();


    // DETOUR CROWD (local steering for independent agents)
    // Create a first agent that always starts at begin position
    mDetourCrowd = new OgreDetourCrowd(mRecastDemo);
    mDetourCrowd->addAgent(beginPos);   // create initial agent at start marker
    mDetourCrowd->setMoveTarget(endPos,false);  // Move agent(s) in crowd to destination
    Ogre::SceneNode *marker = getOrCreateMarker("Agent0", "Cylinder/LightBlue");
    marker->setPosition(beginPos);

    mMinSquaredDistanceToGoal = mRecastDemo->m_agentRadius*mRecastDemo->m_agentRadius;

    // TODO: bij zetten van destination moet je enkel destination voor allemaal zetten in simple mode
    // TODO: framerenderingqueued methode voor detectie goal reached wordt nooit uitgevoerd

    // TODO: add chase mode

    // TODO: initial agent beginpos enkel aanpassen in simpel mode? missch zelfs begin marker hiden in andere modes?

    // TODO: add anti-lockup fix in wander mode, especially on stairs agents sometimes stop

    // TODO: is destination changing in drawPath wel goed idee?

        // TODO: probeer paden te hergebruiken, zet scenarios op voor follow, flee, etc
        // TODO: laat bv toe met shift+click destination te veranderen zonder recalc
    // http://digestingduck.blogspot.com/2010/10/following-moving-target.html
    // http://digestingduck.blogspot.com/2011/01/detourcrowd.html

    // TODO: Voeg steering toe (possibilities: Millington, Buckland, opensteer)
        // http://www.red3d.com/cwr/steer/
    // TODO: voeg meerdere paden toe (meer dan alleen wander? random voor alle individuele agents of group behaviour)

    // TODO: klassen maken voor DetourPath etc, ipv gedoe met path slot
    // TODO: getters definieren op eigen klassen
    // TODO: betere wrapper voor detourCrowd, bv van entities ofzo ipv die max agents met active nest
    // TODO: const en inline in method headings zetten waar mogelijk
    // TODO: path planning etc in aparte thread
    // TODO: serialize navmesh naar .obj

    // TODO: splitting world and navmesh in separate tiles
    // TODO: integrate pathing on Ogre::Terrain
        // http://groups.google.com/group/recastnavigation/browse_thread/thread/14c338dd04285250#

    // TODO: tweak things like MAX_ITERS_PER_UPDATE

    // TODO: create separate methods for eg. addAgent in this class (instead of inlining it all in the input handlers)
}


bool OgreRecastApplication::rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject &foundMovable)
{
    //if(!mRayScnQuery)
        mRayScnQuery = mSceneMgr->createRayQuery(Ogre::Ray(), queryMask);

    mRayScnQuery->setRay(ray);
    Ogre::RaySceneQueryResult& query_result = mRayScnQuery->execute();


        // at this point we have raycast to a series of different objects bounding boxes.
        // we need to test these different objects to see which is the first polygon hit.
        // there are some minor optimizations (distance based) that mean we wont have to
        // check all of the objects most of the time, but the worst case scenario is that
        // we need to test every triangle of every object.
        Ogre::Real closest_distance = -1.0f;
        Ogre::Vector3 closest_result;
        Ogre::MovableObject *closest_movable;
        for (size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++)
        {
            // Debug:
            //Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getName());
            //Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getMovableType());


            // stop checking if we have found a raycast hit that is closer
            // than all remaining entities
            if ((closest_distance >= 0.0f) &&
                (closest_distance < query_result[qr_idx].distance))
            {
                 break;
            }

            // only check this result if its a hit against an entity
            if ((query_result[qr_idx].movable != NULL) &&
                ((query_result[qr_idx].movable->getMovableType().compare("Entity") == 0)
                ||query_result[qr_idx].movable->getMovableType().compare("ManualObject") == 0))
            {
                // mesh data to retrieve
                size_t vertex_count;
                size_t index_count;
                Ogre::Vector3 *vertices;
                unsigned long *indices;

                // get the mesh information
                if(query_result[qr_idx].movable->getMovableType().compare("Entity") == 0) {
                    // For entities
                    // get the entity to check
                    Ogre::Entity *pentity = static_cast<Ogre::Entity*>(query_result[qr_idx].movable);

                    mRecastDemo->getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                                  pentity->getParentNode()->_getDerivedPosition(),
                                  pentity->getParentNode()->_getDerivedOrientation(),
                                  pentity->getParentNode()->_getDerivedScale());
                } else {
                    // For manualObjects
                    // get the entity to check
                    Ogre::ManualObject *pmanual = static_cast<Ogre::ManualObject*>(query_result[qr_idx].movable);

                    mRecastDemo->getManualMeshInformation(pmanual, vertex_count, vertices, index_count, indices,
                                  pmanual->getParentNode()->_getDerivedPosition(),
                                  pmanual->getParentNode()->_getDerivedOrientation(),
                                  pmanual->getParentNode()->_getDerivedScale());
                }

                // test for hitting individual triangles on the mesh
                bool new_closest_found = false;
                for (int i = 0; i < static_cast<int>(index_count); i += 3)
                {
                    // check for a hit against this triangle
                    std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
                        vertices[indices[i+1]], vertices[indices[i+2]], true, false);

                    // if it was a hit check if its the closest
                    if (hit.first)
                    {
                        if ((closest_distance < 0.0f) ||
                            (hit.second < closest_distance))
                        {
                            // this is the closest so far, save it off
                            closest_distance = hit.second;
                            new_closest_found = true;
                        }
                    }
                }


             // free the verticies and indicies memory
                delete[] vertices;
                delete[] indices;

                // if we found a new closest raycast for this object, update the
                // closest_result before moving on to the next object.
                if (new_closest_found)
                {
                    closest_result = ray.getPoint(closest_distance);
                    if(query_result[qr_idx].movable != NULL)
                        closest_movable = query_result[qr_idx].movable;
                }
            }
        }

        // return the result
        if (closest_distance >= 0.0f)
        {
            // raycast success
            result = closest_result;
            //foundMovable = *closest_movable;  // TODO fix this!
            return (true);
        }
        else
        {
            // raycast failed
            return (false);
        }
}

bool OgreRecastApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    // Do ray scene query
    //send a raycast straight out from the camera at the center position
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(0.5, 0.5);

    Ogre::Vector3 rayHitPoint;
    Ogre::MovableObject *rayHitObject;
    if (rayQueryPointInScene(mouseRay, NAVMESH_MASK, rayHitPoint, *rayHitObject)) {
        Ogre::SceneNode *markerNode = NULL;

        if(id == OIS::MB_Right) {
            markerNode = getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen");
        }

        if(id == OIS::MB_Left) {
            markerNode = getOrCreateMarker("EndPos", "Cylinder/Wires/Brown");
        }

        if(markerNode != NULL) {
            markerNode->setPosition(rayHitPoint);
        }

        drawPathBetweenMarkers(1,1);
    }

    BaseApplication::mousePressed(arg, id);
}

Ogre::SceneNode* OgreRecastApplication::getOrCreateMarker(Ogre::String name, Ogre::String materialName)
{
    Ogre::SceneNode *result  = NULL;
    try {
        result = (Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild(name+"Node"));
    } catch(Ogre::Exception ex) {
        result = mSceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
        Ogre::Entity* ent = mSceneMgr->createEntity(name, "Cylinder.mesh");
        if(materialName.compare("") != 0)
            ent->setMaterialName(materialName);
        result->attachObject(ent);
        ent->setQueryFlags(DEFAULT_MASK);   // Exclude from ray queries
        // Set marker scale to size of agent
        result->setScale(mRecastDemo->m_agentRadius*2, mRecastDemo->m_agentHeight,mRecastDemo->m_agentRadius*2);
    }

    return result;
}

void OgreRecastApplication::drawPathBetweenMarkers(int pathNb, int targetId)
{
    try {
        Ogre::Vector3 beginPos = ((Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild("BeginPosNode")))->getPosition();
        Ogre::Vector3 endPos = ((Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild("EndPosNode")))->getPosition();

        int ret = mRecastDemo->FindPath(beginPos, endPos, pathNb, targetId) ;
        if( ret >= 0)
            mRecastDemo->CreateRecastPathLine(pathNb) ; // Draw a line showing path at slot 0
        else
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a path. ("+mRecastDemo->getPathFindErrorMsg(ret)+")");

        mDetourCrowd->removeAgent(0);
        mDetourCrowd->addAgent(beginPos);
        if(mApplicationState == SIMPLE_PATHFIND)
            mDetourCrowd->setMoveTarget(endPos, false);  // Set move target for entire crowd
        else
            mDetourCrowd->setMoveTarget(0, endPos, false);  // Only update destination of first agent
    } catch(Ogre::Exception ex) {
        // Either begin or end marker have not yet been placed
        return;
    }

}

bool OgreRecastApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    mDetourCrowd->updateTick(evt.timeSinceLastFrame);

    Ogre::Vector3 firstAgentPos = getFirstAgentPosition();
    Ogre::Vector3 agentPos;
    std::vector<int> agentIDs = mDetourCrowd->getActiveAgentIds();
    std::vector<int>::iterator iter;

    for(iter=agentIDs.begin(); iter != agentIDs.end(); iter++) {
        int agentID = *iter;
        const dtCrowdAgent* agent = mDetourCrowd->m_crowd->getAgent(agentID);

        const float *pos = agent->npos;
        mRecastDemo->FloatAToOgreVect3(pos,agentPos);
        agentPos.y = agentPos.y+mRecastDemo->m_navMeshOffsetFromGround; // Compensate for distance of navmesh above ground
        ((Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild("Agent"+Ogre::StringConverter::toString(agentID)+"Node")))->setPosition(agentPos);

        // RANDOM WANDER BEHAVIOUR
        if(mApplicationState == CROWD_WANDER &&
           agentID != 0) // No random walking for first agent
        {
            // Check if destination has been reached
//TODO: find out how to ask dtCrowd when an agent has reached its destination
            //const dtCrowd::MoveRequest *m= mDetourCrowd->m_crowd->getActiveMoveTarget( agentID );
            //if(m != NULL && m->pos != NULL) {
            //    Ogre::LogManager::getSingletonPtr()->logMessage("destination reached "+Ogre::StringConverter::toString(i));
            //    mRecastDemo->FloatAToOgreVect3(m->pos, agentDestination);

            // Correct position again (remove offset again)
            agentPos.y = agentPos.y - mRecastDemo->m_navMeshOffsetFromGround;

            // If destination reached: Set new random destination
            if (agentPos.squaredDistance(mDestinations[agentID]) < mMinSquaredDistanceToGoal) {
                mDestinations[agentID] = mRecastDemo->getRandomNavMeshPoint();
                mDetourCrowd->setMoveTarget(agentID, mDestinations[agentID], false);
            }
        }


        // CHASE BEHAVIOUR
        if(mApplicationState == FOLLOW_TARGET &&
           agentID != 0) // First agent doesn't chase itself
        {
            // Only ADJUST path to follow target (no full recalculation of the corridor to follow)
            mDetourCrowd->setMoveTarget(agentID, firstAgentPos, true);
        }
    }

    return BaseApplication::frameRenderingQueued(evt);
}

bool OgreRecastApplication::keyPressed( const OIS::KeyEvent &arg )
{
    // SPACE places a new agent at cursor position
    if(arg.key == OIS::KC_SPACE
        && mDetourCrowd->getNbAgents() < mDetourCrowd->getMaxNbAgents()) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (rayQueryPointInScene(cursorRay, NAVMESH_MASK, rayHitPoint, *rayHitObject)) {
            int agentID = mDetourCrowd->addAgent(rayHitPoint);

            // If in wander mode, give a random destination to agent (otherwise it will take the destination of the previous one automatically)
            if(mApplicationState == CROWD_WANDER) {
                mDestinations[agentID] = mRecastDemo->getRandomNavMeshPoint();
                mDetourCrowd->setMoveTarget(agentID, mDestinations[agentID], false);
            }

            Ogre::SceneNode *marker = getOrCreateMarker("Agent"+Ogre::StringConverter::toString(agentID), "Cylinder/Blue");
            marker->setPosition(rayHitPoint);
        }
    }

    // ENTER switches to different demo mode
    if(arg.key == OIS::KC_RETURN) {
        drawPathBetweenMarkers(1, 1);   // Start Moving main agent between begin and endpoints again (assigns same endposition to all agents in crowd)

        // Change demo mode
        switch (mApplicationState) {
            case SIMPLE_PATHFIND:   // Simple -> wander
                mApplicationState = CROWD_WANDER;
                setRandomTargetsForCrowd();
                break;
            case CROWD_WANDER:      // Wander -> chase
                mApplicationState = FOLLOW_TARGET;
                setFollowTargetForCrowd(getFirstAgentPosition());
                break;
            case FOLLOW_TARGET:     // Chase -> simple
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);    // Reinitialize path that all agents follow
            default:
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);
        }

    }

    return BaseApplication::keyPressed(arg);
}

void OgreRecastApplication::setRandomTargetsForCrowd()
{
    std::vector<int> activeAgentIds = mDetourCrowd->getActiveAgentIds();
    std::vector<int>::iterator iter;
    for(iter = activeAgentIds.begin(); iter != activeAgentIds.end(); iter++) {
        if(*iter != 0) {  // Don't randomize first agent's destination
            mDestinations[*iter] = mRecastDemo->getRandomNavMeshPoint();
            mDetourCrowd->setMoveTarget(*iter, mDestinations[*iter], false);
        }
    }
}

void OgreRecastApplication::setFollowTargetForCrowd(Ogre::Vector3 targetDestination)
{
    std::vector<int> activeAgentIds = mDetourCrowd->getActiveAgentIds();
    std::vector<int>::iterator iter;
    for(iter = activeAgentIds.begin(); iter != activeAgentIds.end(); iter++) {
        if(*iter != 0) {  // Don't move first agent
            // Recalculate a full new path to the target
            mDetourCrowd->setMoveTarget(*iter, targetDestination, false);
        }
    }
}

Ogre::Vector3 OgreRecastApplication::getFirstAgentPosition()
{
    return ((Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild("Agent0Node")))->getPosition();
}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        OgreRecastApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
