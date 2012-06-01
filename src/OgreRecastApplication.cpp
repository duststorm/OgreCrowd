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
#include "TestCharacter.h"
#include "AnimateableCharacter.h"


//--- SETTINGS ------------------------------------------------------------------------

// Set to true to draw debug objects
const bool OgreRecastApplication::DEBUG_DRAW = false;

// Set to true to show agents as animated human characters instead of cylinders
const bool OgreRecastApplication::HUMAN_CHARACTERS = true;

//-------------------------------------------------------------------------------------


OgreRecastApplication::OgreRecastApplication(void)
        : mRecast(0),
        mRayScnQuery(0),
        mDetourCrowd(0),
        mApplicationState(SIMPLE_PATHFIND),
        mLabelOverlay(0),
        mCharacters()
{
}

OgreRecastApplication::~OgreRecastApplication(void)
{
}

void OgreRecastApplication::createScene(void)
{
    // Basic scene setup
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight( "MainLight" );
    light->setPosition(20, 80, 50);

    // Create navigateable dungeon
    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    mapE->setMaterialName("dungeon");
    Ogre::SceneNode* mapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MapNode");
    mapNode->attachObject(mapE);


    // RECAST (navmesh creation)
    // Create the navmesh and show it
    mRecast = new OgreRecast(mSceneMgr);
    if(mRecast->NavMeshBuild(mapE)) {
        mRecast->drawNavMesh();
    } else {
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
        return;
    }


    // DETOUR (pathfinding)
    // Do a pathing between two random points on the navmesh and draw the path
    // Note that because we are using DetourCrowd we will not be doing pathfinds directly, DetourCrowd
    // will do this for us.
    int pathNb = 0;     // The index number for the slot in which the found path is to be stored
    int targetId = 0;   // Number identifying the target the path leads to
    Ogre::Vector3 beginPos = mRecast->getRandomNavMeshPoint();
    Ogre::Vector3 endPos = mRecast->getRandomNavMeshPoint();
    if(OgreRecastApplication::DEBUG_DRAW)
        calculateAndDrawPath(beginPos, endPos, pathNb, targetId);


    // DETOUR CROWD (local steering for independent agents)
    // Create a first agent that always starts at begin position
    mDetourCrowd = new OgreDetourCrowd(mRecast);
    Character *character = createCharacter("Agent0", beginPos);    // create initial agent at start marker
    if(!HUMAN_CHARACTERS)
        character->getEntity()->setMaterialName("Cylinder/LightBlue");  // Give the first agent a different color
    setDestinationForAllAgents(endPos, false);  // Move all agents in crowd to destination



    //-------------------------------------------------
    // The rest of this method is specific to the demo

    // PLACE PATH BEGIN AND END MARKERS
    beginPos.y = beginPos.y + mRecast->m_navMeshOffsetFromGround;
    endPos.y = endPos.y + mRecast->m_navMeshOffsetFromGround;
    getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen")->setPosition(beginPos);
    getOrCreateMarker("EndPos", "Cylinder/Wires/Brown")->setPosition(endPos);


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);


    // SETUP RAY SCENE QUERYING
    // Used for mouse picking begin and end markers and determining the position to add new agents
    // Add navmesh to separate querying group that we will use
    Ogre::SceneNode *navMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    navMeshNode->getAttachedObject("RecastMOWalk")->setQueryFlags(NAVMESH_MASK);
    // Exclude other meshes from navmesh queries
    navMeshNode->getAttachedObject("RecastMONeighbour")->setQueryFlags(DEFAULT_MASK);
    navMeshNode->getAttachedObject("RecastMOBoundary")->setQueryFlags(DEFAULT_MASK);
    mapE->setQueryFlags(DEFAULT_MASK);

    if(!OgreRecastApplication::DEBUG_DRAW)
        navMeshNode->setVisible(false); // Even though we make it invisible, we still keep the navmesh entity in the scene to do ray intersection tests


    // CREATE CURSOR OVERLAY
    Ogre::Overlay *mCrosshair = Ogre::OverlayManager::getSingletonPtr()->getByName("GUI/Crosshair");
    mCrosshair->show(); // Show a cursor in the center of the screen
}

void OgreRecastApplication::createFrameListener()
{
    BaseApplication::createFrameListener();

    // CREATE INFO LABEL
    Ogre::FontManager::getSingleton().getByName("SdkTrays/Caption")->load();    // Fixes a bug with SDK Trays
    mLabelOverlay = mTrayMgr->createLabel(OgreBites::TL_TOPLEFT, "infoLabel", "Simple navigation", 220);
    mLabelOverlay->show();
    OgreBites::Label *label = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "titleLabel", "Recast & Detour Demo", 250);
    label->show();
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


        if(id == OIS::MB_Left) {
            markerNode = getOrCreateMarker("EndPos", "Cylinder/Wires/Brown");

            if(mApplicationState != SIMPLE_PATHFIND)
                mCharacters[0]->updateDestination(rayHitPoint, false);  // Update destination of first agent only
        }

        if(id == OIS::MB_Right && mApplicationState == SIMPLE_PATHFIND) {
            markerNode = getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen");
        }

        if(markerNode != NULL) {
            markerNode->setPosition(rayHitPoint);
        }

        if(mApplicationState == SIMPLE_PATHFIND) {
            drawPathBetweenMarkers(1,1);    // Draw navigation path (in DRAW_DEBUG) and begin marker
            UpdateAllAgentDestinations();   //Steer all agents to set destination
        }
    }

    BaseApplication::mousePressed(arg, id);
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
            Character *character = createCharacter("Agent"+Ogre::StringConverter::toString(mCharacters.size()), rayHitPoint);

            // If in wander mode, give a random destination to agent (otherwise it will take the destination of the previous agent automatically)
            if(mApplicationState == CROWD_WANDER) {
                character->updateDestination(mRecast->getRandomNavMeshPoint(), false);
            }
        }
    }

    // ENTER switches to different demo mode
    if(arg.key == OIS::KC_RETURN) {
        // Change demo mode
        switch (mApplicationState) {
            case SIMPLE_PATHFIND:   // Simple -> wander
                mApplicationState = CROWD_WANDER;
                setRandomTargetsForCrowd();
                setPathAndBeginMarkerVisibility(false);
                mLabelOverlay->setCaption("Random crowd wander");
                break;
            case CROWD_WANDER:      // Wander -> chase
                mApplicationState = FOLLOW_TARGET;
                setFollowTargetForCrowd(mCharacters[0]->getPosition());
                setPathAndBeginMarkerVisibility(false);
                mLabelOverlay->setCaption("Chase");
                break;
            case FOLLOW_TARGET:     // Chase -> simple
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);    // Draw navigation path (in DRAW_DEBUG) and begin marker
                UpdateAllAgentDestinations();   // Reinitialize path that all agents follow and steer them towards end marker
                mLabelOverlay->setCaption("Simple navigation");
                break;
            default:
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);
                UpdateAllAgentDestinations();
                mLabelOverlay->setCaption("Simple navigation");
        }
    }

    return BaseApplication::keyPressed(arg);
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

        // Set marker scale to size of agent
        result->setScale(mRecast->m_agentRadius*2, mRecast->m_agentHeight,mRecast->m_agentRadius*2);

        ent->setQueryFlags(DEFAULT_MASK);   // Exclude from ray queries
    }

    return result;
}

Character* OgreRecastApplication::createCharacter(Ogre::String name, Ogre::Vector3 position)
{
    if(mDetourCrowd->getNbAgents() >= mDetourCrowd->getMaxNbAgents()) {
        Ogre::LogManager::getSingletonPtr()->logMessage("Error: Cannot create crowd agent for new character. Limit of "+Ogre::StringConverter::toString(mDetourCrowd->getMaxNbAgents())+" reached", Ogre::LML_CRITICAL);
        throw new Ogre::Exception(1, "Cannot create crowd agent for new character. Limit of "+Ogre::StringConverter::toString(mDetourCrowd->getMaxNbAgents())+" reached", "OgreRecastApplication::getOrCreateCharacter("+name+")");
    }

    if( HUMAN_CHARACTERS ) {
        // Create human characters
        Character *character = new AnimateableCharacter(name, mSceneMgr, mDetourCrowd, position);
        mCharacters.push_back(character);
        return character;
    } else {
        // Create simple characters (cylinders)
        Character *character = new TestCharacter(name, mSceneMgr, mDetourCrowd, position);
        mCharacters.push_back(character);
        return character;
    }
}

void OgreRecastApplication::drawPathBetweenMarkers(int pathNb, int targetId)
{
    try {
        Ogre::Vector3 beginPos = getOrCreateMarker("BeginPos")->getPosition();
        Ogre::Vector3 endPos = getOrCreateMarker("EndPos")->getPosition();

        // Draw path line if visual debugging is enabled
        if(OgreRecastApplication::DEBUG_DRAW)
            // Find new path from begin to end positions
            calculateAndDrawPath(beginPos, endPos, pathNb, targetId);

        // Show begin and end markers
        setPathAndBeginMarkerVisibility(true);
    } catch(Ogre::Exception ex) {
        // Either begin or end marker have not yet been placed
        return;
    }
}

void OgreRecastApplication::calculateAndDrawPath(Ogre::Vector3 beginPos, Ogre::Vector3 endPos, int pathNb, int targetId)
{
    // Note that this calculated path is not actually used except for debug drawing.
    // DetourCrowd will take care of calculating a separate path for each of its agents.
    int ret = mRecast->FindPath(beginPos, endPos, pathNb, targetId) ;
    if( ret >= 0 )
            mRecast->CreateRecastPathLine(pathNb) ; // Draw a line showing path at specified slot
    else
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a path. ("+mRecast->getPathFindErrorMsg(ret)+")");
}


void OgreRecastApplication::UpdateAllAgentDestinations()
{
    try {
        Ogre::Vector3 beginPos = getOrCreateMarker("BeginPos")->getPosition();
        Ogre::Vector3 endPos = getOrCreateMarker("EndPos")->getPosition();

        // Move first agent to begin position again
        mCharacters[0]->setPosition(beginPos);

        // Set move target for entire crowd
        setDestinationForAllAgents(endPos, false);
    } catch(Ogre::Exception ex) {
        // Either begin or end marker have not yet been placed
        return;
    }
}

void OgreRecastApplication::setPathAndBeginMarkerVisibility(bool visibility)
{
    // Hide or show begin marker
    getOrCreateMarker("BeginPos")->setVisible(visibility);

    // Hide or show path line
    if(mRecast->m_pRecastMOPath)
        mRecast->m_pRecastMOPath->setVisible(visibility);
}

bool OgreRecastApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    // First update the agents using the crowd in which they are controlled
    mDetourCrowd->updateTick(evt.timeSinceLastFrame);

    // Then update all characters controlled by the agents
    Ogre::Vector3 firstAgentPos = Ogre::Vector3::ZERO;
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        // Update character (position, animations, state)
        character->update(evt.timeSinceLastFrame);


        // Set new destinations depending on current demo mode

        // RANDOM WANDER BEHAVIOUR
        if(mApplicationState == CROWD_WANDER &&
           character->getAgentID() != 0) // No random walking for first agent
        {
            // If destination reached: Set new random destination
            if ( character->destinationReached() ) {
                character->updateDestination( mRecast->getRandomNavMeshPoint() );
            }
        }

        // CHASE BEHAVIOUR
        if(mApplicationState == FOLLOW_TARGET) {
            // We specifically need the position of the first agent for the others to chase
            if(character->getAgentID() == 0)
                firstAgentPos = character->getPosition();
            else
                // Only ADJUST path to follow target (no full recalculation of the corridor to follow)
                character->updateDestination(firstAgentPos, true);
        }
    }

    return BaseApplication::frameRenderingQueued(evt);
}

void OgreRecastApplication::setRandomTargetsForCrowd()
{
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        if(character->getAgentID() != 0) {  // Don't randomize first agent's destination
            character->updateDestination( mRecast->getRandomNavMeshPoint() );
        }
    }

}

void OgreRecastApplication::setFollowTargetForCrowd(Ogre::Vector3 targetDestination)
{
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        if(character->getAgentID() != 0) {  // Don't move first agent
            // Recalculate a full new path to the target
            character->updateDestination( targetDestination, false );
        }
    }
}

void OgreRecastApplication::setDestinationForAllAgents(Ogre::Vector3 destination, bool adjustExistingPath)
{
    mDetourCrowd->setMoveTarget(destination, adjustExistingPath);
        // TODO: there is a bug here that sometimes only the first agent will receive this target update

    // Update the destination variable of each character (uses friend relationship with Character)
    for(std::vector<Character*>::iterator iter = mCharacters.begin(); iter != mCharacters.end(); iter++) {
        (*iter)->setDestination(destination);   // This happens here because DetourCrowd does not manage Characters, only agents.
    }
}

bool OgreRecastApplication::rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject &foundMovable)
{
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

                mRecast->getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                              pentity->getParentNode()->_getDerivedPosition(),
                              pentity->getParentNode()->_getDerivedOrientation(),
                              pentity->getParentNode()->_getDerivedScale());
            } else {
                // For manualObjects
                // get the entity to check
                Ogre::ManualObject *pmanual = static_cast<Ogre::ManualObject*>(query_result[qr_idx].movable);

                mRecast->getManualMeshInformation(pmanual, vertex_count, vertices, index_count, indices,
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
