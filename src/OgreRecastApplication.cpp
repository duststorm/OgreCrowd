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
#include "RecastInputGeom.h"
#include <vector>


//--- SETTINGS ------------------------------------------------------------------------

// Set to true to draw debug objects. This is only the initial state of mDebugDraw, you can toggle using V key.
const bool OgreRecastApplication::DEBUG_DRAW = true;

// Set to true to show agents as animated human characters instead of cylinders
const bool OgreRecastApplication::HUMAN_CHARACTERS = true;

// Add extra obstacles (pots)
const bool OgreRecastApplication::OBSTACLES = true;

// Set to true to build simple single navmesh, set to false to build tiled navmesh using detourTileCache that supports temp obstacles
const bool OgreRecastApplication::SINGLE_NAVMESH = false;

//-------------------------------------------------------------------------------------


OgreRecastApplication::OgreRecastApplication(void)
        : mRecast(0),
        mRayScnQuery(0),
        mDetourCrowd(0),
        mApplicationState(SIMPLE_PATHFIND),
        mLabelOverlay(0),
        mCrosshair(0),
        mChaseCam(0),
        mMoveForwardKeyPressed(false),
        mMouseMoveX(0),
        mCharacters(),
        mDebugDraw(DEBUG_DRAW),
        mNavMeshNode(NULL),
        mDebugEntities(),
        mDetourTileCache(NULL)
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
    mCamera->setPosition(-46.3106, 62.3307, 40.7579);
    mCamera->setOrientation(Ogre::Quaternion(0.903189, -0.247085, - 0.338587, - 0.092626));

    // Create navigateable dungeon
    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    mapE->setMaterialName("dungeon");
    Ogre::SceneNode* mapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MapNode");
    mapNode->attachObject(mapE);

    // Add some obstacles
    Ogre::Entity* potE = NULL;
    Ogre::Entity *pot2ProxyE = NULL;
    if (OBSTACLES) {
        Ogre::SceneNode *potNode = mapNode->createChildSceneNode("PotNode");
        potE = mSceneMgr->createEntity("Pot1", "Pot.mesh");
        potNode->attachObject(potE);
        potNode->setScale(0.7, 0.7, 0.7);
        potNode->setPosition(6, 10, -20);

        Ogre::SceneNode *pot2Node = mapNode->createChildSceneNode("Pot2Node");
        Ogre::Entity *pot2E = mSceneMgr->createEntity("Pot2", "Pot.mesh");
        pot2Node->attachObject(pot2E);
        pot2Node->setScale(0.3, 0.3, 0.3);
        pot2Node->setPosition(0.926607, 10, -37.7362);
        // You can also create a proxy or collision mesh for the obstacle to use for building the navmesh
        // This can speed up navmesh creation as we are using simpler cylinder shapes
        Ogre::Vector3 potSize = pot2E->getBoundingBox().getSize();
        pot2ProxyE = mSceneMgr->createEntity("Pot2Proxy", "Cylinder.mesh");
        Ogre::SceneNode *pot2ProxyNode = pot2Node->createChildSceneNode("Pot2ProxyNode");
        pot2ProxyNode->attachObject(pot2ProxyE);
        pot2ProxyNode->setScale(potSize.x, potSize.y, potSize.z);
        pot2ProxyE->setVisible(mDebugDraw); // Hide collision mesh when not debug drawing
        mDebugEntities.push_back(pot2ProxyE);
    }


    // Create list of entities to build navmesh from
    std::vector<Ogre::Entity*> navmeshEnts;
    navmeshEnts.push_back(mapE);  // Add the map

    // You should tweak your navmesh build max error and maxclimb parameter to properly detect smaller obstacles
    if (OBSTACLES) {
        navmeshEnts.push_back(potE);  // Add obstacle
        navmeshEnts.push_back(pot2ProxyE);  // Add proxy or collision mesh for obstacle

        // Create some additional obstacles
        navmeshEnts.push_back( createObstacle("Pot3", Ogre::Vector3(44.2139, 10, -4.70583), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot4", Ogre::Vector3(40.3481, 10, 7.46006), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot5", Ogre::Vector3(37.9414, 10, 6.12506), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot6", Ogre::Vector3(2.98811, 10, -32.6629), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot7", Ogre::Vector3(-2.97011, 10, -33.819), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot8", Ogre::Vector3(-1.17544, 10, -37.0419), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot9", Ogre::Vector3(0.926607, 10, -37.7362), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot10", Ogre::Vector3(18.9451, 10.2355, 0.95), Ogre::Vector3(0.3, 0.3, 0.3)) );
        navmeshEnts.push_back( createObstacle("Pot11", Ogre::Vector3(18.2158, 10.2355, 4), Ogre::Vector3(0.3, 0.3, 0.3)) );
    }


    // RECAST (navmesh creation)
    // Create the navmesh and show it
    mRecast = new OgreRecast(mSceneMgr);
    if(SINGLE_NAVMESH) {
        // Simple recast navmesh build example

        if(mRecast->NavMeshBuild(navmeshEnts)) {
            mRecast->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
            return;
        }
    // DetourTileCache navmesh creation
    } else {
        // More advanced: use DetourTileCache to build a tiled and cached navmesh that can be updated with dynamic obstacles at runtime.

        mDetourTileCache = new OgreDetourTileCache(mRecast);
        if(mDetourTileCache->TileCacheBuild(navmeshEnts)) {
            mDetourTileCache->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh using detourTileCache.");
            return;
        }
    }


    // DETOUR (pathfinding)
    // Do a pathing between two random points on the navmesh and draw the path
    // Note that because we are using DetourCrowd we will not be doing pathfinds directly, DetourCrowd
    // will do this for us.
    int pathNb = 0;     // The index number for the slot in which the found path is to be stored
    int targetId = 0;   // Number identifying the target the path leads to
    Ogre::Vector3 beginPos = mRecast->getRandomNavMeshPoint();
    Ogre::Vector3 endPos = mRecast->getRandomNavMeshPoint();
    if(OgreRecastApplication::mDebugDraw)
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
    if(mDebugDraw) {
        beginPos.y = beginPos.y + mRecast->m_navMeshOffsetFromGround;
        endPos.y = endPos.y + mRecast->m_navMeshOffsetFromGround;
    }
    getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen")->setPosition(beginPos);
    getOrCreateMarker("EndPos", "Cylinder/Wires/Brown")->setPosition(endPos);


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);


    // SETUP RAY SCENE QUERYING
    // Used for mouse picking begin and end markers and determining the position to add new agents
    // Add navmesh to separate querying group that we will use
    mNavMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    //mNavMeshNode->getAttachedObject("RecastMOWalk0")->setQueryFlags(NAVMESH_MASK);    // Will be queryable by default, no need to explicitly set it.
    // Exclude other meshes from navmesh queries
    mNavMeshNode->getAttachedObject("RecastMONeighbour0")->setQueryFlags(DEFAULT_MASK);
    mNavMeshNode->getAttachedObject("RecastMOBoundary0")->setQueryFlags(DEFAULT_MASK);
// TODO
    Ogre::uint32 flags = mapE->getQueryFlags();
    mapE->setQueryFlags(DEFAULT_MASK);
    potE->setQueryFlags(DEFAULT_MASK);
    pot2ProxyE->setQueryFlags(DEFAULT_MASK);

    if(!OgreRecastApplication::mDebugDraw)
        mNavMeshNode->setVisible(false); // Even though we make it invisible, we still keep the navmesh entity in the scene to do ray intersection tests


    // CREATE CURSOR OVERLAY
    mCrosshair = Ogre::OverlayManager::getSingletonPtr()->getByName("GUI/Crosshair");
    mCrosshair->show(); // Show a cursor in the center of the screen


    // SETUP AGENT STEERING DEMO
    // Create a chase camera to follow the first agent
    Ogre::SceneNode *node = mCharacters[0]->getNode();
    mChaseCam = mSceneMgr->createCamera("AgentFollowCamera");
    mChaseCam->setNearClipDistance(0.1);
    node->attachObject(mChaseCam);
    mChaseCam->setPosition(0, mDetourCrowd->getAgentHeight(), mDetourCrowd->getAgentRadius()*4);
    mChaseCam->pitch(Ogre::Degree(-15));
    Ogre::Viewport *vp = mWindow->getViewport(0);
    mChaseCam->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}

bool OgreRecastApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    // Handle buffered input for controlling first agent in steering demo mode
    if(mApplicationState == STEER_AGENT) {
        // This is needed because we do not know in what order mouse and key events are signalled by OIS
        // If key release and mouse move happen in the wrong order an agent can continue walking even though the walk key was released

        Character *firstAgent = mCharacters[0];
        bool agentIsMoving = firstAgent->isMoving();

        if(!mMoveForwardKeyPressed && agentIsMoving) {
            firstAgent->stop(); // Stop a moving agent if forward key is released
        } else if(mMouseMoveX != 0) {
            firstAgent->getNode()->yaw(Ogre::Degree(mMouseMoveX)); // Rotate character according to mouse move
            if(mMoveForwardKeyPressed)
                firstAgent->moveForward();  // Update move direction if mouse was moved
        } else if(mMoveForwardKeyPressed && !agentIsMoving) {
            // Mouse is not moved, but forward key is pressed and agent is not moving
            firstAgent->moveForward();  // Start moving agent
        }
        mMouseMoveX = 0;    // Reset mouse input
    }

    // Update DetourTileCache (handle dynamic changes to navmesh)
    if(!SINGLE_NAVMESH && mDetourTileCache)
        mDetourTileCache->handleUpdate(evt.timeSinceLastFrame);

    // First update the agents using the crowd in which they are controlled
    if(mDetourCrowd)
        mDetourCrowd->updateTick(evt.timeSinceLastFrame);

    // Then update all characters controlled by the agents
    Ogre::Vector3 firstAgentPos = Ogre::Vector3::ZERO;
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        // Update character (position, animations, state)
        character->update(evt.timeSinceLastFrame);


        // Set new destinations depending on current demo mode

        // RANDOM WANDER BEHAVIOUR
        if( (mApplicationState == CROWD_WANDER || mApplicationState == STEER_AGENT) &&
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

Ogre::Entity* OgreRecastApplication::createObstacle(Ogre::String name, Ogre::Vector3 position, Ogre::Vector3 scale) {
    Ogre::SceneNode *mapNode = mSceneMgr->getSceneNode("MapNode");
    Ogre::SceneNode *node = mapNode->createChildSceneNode(name+"Node");
    Ogre::Entity *ent = mSceneMgr->createEntity(name, "Pot.mesh");
    node->attachObject(ent);
    node->setPosition(position);
    node->setScale(scale);
    ent->setQueryFlags(DEFAULT_MASK);   // Exclude from ray queries

    return ent;
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
    if (mApplicationState == STEER_AGENT)
        return true;

    // Do ray scene query
    //send a raycast straight out from the camera at the center position
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(0.5, 0.5);

    Ogre::Vector3 rayHitPoint;
    Ogre::MovableObject *rayHitObject;
    if (rayQueryPointInScene(mouseRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {
        // Compensate for the fact that the ray-queried navmesh is drawn a little above the ground
        rayHitPoint.y = rayHitPoint.y - mRecast->m_navMeshOffsetFromGround;

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
            if(mDebugDraw)
                rayHitPoint.y = rayHitPoint.y + mRecast->m_navMeshOffsetFromGround;
            markerNode->setPosition(rayHitPoint);
        }

        if(mApplicationState == SIMPLE_PATHFIND) {
            drawPathBetweenMarkers(1,1);    // Draw navigation path (in DRAW_DEBUG) and begin marker
            UpdateAllAgentDestinations();   //Steer all agents to set destination
        }
    }

    BaseApplication::mousePressed(arg, id);
}

bool OgreRecastApplication::mouseMoved(const OIS::MouseEvent &arg)
{
    if (mApplicationState == STEER_AGENT) {
        // Buffer mouse move input in agent steering demo mode
        mMouseMoveX = -arg.state.X.rel;
        return true;
    }

    return BaseApplication::mouseMoved(arg);
}

bool OgreRecastApplication::keyPressed( const OIS::KeyEvent &arg )
{
    // SPACE places a new agent at cursor position
    if(arg.key == OIS::KC_SPACE
       && mApplicationState != STEER_AGENT  // no adding agents in steering mode (no mouse pointer)
       && mDetourCrowd->getNbAgents() < mDetourCrowd->getMaxNbAgents()) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (rayQueryPointInScene(cursorRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {
            //Ogre::LogManager::getSingletonPtr()->logMessage("Info: added agent at position "+Ogre::StringConverter::toString(rayHitPoint));

            Character *character = createCharacter("Agent"+Ogre::StringConverter::toString(mCharacters.size()), rayHitPoint);

            // If in wander mode, give a random destination to agent (otherwise it will take the destination of the previous agent automatically)
            if(mApplicationState == CROWD_WANDER || mApplicationState == STEER_AGENT) {
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
                mCrosshair->show();
                mWindow->getViewport(0)->setCamera(mCamera);
                break;
            case CROWD_WANDER:      // Wander -> chase
                mApplicationState = FOLLOW_TARGET;
                setFollowTargetForCrowd(mCharacters[0]->getPosition());
                setPathAndBeginMarkerVisibility(false);
                mLabelOverlay->setCaption("Chase");
                mCrosshair->show();
                mWindow->getViewport(0)->setCamera(mCamera);
                break;
            case FOLLOW_TARGET:     // Chase -> steer
                mApplicationState = STEER_AGENT;
                setRandomTargetsForCrowd();
                setPathAndBeginMarkerVisibility(false);
                mDetourCrowd->stopAgent(0); // Stop first agent
                mLabelOverlay->setCaption("Steer agent");
                mCrosshair->hide(); // Hide crosshair
                mWindow->getViewport(0)->setCamera(mChaseCam);  // Set chase camera on first agent
                break;
            case STEER_AGENT:       // Steer -> simple
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);    // Draw navigation path (in DRAW_DEBUG) and begin marker
                UpdateAllAgentDestinations();   // Reinitialize path that all agents follow and steer them towards end marker
                mLabelOverlay->setCaption("Simple navigation");
                mCrosshair->show();
                mWindow->getViewport(0)->setCamera(mCamera);
                break;
            default:
                mApplicationState = SIMPLE_PATHFIND;
                drawPathBetweenMarkers(1,1);
                UpdateAllAgentDestinations();
                mLabelOverlay->setCaption("Simple navigation");
                mCrosshair->show();
                mWindow->getViewport(0)->setCamera(mCamera);
        }
    }

    // Backspace adds a temporary obstacle to the navmesh (in dtTileCache mode)
    if(!SINGLE_NAVMESH && mApplicationState != STEER_AGENT && arg.key == OIS::KC_BACK) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (rayQueryPointInScene(cursorRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {
            dtObstacleRef oRef = mDetourTileCache->addTempObstacle(rayHitPoint);
            if (oRef) {
                // Add visualization of temp obstacle
                Ogre::SceneNode *obstacleMarker = getOrCreateMarker("TempObstacle"+Ogre::StringConverter::toString(oRef));
                obstacleMarker->setPosition(rayHitPoint);
                obstacleMarker->setVisible(mDebugDraw);
                obstacleMarker->setScale(TEMP_OBSTACLE_RADIUS, TEMP_OBSTACLE_HEIGHT, TEMP_OBSTACLE_RADIUS);
                Ogre::Entity *obstacleEnt = (Ogre::Entity*)obstacleMarker->getAttachedObject(0);
                obstacleEnt->setQueryFlags(DEFAULT_MASK);  // exclude from navmesh queries
                mDebugEntities.push_back(obstacleEnt);
            }
        }
    }

    // Delete removes a temporary obstacle from the navmesh (in dtTileCache mode)
    if(!SINGLE_NAVMESH && mApplicationState != STEER_AGENT && arg.key == OIS::KC_DELETE) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (rayQueryPointInScene(cursorRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {
            dtObstacleRef oRef = mDetourTileCache->removeTempObstacle(cursorRay.getOrigin(), rayHitPoint);
            if (oRef) {
                // Add visualization of temp obstacle
                Ogre::SceneNode *obstacleMarker = getOrCreateMarker("TempObstacle"+Ogre::StringConverter::toString(oRef));
                Ogre::Entity *obstacleEnt = (Ogre::Entity*)obstacleMarker->getAttachedObject(0);
                // Remove debug entity
                mDebugEntities.erase(remove(mDebugEntities.begin(), mDebugEntities.end(), obstacleEnt), mDebugEntities.end());
                obstacleEnt->detachFromParent();
                mSceneMgr->destroyEntity(obstacleEnt);
            }
        }
    }

    // V key shows or hides recast debug drawing
    if(arg.key == OIS::KC_V) {
        mDebugDraw = !mDebugDraw;
        setDebugVisibility(mDebugDraw);
    }

    // Buffer input of W key for controlling first agent in steering demo mode
    if (arg.key == OIS::KC_W)
        mMoveForwardKeyPressed = true;

    // Disable regular camera movement in agent steering demo mode
    if( mApplicationState == STEER_AGENT &&
        (arg.key == OIS::KC_W ||
         arg.key == OIS::KC_A ||
         arg.key == OIS::KC_S ||
         arg.key == OIS::KC_D ||
         arg.key == OIS::KC_PGUP ||
         arg.key == OIS::KC_PGDOWN  ))
        return true;

    return BaseApplication::keyPressed(arg);
}

bool OgreRecastApplication::keyReleased(const OIS::KeyEvent &arg)
{
    // Buffer input of W key for controlling first agent in steering demo mode
    if (arg.key == OIS::KC_W)
        mMoveForwardKeyPressed = false;

    return BaseApplication::keyReleased(arg);
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
        Character *character = new AnimateableCharacter(name, mSceneMgr, mDetourCrowd, mDebugDraw, position);
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
        if(OgreRecastApplication::mDebugDraw)
            // Find new path from begin to end positions
            calculateAndDrawPath(beginPos, endPos, pathNb, targetId);

        // Show begin and end markers
        setPathAndBeginMarkerVisibility(mDebugDraw);
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
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a (full) path ("+mRecast->getPathFindErrorMsg(ret)+"). It's possible there is a partial path.");
        // We dont bother with partial paths as this is only for debug drawing. DetourCrowd handles this for us anyway.
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

bool OgreRecastApplication::rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject **foundMovable)
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

                InputGeom::getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                                              pentity->getParentNode()->_getDerivedPosition(),
                                              pentity->getParentNode()->_getDerivedOrientation(),
                                              pentity->getParentNode()->_getDerivedScale());
            } else {
                // For manualObjects
                // get the entity to check
                Ogre::ManualObject *pmanual = static_cast<Ogre::ManualObject*>(query_result[qr_idx].movable);

                InputGeom::getManualMeshInformation(pmanual, vertex_count, vertices, index_count, indices,
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
        *foundMovable = closest_movable;
        return (true);
    }
    else
    {
        // raycast failed
        return (false);
    }

}

void OgreRecastApplication::setDebugVisibility(bool visible)
{
    mDebugDraw = visible;

    mNavMeshNode->setVisible(visible);

    if (mApplicationState == SIMPLE_PATHFIND)
        drawPathBetweenMarkers(1, 1);
    else
        setPathAndBeginMarkerVisibility(false);

    // Change visibility of all registered debug entities for the application
    for(std::vector<Ogre::Entity*>::iterator iter = mDebugEntities.begin(); iter != mDebugEntities.end(); iter++) {
        Ogre::Entity *ent = *iter;
        ent->setVisible(visible);
    }

    for(std::vector<Character*>::iterator iter = mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;
        character->setDebugVisibility(visible);
    }

    Ogre::SceneNode* beginMarker = getOrCreateMarker("BeginPos");
    Ogre::SceneNode* endMarker = getOrCreateMarker("EndPos");
    Ogre::Vector3 beginPos= beginMarker->getPosition();
    Ogre::Vector3 endPos= endMarker->getPosition();
    if (visible) {
        beginPos.y = beginPos.y + mRecast->m_navMeshEdgesOffsetFromGround;
        endPos.y = endPos.y + mRecast->m_navMeshEdgesOffsetFromGround;
    } else {
        beginPos.y = beginPos.y - mRecast->m_navMeshEdgesOffsetFromGround;
        endPos.y = endPos.y - mRecast->m_navMeshEdgesOffsetFromGround;
    }
    beginMarker->setPosition(beginPos);
    endMarker->setPosition(endPos);
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
