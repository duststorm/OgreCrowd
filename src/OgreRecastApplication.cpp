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
#include "ConvexShapeObstacle.h"
#include "CylinderObstacle.h"
#include "RecastInputGeom.h"
#include "OgreRecastTerrainApplication.h"
#include "SettingsFileParser.h"


//--- SETTINGS ------------------------------------------------------------------------

// These are the default settings and can be overridden with a OgreRecast.cfg configuration file

// Set to true to draw debug objects. This is only the initial state of mDebugDraw, you can toggle using V key.
bool OgreRecastApplication::DEBUG_DRAW = true;

// Set to true to show agents as animated human characters instead of cylinders
bool OgreRecastApplication::HUMAN_CHARACTERS = true;

// Add extra obstacles to the scene (pots)
bool OgreRecastApplication::OBSTACLES = true;

// Set to true to build simple single navmesh, set to false to build tiled navmesh using detourTileCache that supports temp obstacles
// Note that for terrain it's not a good idea to construct a single navmesh, as it's much slower
bool OgreRecastApplication::SINGLE_NAVMESH = false;
// Set to true to also query dungeon mesh when clicking to set begin position or destination
bool OgreRecastApplication::RAYCAST_SCENE = false;

// Set to true to use a temp obstacle in the agent steering demo instead of an agent with velocity (only works when SINGLE_NAVMESH is false)
bool OgreRecastApplication::TEMP_OBSTACLE_STEERING = true;

// Set to true to place boxes as convex obstacles on the navmesh instead of the cylindrical temporary obstacles
bool OgreRecastApplication::COMPLEX_OBSTACLES = true;

// Set to true to demo navmesh on terrain
bool OgreRecastApplication::TERRAIN = false;

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
        mDetourTileCache(NULL),
        mGateHull(0),
        mGate(0),
        mGateClosed(false)
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

    // Set the default ray query mask for any created scene object
    Ogre::MovableObject::setDefaultQueryFlags(DEFAULT_MASK);

    // Create navigateable dungeon
    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    Ogre::SceneNode* mapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MapNode");
    mapNode->attachObject(mapE);

    // Add gate
    Ogre::Entity *gateE = mSceneMgr->createEntity("Gate", "Gate.mesh");
    mGate = mSceneMgr->getRootSceneNode()->createChildSceneNode("GateNode");
    mGate->attachObject(gateE);
    mGate->setPosition(-9.22, 0, 0);
    mGateClosed = true;
    if(SINGLE_NAVMESH)
       // No temporary obstacles available, add gate to the navmesh input
        mNavmeshEnts.push_back(gateE);

    // Add some obstacles (demonstrates that you can build a navmesh from multiple entities)
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
    mNavmeshEnts.push_back(mapE);  // Add the map

    // You should tweak your navmesh build max error and maxclimb parameter to properly detect smaller obstacles
    if (OBSTACLES) {
        mNavmeshEnts.push_back(potE);  // Add obstacle
        mNavmeshEnts.push_back(pot2ProxyE);  // Add proxy or collision mesh for obstacle

        // Create some additional obstacles
        mNavmeshEnts.push_back( createObstacle("Pot3", Ogre::Vector3(44.2139, 10, -4.70583), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot4", Ogre::Vector3(40.3481, 10, 7.46006), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot5", Ogre::Vector3(37.9414, 10, 6.12506), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot6", Ogre::Vector3(2.98811, 10, -32.6629), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot7", Ogre::Vector3(-2.97011, 10, -33.819), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot8", Ogre::Vector3(-1.17544, 10, -37.0419), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot9", Ogre::Vector3(0.926607, 10, -37.7362), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot10", Ogre::Vector3(18.9451, 10.2355, 0.95), Ogre::Vector3(0.3, 0.3, 0.3)) );
        mNavmeshEnts.push_back( createObstacle("Pot11", Ogre::Vector3(18.2158, 10.2355, 4), Ogre::Vector3(0.3, 0.3, 0.3)) );
    }


    // RECAST (navmesh creation)
    // Create the navmesh and show it
    mRecast = new OgreRecast(mSceneMgr);    // Use default configuration
    if(SINGLE_NAVMESH) {
        // Simple recast navmesh build example

        if(mRecast->NavMeshBuild(mNavmeshEnts)) {
            mRecast->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
            return;
        }
    // DetourTileCache navmesh creation
    } else {
        // More advanced: use DetourTileCache to build a tiled and cached navmesh that can be updated with dynamic obstacles at runtime.

        mDetourTileCache = new OgreDetourTileCache(mRecast);
        if(mDetourTileCache->TileCacheBuild(mNavmeshEnts)) {
            mDetourTileCache->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh using detourTileCache.");
            return;
        }

        // Create a convex obstacle for the gate using its world-coordinate bounding box
        mGateHull = new ConvexVolume(InputGeom::getWorldSpaceBoundingBox(gateE), mRecast->getAgentRadius());
        // Note: it's important to choose a proper area type you want to mark with the polygon! I just set it to "unwalkable"
        mGateHull->area = RC_NULL_AREA;   // Set area described by convex polygon to "unwalkable"
        mGateHull->hmin = mGateHull->hmin - 0.3;    // Extend a bit downwards so it hits the ground (navmesh) for certain. (Maybe this is not necessary)
        mDetourTileCache->addConvexShapeObstacle(mGateHull);
        InputGeom::drawConvexVolume(mGateHull, mSceneMgr);
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
        beginPos.y = beginPos.y + mRecast->getNavmeshOffsetFromGround();
        endPos.y = endPos.y + mRecast->getNavmeshOffsetFromGround();
    }
    getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen")->setPosition(beginPos);
    getOrCreateMarker("EndPos", "Cylinder/Wires/Brown")->setPosition(endPos);


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);


    // SETUP RAY SCENE QUERYING AND DEBUG DRAWING
    // Used for mouse picking begin and end markers and determining the position to add new agents
    // Add navmesh to separate querying group that we will use
    mNavMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

    if (RAYCAST_SCENE || OgreRecast::STATIC_GEOM_DEBUG) // also when using static geometry because it's hard to ray query static geometry
        mapE->setQueryFlags(NAVMESH_MASK);

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

    // Update DetourTileCache (handle dynamic changes to navmesh such as temp obstacles)
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

    // Make sure that any redrawn navmesh tiles have the proper query mask
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

    Ogre::Vector3 rayHitPoint;
    //send a raycast straight out from the camera at the center position
    if(queryCursorPosition(rayHitPoint)) {

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
                rayHitPoint.y = rayHitPoint.y + mRecast->getNavmeshOffsetFromGround();
            markerNode->setPosition(rayHitPoint);
        }

        if(mApplicationState == SIMPLE_PATHFIND) {
            drawPathBetweenMarkers(1,1);    // Draw navigation path (in DRAW_DEBUG) and begin marker
            UpdateAllAgentDestinations();   //Steer all agents to set destination
        }
    }

    return BaseApplication::mousePressed(arg, id);
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
    // Make sure that any redrawn navmesh tiles have the proper query mask
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

    // SPACE places a new agent at cursor position
    if(arg.key == OIS::KC_SPACE
       && mApplicationState != STEER_AGENT  // no adding agents in steering mode (no mouse pointer)
       && mDetourCrowd->getNbAgents() < mDetourCrowd->getMaxNbAgents()) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint)) {

            Ogre::LogManager::getSingletonPtr()->logMessage("Info: added agent at position "+Ogre::StringConverter::toString(rayHitPoint));

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
                if (!SINGLE_NAVMESH && TEMP_OBSTACLE_STEERING) {
                    // Use temporary obstacle for player collision avoidance
                    // Player has absolute control
                    mLabelOverlay->setCaption("Steer obstacle");
                    mCharacters[0]->setDetourTileCache(mDetourTileCache);
                    mCharacters[0]->setAgentControlled(false);
                } else {
                    // Steer an agent in the crowd (less control by the player)
                    mLabelOverlay->setCaption("Steer agent");
                }
                mCrosshair->hide(); // Hide crosshair
                mWindow->getViewport(0)->setCamera(mChaseCam);  // Set chase camera on first agent
                break;
            case STEER_AGENT:       // Steer -> simple
                mApplicationState = SIMPLE_PATHFIND;
                if (!SINGLE_NAVMESH && TEMP_OBSTACLE_STEERING)
                    mCharacters[0]->setAgentControlled(true);  // Restore agent controlled state to first character
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
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint)) {

            Ogre::LogManager::getSingletonPtr()->logMessage("Adding obstacle on point "+Ogre::StringConverter::toString(rayHitPoint));

            if(COMPLEX_OBSTACLES) {
                // Place a convex hull obstacle on the navmesh

                // First check whether not too many convex obstacles were created already
                if(mObstacles.size() < InputGeom::MAX_VOLUMES)
                    mObstacles.push_back(new ConvexShapeObstacle(rayHitPoint, mDetourCrowd->getAgentRadius(), mDetourTileCache));
                        // Create convex hull with agent radios offset around the object (this is important so agents don't walk through the edges of the obstacle!)
            } else {
                // Place simple cylindrical temporary obstacles

                // There is no limit on the number of cylindrical temp obstacles, only a limit on the number add/remove requests per time period (MAX_REQUESTS)
                mObstacles.push_back(new CylinderObstacle(rayHitPoint, mDetourTileCache));
            }
        }
    }

    // Delete removes a temporary obstacle from the navmesh (in dtTileCache mode)
    if(!SINGLE_NAVMESH && mApplicationState != STEER_AGENT && arg.key == OIS::KC_DELETE) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (queryCursorPosition(rayHitPoint, OBSTACLE_MASK, false, &rayHitObject)) {

            // Find the obstacle associated with the hit entity
            Obstacle *obst = NULL;
            for(std::vector<Obstacle*>::iterator iter = mObstacles.begin(); iter != mObstacles.end(); iter++) {
                if((*iter)->getEntity() == rayHitObject) {
                    obst = *iter;
                    break;
                }
            }

            if(!obst)
                return true;

            // Remove obstacle from obstacles list
            mObstacles.erase(remove(mObstacles.begin(), mObstacles.end(), obst), mObstacles.end());

            // Call destructor on obstacle (will also remove it from tilecache)
            delete obst;
        }
    }

    // K key adds a wooden pallet to the scene that can be climbed by agents
    if(arg.key == OIS::KC_K && !SINGLE_NAVMESH) {
        // Find position on any geometry pointed to by cursor in the middle of the screen
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint, DEFAULT_MASK, false)) {

            // Create a little height offset for placing the obstacle
            rayHitPoint.y += 0.3;

            Ogre::Entity* palletE =  mSceneMgr->createEntity("WoodPallet.mesh");
            Ogre::SceneNode* palletN =  mSceneMgr->getRootSceneNode()->createChildSceneNode();
            palletN->attachObject(palletE);
            palletN->setPosition(rayHitPoint);
            // Good care needs to be taken with the size of obstacles and the navmesh generation properties
            // Note that in this case it would probably be better to create a simple bounding box shape for the pallet, to not have troube with the holes in the pallet.
            palletN->setScale(0.4, 0.7, 0.4);
            mWalkableObjects.push_back(palletE);

            // Create a list containing both navmesh entities and obstacles, as well as all created walkable objects
            std::vector<Ogre::Entity*> detourInputs;
            detourInputs.insert(detourInputs.end(), mNavmeshEnts.begin(), mNavmeshEnts.end());
            detourInputs.insert(detourInputs.end(), mWalkableObjects.begin(), mWalkableObjects.end());
// TODO I actually want this inputgeom to be reused also when adding a (convex) temp obstacle, but maybe the best way is adding a proper addConvexObstacle method to ogreDtTileCache

            // Rebuild the tiles overlapping the bounding box of the added object
            Ogre::AxisAlignedBox bb = InputGeom::getWorldSpaceBoundingBox(palletE);

            // Enable to see the tile area that was rebuilt
            OgreDetourTileCache::DEBUG_DRAW_REBUILT_BB = true;

            mDetourTileCache->buildTiles(detourInputs, &bb);
        }
    }

    // I key removes the wooden pallet that is pointed to by the cursor
    if(arg.key == OIS::KC_I && !SINGLE_NAVMESH) {
        // Find position on any geometry pointed to by cursor in the middle of the screen
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (queryCursorPosition(rayHitPoint, DEFAULT_MASK, false, &rayHitObject)) {

            // If we hit a pallet, remove it
            if( std::find(mWalkableObjects.begin(), mWalkableObjects.end(), rayHitObject) != mWalkableObjects.end() ) {
                mWalkableObjects.erase(std::remove(mWalkableObjects.begin(), mWalkableObjects.end(), rayHitObject));

                // Create a list containing both navmesh entities and obstacles, as well as all created walkable objects
                std::vector<Ogre::Entity*> detourInputs;
                detourInputs.insert(detourInputs.end(), mNavmeshEnts.begin(), mNavmeshEnts.end());
                detourInputs.insert(detourInputs.end(), mWalkableObjects.begin(), mWalkableObjects.end());
                                            // the hit pallet is removed from this list

                // Rebuild the tiles overlapping the bounding box, with the pallet removed
                Ogre::AxisAlignedBox bb = InputGeom::getWorldSpaceBoundingBox(rayHitObject);

                mDetourTileCache->buildTiles(detourInputs, &bb);  // Update tile again, this time without the pallet

                // Remove pallet entity from scene and destroy it
                rayHitObject->getParentSceneNode()->detachObject(rayHitObject);
                mSceneMgr->destroyEntity((Ogre::Entity*)rayHitObject);
            }
        }
    }

    // O key opens or closes the gate
    if(arg.key == OIS::KC_O && !SINGLE_NAVMESH) {
        if(mGateClosed) {
            // Open gate
            mDetourTileCache->removeConvexShapeObstacle(mGateHull);
            Ogre::Vector3 pos = mGate->getPosition();
            pos.y += mDetourCrowd->getAgentHeight()+ 0.3;
            mGate->setPosition(pos);
            mGateClosed = false;
        } else {
            // Close gate
            mDetourTileCache->addConvexShapeObstacle(mGateHull);
            Ogre::Vector3 pos = mGate->getPosition();
            pos.y -= mDetourCrowd->getAgentHeight() +0.3;
            mGate->setPosition(pos);
            mGateClosed = true;
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



    // Debug function: test which navmesh tiles are near the queried point
    if(!SINGLE_NAVMESH && arg.key == OIS::KC_X) {
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint)) {

            std::vector<dtTileRef> tiles = mDetourTileCache->getTilesAroundPoint(rayHitPoint, 1);
            Ogre::String strTiles = "";
            for (std::vector<dtTileRef>::iterator iter = tiles.begin(); iter != tiles.end(); iter++) {
                dtTileRef tile = *iter;
                strTiles = strTiles + " " + Ogre::StringConverter::toString(tile);
            }
            Ogre::LogManager::getSingletonPtr()->logMessage("Found tiles for point "+Ogre::StringConverter::toString(rayHitPoint)+": "+ strTiles);
        }
    }

    // For testing obstacle transformations
    if(arg.key == OIS::KC_T) {
        // Rotate obstacles
        for (std::vector<Obstacle*>::iterator iter = mObstacles.begin(); iter != mObstacles.end(); iter++) {
            Obstacle *obst = *iter;

            // Rotate obstacle
            Ogre::Degree rotation = Ogre::Degree(obst->getOrientation().getYaw());
            rotation += Ogre::Degree(25); // Rotate 25 degrees
            obst->updateOrientation(Ogre::Quaternion(rotation, Ogre::Vector3::UNIT_Y));

            // Move obstacle
//            Ogre::Vector3 pos = obst->getPosition() + Ogre::Vector3::UNIT_X;
//            obst->updatePosition(pos);
        }
    }



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
        result->setScale(mRecast->getAgentRadius()*2, mRecast->getAgentHeight(),mRecast->getAgentRadius()*2);

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
    // Set destination for all agents in the crowd
    mDetourCrowd->setMoveTarget(destination, adjustExistingPath);
        // TODO: there is a bug here that sometimes only the first agent will receive this target update

    // Update the destination variable of each character to reflect the change to the crowd agents (uses friend relationship with Character)
    for(std::vector<Character*>::iterator iter = mCharacters.begin(); iter != mCharacters.end(); iter++) {
        (*iter)->setDestination(destination);   // This happens here because DetourCrowd does not manage Characters, only agents.
    }
}

bool OgreRecastApplication::queryCursorPosition(Ogre::Vector3 &rayHitPoint, unsigned long queryflags, bool clipToNavmesh, Ogre::MovableObject **rayHitObject)
{
    // Do ray scene query
    //send a raycast straight out from the camera at the center position
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(0.5, 0.5);

    Ogre::MovableObject *hitObject;
    if (rayQueryPointInScene(mouseRay, queryflags, rayHitPoint, &hitObject)) {

        if ( Ogre::StringUtil::startsWith(hitObject->getName(), "recastmowalk", true) ) {
            // Compensate for the fact that the ray-queried navmesh is drawn a little above the ground
            rayHitPoint.y = rayHitPoint.y - mRecast->getNavmeshOffsetFromGround();
        } else if(clipToNavmesh) {
            // Queried point was not on navmesh, find nearest point on the navmesh (if not possible returns exact point)
            mRecast->findNearestPointOnNavmesh(rayHitPoint, rayHitPoint);
        }

        // Pass pointer to hit movable
        if (rayHitObject)
            *rayHitObject = hitObject;

        return true;
    }

    return false;
}

bool OgreRecastApplication::rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject **foundMovable)
{
// TODO: destroy queries using scenemgr::destroyRayQuery or reuse one query object by storing it in a member variable
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
        beginPos.y = beginPos.y + mRecast->getNavmeshOffsetFromGround();
        endPos.y = endPos.y + mRecast->getNavmeshOffsetFromGround();
    } else {
        beginPos.y = beginPos.y - mRecast->getNavmeshOffsetFromGround();
        endPos.y = endPos.y - mRecast->getNavmeshOffsetFromGround();
    }
    beginMarker->setPosition(beginPos);
    endMarker->setPosition(endPos);
}


void OgreRecastApplication::loadConfig(Ogre::String configFileName)
{
    std::cout << "Loading settings from " << configFileName << std::endl;

    try {
        SettingsFileParser sfp = SettingsFileParser(configFileName);

        BaseApplication::RESTORE_CONFIG = sfp.mRestoreConfig;
        BaseApplication::DISABLE_MOUSE_GRAB = !sfp.mLockMouse;

        OgreRecastApplication::DEBUG_DRAW = sfp.mDebugDraw;
        OgreRecastApplication::HUMAN_CHARACTERS = sfp.mHumanChars;
        OgreRecastApplication::OBSTACLES = sfp.mObstacles;
        OgreRecastApplication::SINGLE_NAVMESH = sfp.mSingleNavmesh;
        OgreRecastApplication::RAYCAST_SCENE = sfp.mRaycastScene;
        OgreRecastApplication::TEMP_OBSTACLE_STEERING = sfp.mTempObstacleSteering;
        OgreRecastApplication::COMPLEX_OBSTACLES = sfp.mComplexObstacles;
        OgreRecastApplication::TERRAIN = sfp.mTerrain;

        OgreRecastTerrainApplication::TERRAIN_TILES_X = sfp.mTerrainTilesX;
        OgreRecastTerrainApplication::TERRAIN_TILES_Z = sfp.mTerrainTilesZ;
        OgreRecastTerrainApplication::TERRAIN_TILE_SIZE = sfp.mTerrainTileSize;
        OgreRecastTerrainApplication::TERRAIN_HEIGHT_SCALE = sfp.mTerrainHeightScale;
        OgreRecastTerrainApplication::TERRAIN_TILE_RESOLUTION = sfp.mTerrainTileResolution;
    } catch (Ogre::Exception e) {
        std::cout << "WARNING: Could not find file " << configFileName << ". Using default settings." << std::endl;
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
        // Load configuration settings from cfg file
        OgreRecastApplication::loadConfig("OgreRecast.cfg");

        // Create application object
        OgreRecastApplication *app = NULL;
        if(OgreRecastApplication::TERRAIN) {
            app = new OgreRecastTerrainApplication();
        } else {
            app = new OgreRecastApplication();
        }

        try {
            app->go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }
        delete app;

        return 0;
    }

#ifdef __cplusplus
}
#endif
