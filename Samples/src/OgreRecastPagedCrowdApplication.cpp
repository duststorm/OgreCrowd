/*
    OgreCrowd
    ---------

    Copyright (c) 2012 Jonas Hauquier

    Additional contributions by:

    - mkultra333
    - Paul Wilson

    Sincere thanks and to:

    - Mikko Mononen (developer of Recast navigation libraries)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#include "OgreRecastPagedCrowdApplication.h"
#include "OgreRecastApplication.h"
#include "AnimateableCharacter.h"
#include "TestCharacter.h"
#include "RecastInputGeom.h"
#include "OgreRecastNavmeshPruner.h"
#include "InstancedCharacter.h"
#include "CrowdManager.h"


// TODO fix bug where a lot of agents are located on world origin

// TODO extract CrowdInstancer class

// TODO incorporate instancing + hardware skinning

// TODO manage sets of character types, and when instancing also manage their fitting instanceManagers

// TODO larger scale demo (and more interesting scene)

// TODO make crowd wandering more purposeful by use of waypoints, maybe make the exact behaviour configurable


// TODO consider this alternative way of placing new agents that walked off the grid: place them on the outer edge of the page, not just on a border tile



// TODO actual loaded navmesh grid (in detourTileCache) should be at least one tile larger than the one loaded with crowd agents, so that agents can effectively walk out of the paged area (and reappear somewhere else)!! Document/enforce this


const Ogre::Real OgreRecastPagedCrowdApplication::TOPDOWN_CAMERA_HEIGHT = 80;
const bool OgreRecastPagedCrowdApplication::EXTRACT_WALKABLE_AREAS = true;


OgreRecastPagedCrowdApplication::OgreRecastPagedCrowdApplication()
    : mRecast(0)
    , mNavMeshNode(0)
    , mDetourTileCache(0)
    , mDebugEntities()
    , mTopDownCamera(false)
    , mGoingUp(false)
    , mGoingDown(false)
    , mGoingLeft(false)
    , mGoingRight(false)
    , mDebugDraw(false)
{
    CrowdManager::HUMAN_CHARACTERS = OgreRecastApplication::HUMAN_CHARACTERS;
}


void OgreRecastPagedCrowdApplication::createScene(void)
{
    // Basic scene setup
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight( "MainLight" );
    light->setPosition(20, 80, 50);
    mCamera->setPosition(-46.3106, 62.3307, 40.7579);
    mCamera->setOrientation(Ogre::Quaternion(0.903189, -0.247085, - 0.338587, - 0.092626));

    // Create world plane
    Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingletonPtr()->createPlane("GroundPlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                      Ogre::Plane(Ogre::Vector3::UNIT_Y, 0), 100, 100, 50, 50, true, 1, 1.0f, 1.0f, Ogre::Vector3::UNIT_Z);
    Ogre::SceneNode *planeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("GroundPlaneNode");
    Ogre::Entity *planeEnt = mSceneMgr->createEntity(planeMesh);
    planeEnt->setMaterialName("number_grid");
    planeNode->attachObject(planeEnt);

    // Create list of entities to build navmesh from
    mNavmeshEnts.push_back(planeEnt);  // Add the ground plane

    // Add some random boxes
    // (well, not completely random as we need to know which area to mark as "walkable")
    Ogre::Real rotations[15] = {175.122f, -106.854f, 88.3572f, 65.4711f, -20.0449f, -14.5578f, -127.722f, 20.4753f, -42.361f, 58.9315f, 129.366f, -110.749f, 82.9775f, -57.9149f, 55.1907f};
    Ogre::Vector3 positions[15] = {Ogre::Vector3(-24.3478, 0, -6.92418), Ogre::Vector3(-13.3589, 0, -10.2301), Ogre::Vector3(3.05032, 0, -26.2014), Ogre::Vector3(8.69059, 0 ,-39.3912),
                                   Ogre::Vector3(-14.8654, 0, 10.5836), Ogre::Vector3(2.96917, 0, -38.2776), Ogre::Vector3(35.6507, 0, -7.31114), Ogre::Vector3(1.67711, 0, 11.7132),
                                   Ogre::Vector3(19.2971, 0, 30.7451), Ogre::Vector3(25.6568, 0, -38.9435), Ogre::Vector3(-21.2673, 0, 8.05908), Ogre::Vector3(8.50251, 0, -33.025),
                                   Ogre::Vector3(22.3011, 0, 23.2297), Ogre::Vector3(22.9099, 0, -37.1353), Ogre::Vector3(-6.50643, 0, -22.5862) };
    for(int i = 0; i < 15; i++) {
        Ogre::Entity *boxEnt = mSceneMgr->createEntity("Box_"+Ogre::StringConverter::toString(i), "Box.mesh");
        Ogre::SceneNode *boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(boxEnt->getName()+"_Node");
        boxNode->attachObject(boxEnt);
        //boxNode->setPosition(Ogre::Math::RangeRandom(-40, 40), 0, Ogre::Math::RangeRandom(-40, 40));
        boxNode->setPosition(positions[i]);
        if(EXTRACT_WALKABLE_AREAS)
            boxNode->setScale(10, 10, 10);
        else
            boxNode->setScale(10, 2.5, 10);    // Don't make boxes higher than agents, or they can be placed inside boxes!
        //boxNode->yaw(Ogre::Degree(Ogre::Math::RangeRandom(0, 360)));
        boxNode->yaw(Ogre::Radian(rotations[i]));

        mNavmeshEnts.push_back(boxEnt);     // Add to navmesh generation input
    }


    // RECAST (navmesh creation)
    mRecast = new OgreRecast(mSceneMgr);    // Use default configuration
    // Using DetourTileCache
    // More advanced: use DetourTileCache to build a tiled and cached navmesh that can be updated with dynamic obstacles at runtime.

    mDetourTileCache = new OgreDetourTileCache(mRecast);
    if(mDetourTileCache->TileCacheBuild(mNavmeshEnts)) {
        mDetourTileCache->drawNavMesh();
    } else {
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh using detourTileCache.");
        return;
    }

    if(EXTRACT_WALKABLE_AREAS) {
        // Mark walkable area (where agents will be spawned)
        OgreRecastNavmeshPruner *navMeshPruner = mRecast->getNavmeshPruner();
        // Start tracing at the origin position on the navmesh, and include all areas that are reachable from there (there is no box in the center)
        navMeshPruner->floodNavmesh(Ogre::Vector3::ZERO);
        navMeshPruner->pruneSelected();
    }


    // Setup crowd manager
    mCrowdManager = new CrowdManager(mDetourTileCache, mSceneMgr, mCamera);


    setDebugVisibility(true);


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);
}

void OgreRecastPagedCrowdApplication::setDebugVisibility(bool visible)
{
    mDebugDraw = visible;

    mCrowdManager->setDebugVisibility(visible);

//    mNavMeshNode->setVisible(visible);

    // Change visibility of all registered debug entities for the application
    for(std::vector<Ogre::Entity*>::iterator iter = mDebugEntities.begin(); iter != mDebugEntities.end(); iter++) {
        Ogre::Entity *ent = *iter;
        ent->setVisible(visible);
    }
}

bool OgreRecastPagedCrowdApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    // Perform navmesh updates
    mDetourTileCache->handleUpdate(evt.timeSinceLastFrame);

    // Update crowd
    mCrowdManager->update(evt.timeSinceLastFrame);


    // Update top-down Camera
    if(mTopDownCamera) {
        // build our acceleration vector based on keyboard input composite
        Ogre::Vector3 accel = Ogre::Vector3::ZERO;

        if(mGoingUp) accel += mCamera->getUp();
        if(mGoingDown) accel -= mCamera->getUp();
        if(mGoingLeft) accel -= mCamera->getRight();
        if(mGoingRight) accel += mCamera->getRight();

        if(!accel.isZeroLength()) {
            accel.normalise();
            accel *= (mCameraMan->getTopSpeed()/ 20);
            mCamera->setPosition(mCamera->getPosition()+accel);
        }
    }

    // Update debug info panel
    updateDebugInfo();

    return BaseApplication::frameRenderingQueued(evt);
}


void OgreRecastPagedCrowdApplication::createFrameListener()
{
    BaseApplication::createFrameListener();

    // CREATE PAGED CROWD DEBUG PANEL
    Ogre::FontManager::getSingleton().getByName("SdkTrays/Caption")->load();    // Fixes a bug with SDK Trays
    Ogre::StringVector items;
    items.push_back("Total agents");        // 0
    items.push_back("Loaded agents");       // 1
    items.push_back("Paged grid");          // 2
    items.push_back("Loaded tiles");        // 3
    items.push_back("Border tiles");        // 4
    items.push_back("Camera type");         // 5
    mDebugPanel = mTrayMgr->createParamsPanel(OgreBites::TL_TOPLEFT, "PagedCrowdDebugPanel", 200, items);
    if(mDebugDraw)
        mDebugPanel->show();
}

void OgreRecastPagedCrowdApplication::updateDebugInfo()
{
    if(mDebugPanel->isVisible()) {
        mDebugPanel->setParamValue(0, Ogre::StringConverter::toString(mCrowdManager->getSize()));                 // Total agents
        mDebugPanel->setParamValue(1, Ogre::StringConverter::toString(mCrowdManager->getNbAssignedAgents())); // Loaded agents
        Ogre::String dimensionStr = Ogre::StringConverter::toString(mCrowdManager->getGridDimensions());
        mDebugPanel->setParamValue(2, dimensionStr+"x"+dimensionStr);                               // Grid size
        mDebugPanel->setParamValue(3, Ogre::StringConverter::toString(mCrowdManager->getNbLoadedTiles()));         // Loaded tiles
        mDebugPanel->setParamValue(4, Ogre::StringConverter::toString(mCrowdManager->getNbBorderTiles()));         // Border tiles
        mDebugPanel->setParamValue(5, mTopDownCamera?"Top-down":"Free");                            // Camera type
    }
}


bool OgreRecastPagedCrowdApplication::keyPressed(const OIS::KeyEvent &arg)
{
    // Change camera mode
    if(arg.key == OIS::KC_RETURN) {
        mTopDownCamera = !mTopDownCamera;

        if(mTopDownCamera) {
            mCameraMan->setStyle(OgreBites::CS_MANUAL);
            Ogre::Vector3 cameraPos = mCamera->getPosition();
            cameraPos.y = TOPDOWN_CAMERA_HEIGHT;
            mCamera->setPosition(cameraPos);
            mCamera->setFixedYawAxis(false);
            mCamera->setDirection(-Ogre::Vector3::UNIT_Y);
        } else {
            mCamera->setFixedYawAxis(true);
            mCameraMan->setStyle(OgreBites::CS_FREELOOK);
        }

        if(mDebugPanel->isVisible())
            mDebugPanel->setParamValue(5, mTopDownCamera?"Top-down":"Free");              // Camera type
    }

    // Override camera movement in top-down camera mode
    if (arg.key == OIS::KC_W || arg.key == OIS::KC_UP) mGoingUp = true;
    else if (arg.key == OIS::KC_S || arg.key == OIS::KC_DOWN) mGoingDown= true;
    else if (arg.key == OIS::KC_A || arg.key == OIS::KC_LEFT) mGoingLeft = true;
    else if (arg.key == OIS::KC_D || arg.key == OIS::KC_RIGHT) mGoingRight = true;

    return BaseApplication::keyPressed(arg);
}

bool OgreRecastPagedCrowdApplication::keyReleased(const OIS::KeyEvent &arg)
{
    // Override camera movement in top-down camera mode
    if (arg.key == OIS::KC_W || arg.key == OIS::KC_UP) mGoingUp = false;
    else if (arg.key == OIS::KC_S || arg.key == OIS::KC_DOWN) mGoingDown= false;
    else if (arg.key == OIS::KC_A || arg.key == OIS::KC_LEFT) mGoingLeft = false;
    else if (arg.key == OIS::KC_D || arg.key == OIS::KC_RIGHT) mGoingRight = false;

    return BaseApplication::keyReleased(arg);
}
