#include "include/OgreRecastTerrainApplication.h"
#include <Terrain/OgreTerrain.h>

OgreRecastTerrainApplication::OgreRecastTerrainApplication()
    : mTerrainsImported(false),
      mTerrainGroup(NULL),
      mTerrainGlobals(NULL)
{
}

OgreRecastTerrainApplication::~OgreRecastTerrainApplication()
{
}


void OgreRecastTerrainApplication::createScene()
{
    // Basic scene setup
    Ogre::Vector3 lightdir(0.55, -0.3, 0.75);
    lightdir.normalise();
    Ogre::Light* light = mSceneMgr->createLight("MainLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDirection(lightdir);
    light->setDiffuseColour(Ogre::ColourValue::White);
    light->setSpecularColour(Ogre::ColourValue(0.4, 0.4, 0.4));
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
    mCamera->setPosition(-3161.99, 866.029, -6214.35);
    mCamera->setOrientation(Ogre::Quaternion(0.21886, -0.0417, -0.9576, -0.1826));
    mCamera->setNearClipDistance(0.1);
    mCamera->setFarClipDistance(50000);
    if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
    {
        mCamera->setFarClipDistance(0);   // enable infinite far clip distance if we can
    }

    // Set viewport color to blue (makes bounding boxes more visible)
    Ogre::Viewport *vp = mWindow->getViewport(0);
    vp->setBackgroundColour(Ogre::ColourValue(13.0/255,221.0/255,229.0/255));


    // Set the default ray query mask for any created scene object
    Ogre::MovableObject::setDefaultQueryFlags(DEFAULT_MASK);

    // Create navigateable terrain
    mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();
    mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z, 513, 12000.0f);
    mTerrainGroup->setFilenameConvention(Ogre::String("OgreRecastTerrain"), Ogre::String("dat"));
    mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);
    configureTerrainDefaults(light);

    for (long x = 0; x <= 0; ++x)
        for (long y = 0; y <= 0; ++y)
            defineTerrain(x, y);

    // sync load since we want everything in place when we start
    mTerrainGroup->loadAllTerrains(true);

    if (mTerrainsImported)
    {
        Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
        while(ti.hasMoreElements())
        {
            Ogre::Terrain* t = ti.getNext()->instance;
            initBlendMaps(t);
        }
    }

    mTerrainGroup->freeTemporaryResources();

// TODO extract recast/detour parameters out of their wrappers, use config classes
    // RECAST (navmesh creation)
    // Create the navmesh and show it
    mRecast = new OgreRecast(mSceneMgr);
    InputGeom *geom = new InputGeom(mTerrainGroup);
    // Debug draw the recast bounding box around the terrain tile
    ConvexVolume bb = ConvexVolume(geom->getBoundingBox());
    InputGeom::drawConvexVolume(&bb, mSceneMgr);
    // Verify rasterized terrain mesh
//    geom->debugMesh(mSceneMgr);
    if(SINGLE_NAVMESH) {
        // Simple recast navmesh build example

        if(mRecast->NavMeshBuild(geom)) {
            mRecast->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
            return;
        }

    // DetourTileCache navmesh creation
    } else {
        // More advanced: use DetourTileCache to build a tiled and cached navmesh that can be updated with dynamic obstacles at runtime.

        mDetourTileCache = new OgreDetourTileCache(mRecast);
        if(mDetourTileCache->TileCacheBuild(geom)) {
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
    mCameraMan->setTopSpeed(1000);


    // SETUP RAY SCENE QUERYING AND DEBUG DRAWING
    // Used for mouse picking begin and end markers and determining the position to add new agents
    // Add navmesh to separate querying group that we will use
    mNavMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

// TODO raycast terrain
/*
    if (RAYCAST_SCENE)
        mapE->setQueryFlags(NAVMESH_MASK);
*/

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
    mChaseCam->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}

void OgreRecastTerrainApplication::configureTerrainDefaults(Ogre::Light* light)
{
    // Configure global
    mTerrainGlobals->setMaxPixelError(8);   // Precision of terrain
    // testing composite map
    mTerrainGlobals->setCompositeMapDistance(3000);     // Max distance of terrain to be rendered

    // Important to set these so that the terrain knows what to use for derived (non-realtime) data
    mTerrainGlobals->setLightMapDirection(light->getDerivedDirection());
    mTerrainGlobals->setCompositeMapAmbient(mSceneMgr->getAmbientLight());
    mTerrainGlobals->setCompositeMapDiffuse(light->getDiffuseColour());

    // Configure default import settings for if we use imported image
    Ogre::Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
    defaultimp.terrainSize = 513;
    defaultimp.worldSize = 12000.0f;
    defaultimp.inputScale = 600; // due terrain.png is 8 bpp
    defaultimp.minBatchSize = 33;
    defaultimp.maxBatchSize = 65;

    // textures
    defaultimp.layerList.resize(3);
    defaultimp.layerList[0].worldSize = 100;
    defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
    defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
    defaultimp.layerList[1].worldSize = 30;
    defaultimp.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
    defaultimp.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
    defaultimp.layerList[2].worldSize = 200;
    defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
    defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");
}


/**
  * Flipping is done to imitate seamless terrain so you can create unlimited terrain using a
  * single 513x513 heightmap, it's just a trick.
  **/
void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
    img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (flipX)
        img.flipAroundY();
    if (flipY)
        img.flipAroundX();

}

void OgreRecastTerrainApplication::defineTerrain(long x, long y)
{
    Ogre::String filename = mTerrainGroup->generateFilename(x, y);
    if (Ogre::ResourceGroupManager::getSingleton().resourceExists(mTerrainGroup->getResourceGroup(), filename))
    {
        mTerrainGroup->defineTerrain(x, y);
    }
    else
    {
        Ogre::Image img;
        getTerrainImage(x % 2 != 0, y % 2 != 0, img);
        mTerrainGroup->defineTerrain(x, y, &img);
        mTerrainsImported = true;
    }
}


void OgreRecastTerrainApplication::initBlendMaps(Ogre::Terrain* terrain)
{
    Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
    Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);
    Ogre::Real minHeight0 = 70;
    Ogre::Real fadeDist0 = 40;
    Ogre::Real minHeight1 = 70;
    Ogre::Real fadeDist1 = 15;
    float* pBlend0 = blendMap0->getBlendPointer();
    float* pBlend1 = blendMap1->getBlendPointer();
    for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y)
    {
        for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x)
        {
            Ogre::Real tx, ty;

            blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
            Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
            Ogre::Real val = (height - minHeight0) / fadeDist0;
            val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
            *pBlend0++ = val;

            val = (height - minHeight1) / fadeDist1;
            val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
            *pBlend1++ = val;
        }
    }
    blendMap0->dirty();
    blendMap1->dirty();
    blendMap0->update();
    blendMap1->update();
}


bool OgreRecastTerrainApplication::keyPressed( const OIS::KeyEvent &arg )
{
    if(  arg.key == OIS::KC_O
      || arg.key == OIS::KC_BACK
      || arg.key == OIS::KC_DELETE
      || arg.key == OIS::KC_K
      || arg.key == OIS::KC_I)
        return BaseApplication::keyPressed(arg);    // Avoid these features from the dungeon demo

    // Use X to test terrain height
    if(arg.key == OIS::KC_X) {
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);

        // Perform the scene query
        Ogre::TerrainGroup::RayResult result = mTerrainGroup->rayIntersects(cursorRay);
        if(result.hit) {
            Ogre::Real terrainHeight = result.position.y;
            Ogre::LogManager::getSingletonPtr()->logMessage("Terrain height: "+ Ogre::StringConverter::toString(terrainHeight));
        }
    }

    return OgreRecastApplication::keyPressed(arg);

    // Make sure that any redrawn navmesh tiles have the proper query mask
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

    // SPACE places a new agent at cursor position
    if(arg.key == OIS::KC_SPACE
//       && mApplicationState != STEER_AGENT  // no adding agents in steering mode (no mouse pointer)
       && mDetourCrowd->getNbAgents() < mDetourCrowd->getMaxNbAgents()) {
        // Find position on navmesh pointed to by cursor in the middle of the screen
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);
        Ogre::Vector3 rayHitPoint;
        Ogre::MovableObject *rayHitObject;
        if (rayQueryPointInScene(cursorRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {
            if ( Ogre::StringUtil::startsWith(rayHitObject->getName(), "recastmowalk", true) ) {
                // Compensate for the fact that the ray-queried navmesh is drawn a little above the ground
                rayHitPoint.y = rayHitPoint.y - mRecast->m_navMeshOffsetFromGround;
            } else {
                // Queried point was not on navmesh, find nearest point on the navmesh
                mRecast->findNearestPointOnNavmesh(rayHitPoint, rayHitPoint);
            }

            Ogre::LogManager::getSingletonPtr()->logMessage("Info: added agent at position "+Ogre::StringConverter::toString(rayHitPoint));

            Character *character = createCharacter("Agent"+Ogre::StringConverter::toString(mCharacters.size()), rayHitPoint);

            // If in wander mode, give a random destination to agent (otherwise it will take the destination of the previous agent automatically)
            if(mApplicationState == CROWD_WANDER || mApplicationState == STEER_AGENT) {
                character->updateDestination(mRecast->getRandomNavMeshPoint(), false);
            }
        }
    }

    return BaseApplication::keyPressed(arg);
}

bool OgreRecastTerrainApplication::keyReleased(const OIS::KeyEvent &arg)
{
    return BaseApplication::keyReleased(arg);
}

void OgreRecastTerrainApplication::destroyScene(void)
{
    OGRE_DELETE mTerrainGroup;
    OGRE_DELETE mTerrainGlobals;
}

bool OgreRecastTerrainApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    // Make sure that any redrawn navmesh tiles have the proper query mask
    for (int i = 0; i < mNavMeshNode->numAttachedObjects(); i++) {
        Ogre::MovableObject *obj = mNavMeshNode->getAttachedObject(i);
        obj->setQueryFlags(NAVMESH_MASK);
    }

    // Do ray scene query
    //send a raycast straight out from the camera at the center position
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(0.5, 0.5);

    Ogre::Vector3 rayHitPoint;
    Ogre::MovableObject *rayHitObject;
    if (rayQueryPointInScene(mouseRay, NAVMESH_MASK, rayHitPoint, &rayHitObject)) {

        if ( Ogre::StringUtil::startsWith(rayHitObject->getName(), "recastmowalk", true) ) {
            // Compensate for the fact that the ray-queried navmesh is drawn a little above the ground
            rayHitPoint.y = rayHitPoint.y - mRecast->m_navMeshOffsetFromGround;
        } else {
            // Queried point was not on navmesh, find nearest point on the navmesh
            mRecast->findNearestPointOnNavmesh(rayHitPoint, rayHitPoint);
        }

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

    return BaseApplication::mousePressed(arg, id);
}


Character* OgreRecastTerrainApplication::createCharacter(Ogre::String name, Ogre::Vector3 position)
{
    Character* character = OgreRecastApplication::createCharacter(name, position);
    // Enable terrain clipping for character
    character->clipToTerrain(mTerrainGroup);
    return character;
}
