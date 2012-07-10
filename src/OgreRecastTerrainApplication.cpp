#include "OgreRecastTerrainApplication.h"
#include <Terrain/OgreTerrain.h>


// Number of terrain tiles to generate. If setting higher than 1, make sure that terrain
// tile size is not too high. The total size of the world is still limited because of allocation
// limits in detourTileCache
size_t OgreRecastTerrainApplication::TERRAIN_TILES_X = 1;
size_t OgreRecastTerrainApplication::TERRAIN_TILES_Z = 1;

// TODO make detourTileCache coordinates localized to area currently in screen

// World size of one terrain tile
float OgreRecastTerrainApplication::TERRAIN_TILE_SIZE = 12000.0f;
// Determines number of vertices in one terrain tile
Ogre::uint16 OgreRecastTerrainApplication::TERRAIN_TILE_RESOLUTION = 513;

// Scale of terrain height, relative to terrain tile size. Set to 1 for regular scale.
float OgreRecastTerrainApplication::TERRAIN_HEIGHT_SCALE = 1.0f;


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
    float terrainScaleX = TERRAIN_TILES_X * TERRAIN_TILE_SIZE/2;
    float terrainScaleZ = TERRAIN_TILES_Z * TERRAIN_TILE_SIZE/2;
    float terrainHeightScale = TERRAIN_TILE_SIZE/20 * TERRAIN_HEIGHT_SCALE;
    mCamera->setPosition(-terrainScaleX/1.897539208, terrainHeightScale/0.667386703, -terrainScaleZ/0.965507253);
//    mCamera->setPosition(-3161.99, 866.029, -6214.35);
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
    mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(mSceneMgr, Ogre::Terrain::ALIGN_X_Z, TERRAIN_TILE_RESOLUTION, TERRAIN_TILE_SIZE);
    mTerrainGroup->setFilenameConvention(Ogre::String("OgreRecastTerrain"), Ogre::String("dat"));
    mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);
    configureTerrainDefaults(light);

    for (long x = 0; x < TERRAIN_TILES_X; ++x)
        for (long y = 0; y < TERRAIN_TILES_Z; ++y)
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

    // RECAST (navmesh creation)
    mGeom = new InputGeom(mTerrainGroup);
    // Debug draw the recast bounding box around the terrain tile
    ConvexVolume bb = ConvexVolume(mGeom->getBoundingBox());
    InputGeom::drawConvexVolume(&bb, mSceneMgr);

    // Uncomment to verify rasterized terrain mesh
//    mGeom->debugMesh(mSceneMgr);

    if(SINGLE_NAVMESH) {
        // Simple recast navmesh build example
        // For large terrain meshes this is not recommended, as the build takes a very long time

        // Initialize custom navmesh parameters
        OgreRecastConfigParams recastParams = OgreRecastConfigParams();
        recastParams.setCellSize(50);
        recastParams.setCellHeight(6);
        recastParams.setAgentMaxSlope(45);
        recastParams.setAgentHeight(2.5);
        recastParams.setAgentMaxClimb(15);
        recastParams.setAgentRadius(0.5);
        recastParams.setEdgeMaxLen(2);
        recastParams.setEdgeMaxError(1.3);
        recastParams.setRegionMinSize(50);
        recastParams.setRegionMergeSize(20);
        recastParams.setDetailSampleDist(5);
        recastParams.setDetailSampleMaxError(5);

        mRecast = new OgreRecast(mSceneMgr, recastParams);

        if(mRecast->NavMeshBuild(mGeom)) {
            mRecast->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
            return;
        }

    // DetourTileCache navmesh creation
    } else {
        // More advanced: use DetourTileCache to build a tiled and cached navmesh that can be updated with dynamic obstacles at runtime.

        // Initialize custom navmesh parameters
        OgreRecastConfigParams recastParams = OgreRecastConfigParams();
        recastParams.setCellSize(0.3);
        recastParams.setCellHeight(0.5);
        recastParams.setAgentMaxSlope(45);
        recastParams.setAgentHeight(2.5);
        recastParams.setAgentMaxClimb(0.5);
        recastParams.setAgentRadius(0.5);
//        recastParams.setEdgeMaxLen(2);
//        recastParams.setEdgeMaxError(1.3);
//        recastParams.setRegionMinSize(50);
//        recastParams.setRegionMergeSize(20);
//        recastParams.setDetailSampleDist(5);
//        recastParams.setDetailSampleMaxError(5);

        mRecast = new OgreRecast(mSceneMgr, recastParams);

        // Optimize debug drawing batch count
        OgreRecast::STATIC_GEOM_DEBUG = true;

        // Make it quiet to avoid spamming the log
        OgreRecast::VERBOSE = false;

        mDetourTileCache = new OgreDetourTileCache(mRecast, 48);
        mDetourTileCache->configure(mGeom);
        Ogre::AxisAlignedBox areaToLoad;

        // Select a small area to build the initial part of the navmesh from
        areaToLoad.setMinimum(Ogre::Vector3(-TERRAIN_TILE_SIZE/5, 0, -TERRAIN_TILE_SIZE/2));
        areaToLoad.setMaximum(Ogre::Vector3(-TERRAIN_TILE_SIZE/5.217, terrainHeightScale, -TERRAIN_TILE_SIZE/2.0689));

        // Disable dtTileCache debug drawing to improve performance.
        // Don't do this when OgreRecast::STATIC_GEOM_DEBUG is false as the
        // demo will depend on the navmesh geometry being in the scene!
//        mDetourTileCache->DEBUG_DRAW = false;

        mDetourTileCache->buildTiles(mGeom, &areaToLoad);    // Only build a few tiles

        // This builds all of the tiles in the navmesh. Unmanageable with such big terrain sizes
        /*
        if(mDetourTileCache->TileCacheBuild(geom)) {
            mDetourTileCache->drawNavMesh();
        } else {
            Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh using detourTileCache.");
            return;
        }
        */
    }

    // You can save out the input geom to wavefront .obj like so:
    // This can be used, for example, for testing in the official demo that comes with recast
//    if(mRecast->m_geom)
//        mRecast->m_geom->writeObj("terrainTestObj.obj");
//    else
//        mDetourTileCache->m_geom->writeObj("terrainTestObj.obj");


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
    mCameraMan->setTopSpeed(500);


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
    defaultimp.terrainSize = TERRAIN_TILE_RESOLUTION;
    defaultimp.worldSize = TERRAIN_TILE_SIZE;
    defaultimp.inputScale = TERRAIN_HEIGHT_SCALE * TERRAIN_TILE_SIZE/20; // was originally 600 due terrain.png is 8 bpp (with tile world size 12000)
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
    // Build navmesh around cursor position
    if( arg.key == OIS::KC_N) {
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint, NAVMESH_MASK, false)) {
            // Size of the area to build around cursor
            float boxSize = 50;

            Ogre::AxisAlignedBox box;
            box.setMinimum(rayHitPoint.x - boxSize/2, mRecast->getConfig().bmin[1], rayHitPoint.z - boxSize/2);
            box.setMaximum(rayHitPoint.x + boxSize/2, mRecast->getConfig().bmax[1], rayHitPoint.z + boxSize/2);

// TODO stop storing the complete inputGeom, query terrain and needed entities when necessary
// TODO eventually you will run out of memory (either because of GPU memory of inefficient debug drawing geometry, or CPU ram due to tilecache and navmesh becoming too large). Add features for caching out unused tiles.
            // Build tiles around cursor
            mDetourTileCache->buildTiles(mGeom, &box);
        }
    }

    // Place house
    if( arg.key == OIS::KC_H) {
        Ogre::Vector3 rayHitPoint;
        if (queryCursorPosition(rayHitPoint, NAVMESH_MASK, false)) {
            Ogre::Entity *houseEnt = NULL;
            if(Ogre::Math::RangeRandom(0,2) < 1) {
                houseEnt = mSceneMgr->createEntity("tudorhouse.mesh");
                Ogre::SceneNode *houseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
                houseNode->attachObject(houseEnt);
                houseNode->rotate(Ogre::Vector3::UNIT_Y, Ogre::Degree(Ogre::Math::RangeRandom(0, 360)));
                float houseHeight = houseEnt->getBoundingBox().getSize().y;
                float scale = 0.02;
                houseNode->setScale(scale, scale, scale);
                rayHitPoint.y += ( houseHeight/2 ) * scale; // tudorhouse mesh doesn't have its origin on the ground
                houseNode->setPosition(rayHitPoint);
            } else {
                houseEnt = mSceneMgr->createEntity("highlanderhouse.01.mesh");
                Ogre::SceneNode *houseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
                houseNode->attachObject(houseEnt);
                houseNode->rotate(Ogre::Vector3::UNIT_Y, Ogre::Degree(Ogre::Math::RangeRandom(0, 360)));
                float scale = 2.5;
                houseNode->setScale(scale, scale, scale);
                houseNode->setPosition(rayHitPoint);
            }
            mNavmeshEnts.push_back(houseEnt);

            // Get bounding box of the entity. All navmesh tiles touching it will have to be rebuilt.
            Ogre::AxisAlignedBox bb = InputGeom::getWorldSpaceBoundingBox(houseEnt);
            bb = mDetourTileCache->getTileAlignedBox(bb);

            // Create partial input geom only for the tiles we want to rebuild
            InputGeom geom = InputGeom(bb, mTerrainGroup, mNavmeshEnts);

            // Rebuild tiles that touch inputGeom bounding box
            mDetourTileCache->buildTiles(&geom);
        }
    }



    // Avoid these features from the dungeon demo
    if(  arg.key == OIS::KC_O   // There is no door outdoor! :)
      || arg.key == OIS::KC_K   // Walkable geometry works with terrain sample too, the only problem is that
      || arg.key == OIS::KC_I)  //      currently agents are clipped to terrain height.
        return BaseApplication::keyPressed(arg);

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

    // Resort to original recast demo functionality
    return OgreRecastApplication::keyPressed(arg);
}

void OgreRecastTerrainApplication::destroyScene(void)
{
    OGRE_DELETE mTerrainGroup;
    OGRE_DELETE mTerrainGlobals;
}

bool OgreRecastTerrainApplication::queryCursorPosition(Ogre::Vector3 &rayHitPoint, unsigned long queryflags, bool clipToNavmesh, Ogre::MovableObject **rayHitObject)
{
    if ( (OgreRecast::STATIC_GEOM_DEBUG || OgreRecastApplication::RAYCAST_SCENE)
            && queryflags == NAVMESH_MASK ) {
        // Raycast terrain
        Ogre::Ray cursorRay = mCamera->getCameraToViewportRay(0.5, 0.5);

        // Perform the scene query
        Ogre::TerrainGroup::RayResult result = mTerrainGroup->rayIntersects(cursorRay);
        if(result.hit) {
            // Queried point was not on navmesh, find nearest point on the navmesh
            if (clipToNavmesh && mRecast->findNearestPointOnNavmesh(result.position, rayHitPoint) ) {
                return true;
            } else {
                rayHitPoint = result.position;
                return true;
            }

        } else {
            return false;
        }
    } else {
        return OgreRecastApplication::queryCursorPosition(rayHitPoint, queryflags, clipToNavmesh, rayHitObject);
    }
}


Character* OgreRecastTerrainApplication::createCharacter(Ogre::String name, Ogre::Vector3 position)
{
    Character* character = OgreRecastApplication::createCharacter(name, position);
    // Enable terrain clipping for character
    character->clipToTerrain(mTerrainGroup);
    return character;
}

bool OgreRecastTerrainApplication::frameRenderingQueued(const Ogre::FrameEvent &evt)
{
    // Update navmesh debug drawing
    mRecast->update();

    // Make sure line drawings are hidden if debug is off
    mNavMeshNode->setVisible(mDebugDraw);

    return OgreRecastApplication::frameRenderingQueued(evt);
}

void OgreRecastTerrainApplication::setDebugVisibility(bool visible)
{
    OgreRecastApplication::setDebugVisibility(visible);

    if(OgreRecast::STATIC_GEOM_DEBUG) {
        try {
            Ogre::StaticGeometry *sg = mSceneMgr->getStaticGeometry("NavmeshDebugStaticGeom");
            sg->setVisible(visible);
        } catch (Ogre::Exception e) {
            // Do nothing
        }
    }
}
