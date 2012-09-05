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

#include "include/OgreRecastPagedCrowdApplication.h"
#include "OgreRecastApplication.h"
#include "AnimateableCharacter.h"
#include "TestCharacter.h"
#include "RecastInputGeom.h"
#include "OgreRecastNavmeshPruner.h"

const Ogre::Real OgreRecastPagedCrowdApplication::CROWD_PAGE_UPDATE_DELTA = 1;
const bool OgreRecastPagedCrowdApplication::EXTRACT_WALKABLE_AREAS = true;


// TODO prune unreachable areas (and increase height of boxes again) (https://groups.google.com/forum/#!topic/recastnavigation/SKbh93zsP5M   and   http://digestingduck.blogspot.be/2010/03/sketch-for-hierarchical-pathfinding.html    and    https://groups.google.com/forum/?fromgroups=#!topic/recastnavigation/iUvLGMokj20   and    https://groups.google.com/forum/?fromgroups=#!topic/recastnavigation/WLMRcDjsxFc)

// TODO extract CrowdInstancer class

// TODO larger scale demo (and more interesting scene)

// TODO incorporate instancing + hardware skinning

// TODO make crowd wandering more purposeful by use of waypoints, maybe make the exact behaviour configurable

// TODO fix bug where agents gets removed and re-added constantly

// TODO make crowd wander more robust (periodically check liveliness of agents so they don't get stuck)


// TODO this can also be DetourCrowd::MAX_AGENTS or allow setting of Max agents in detourCrowd
const Ogre::Real OgreRecastPagedCrowdApplication::MAX_CROWD_SIZE = 50;

const Ogre::Real OgreRecastPagedCrowdApplication::RADIUS_EPSILON = 1;

const Ogre::Real OgreRecastPagedCrowdApplication::TOPDOWN_CAMERA_HEIGHT = 80;

// TODO actual loaded navmesh grid (in detourTileCache) should be at least one tile larger than the one loaded with crowd agents, so that agents can effectively walk out of the paged area (and reappear somewhere else)!! Document/enforce this


OgreRecastPagedCrowdApplication::OgreRecastPagedCrowdApplication()
    : mRecast(0)
    , mDetourCrowd(0)
    , mCharacters()
    , mNavMeshNode(0)
    , mDebugEntities()
    , mDetourTileCache(0)
    , mCurrentlyPagedArea()
    , mPagedAreaDistance(2)
    , mNbPagedTiles(0)
    , mNbTilesInBorder(0)
    , mAreaDebug(0)
    , mTopDownCamera(false)
    , mGoingUp(false)
    , mGoingDown(false)
    , mGoingLeft(false)
    , mGoingRight(false)
    , mDebugDraw(false)
    , mBorderTiles()
{
    // Number of tiles filled with agents
    if(mPagedAreaDistance == 0) {
        mNbPagedTiles = 1;
        mNbTilesInBorder = 1;
        mDimension = 1;
    } else {
        // Dimensions of the square grid (dimension x dimension)
        mDimension = (2*(mPagedAreaDistance+1))-1;
        mNbPagedTiles = mDimension*mDimension;

        // Number of tiles in the outer border
        mNbTilesInBorder = (2*mDimension) + 2*(mDimension-2);
    }

    if(MAX_CROWD_SIZE < OgreDetourCrowd::MAX_AGENTS)
        mCrowdSize = MAX_CROWD_SIZE;
    else
        mCrowdSize = OgreDetourCrowd::MAX_AGENTS;

    mCrowdSize = 100;

// TODO make sure crowdSize is a multiple of nbPagedTiles?

    mCharacters.reserve(mCrowdSize);
    mUnassignedCharacters.reserve(mCrowdSize /*mCrowdSize*mNbTilesInBorder*/);
    mAssignedCharacters.reserve(mCrowdSize);
    mBorderTiles.reserve(mNbTilesInBorder);
}

void OgreRecastPagedCrowdApplication::initAgents()
{
    // Make sure camera is already set!
    mCurrentlyPagedArea = calculatePopulatedArea();
    updateBorderTiles();    // Make sure agents that walk out of the area are placed in the right border tiles

    updatePagedAreaDebug(mCurrentlyPagedArea);

    // Initialize and place agents: distribute uniformly
    int agentsPerTile = mCrowdSize/mNbPagedTiles;
    int nbAgents = -1;
    for (int x = mCurrentlyPagedArea.xMin; x <= mCurrentlyPagedArea.xMax; x++) {
        for (int y = mCurrentlyPagedArea.yMin; y <= mCurrentlyPagedArea.yMax; y++) {
            if(tileExists(x,y)) {
                debugPrint("Init: load "+Ogre::StringConverter::toString(agentsPerTile)+" agents on tile "+tileToStr(x,y)+".");
                for(int i = 0; i < agentsPerTile; i++) {
                    nbAgents++;
    // TODO another option is to place agents randomly in a circle around the whole paged area, same for a complete move to another position!
                    //Ogre::Vector3 position = getRandomPositionInNavmeshTileSet(NavmeshTileSet);
                    Ogre::Vector3 position = getRandomPositionInNavmeshTile(x, y);
                    Character *character;
                    if(OgreRecastApplication::HUMAN_CHARACTERS)
                        character = new AnimateableCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, false, position);
                    else
                        character = new TestCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, position);
                    mCharacters.push_back(character);
                    mAssignedCharacters.push_back(character);
                    assignAgentDestination(character);
                }
            } else {
                debugPrint("Init: Tile "+tileToStr(x,y)+" does not exist, 0 agents loaded.");
            }
        }
    }

    // Create any remaining unassigned characters
    nbAgents++;
    while(nbAgents < mCrowdSize) {
        Character *character;
        if(OgreRecastApplication::HUMAN_CHARACTERS)
            character = new AnimateableCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd);
        else
            character = new TestCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd);
        character->unLoad();
        mCharacters.push_back(character);
        mUnassignedCharacters.push_back(character);
        nbAgents++;
    }
}

Ogre::Vector3 OgreRecastPagedCrowdApplication::assignAgentDestination(Character* character)
{
    // Assign a random destination (wandering)
    Ogre::Vector3 rndPt = mRecast->getRandomNavMeshPoint();
    character->updateDestination(rndPt);

    return rndPt;
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
            boxNode->setScale(10, 2.5, 10);
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


    // DETOUR CROWD (local steering for independent agents)
    mDetourCrowd = new OgreDetourCrowd(mRecast);        // TODO add option of specifying max crowd size?

    if(EXTRACT_WALKABLE_AREAS) {
        // Mark walkable area (where agents will be spawned)
        OgreRecastNavmeshPruner *navMeshPruner = mRecast->getNavmeshPruner();
        // Start tracing at the origin position on the navmesh, and include all areas that are reachable from there (there is no box in the center)
        navMeshPruner->floodNavmesh(Ogre::Vector3::ZERO);
        navMeshPruner->pruneSelected();
    }


    // DETOUR CROWD PAGING
    setDebugVisibility(true);
    // Determine size of populated area (centered around camera and navmesh tile-aligned)
    initAgents();


    // ADJUST CAMERA MOVING SPEED (default is 150)
    mCameraMan->setTopSpeed(80);
}

void OgreRecastPagedCrowdApplication::setDebugVisibility(bool visible)
{
    mDebugDraw = visible;

//    mNavMeshNode->setVisible(visible);

    // Change visibility of all registered debug entities for the application
    for(std::vector<Ogre::Entity*>::iterator iter = mDebugEntities.begin(); iter != mDebugEntities.end(); iter++) {
        Ogre::Entity *ent = *iter;
        ent->setVisible(visible);
    }

    for(std::vector<Character*>::iterator iter = mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;
        character->setDebugVisibility(visible);
    }
}

bool OgreRecastPagedCrowdApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    // Perform navmesh updates
    mDetourTileCache->handleUpdate(evt.timeSinceLastFrame);

    // Update paged crowd
    updatePagedCrowd(evt.timeSinceLastFrame);

    // Update crowd agents
    mDetourCrowd->updateTick(evt.timeSinceLastFrame);

    // Then update all characters controlled by the agents
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        // Update character (position, animations, state)
        character->update(evt.timeSinceLastFrame);

        // Set new destinations when agents reach their current destination (random wander)
        if ( character->destinationReached() ) {
            character->updateDestination( mRecast->getRandomNavMeshPoint() );
        }
    }


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

    return BaseApplication::frameRenderingQueued(evt);

}

// TODO make Camera a parameter?
OgreRecastPagedCrowdApplication::NavmeshTileSet OgreRecastPagedCrowdApplication::calculatePopulatedArea()
{
    NavmeshTileSet result;

    Ogre::Vector3 cameraPos = mCamera->getPosition();
    Ogre::Vector2 camTilePos = mDetourTileCache->getTileAtPos(cameraPos);

    result.xMin = camTilePos.x - (mPagedAreaDistance);
    result.xMax = camTilePos.x + (mPagedAreaDistance);

    result.yMin = camTilePos.y - (mPagedAreaDistance);
    result.yMax = camTilePos.y + (mPagedAreaDistance);

    return result;
}

bool OgreRecastPagedCrowdApplication::updatePagedCrowd(Ogre::Real timeSinceLastFrame)
{
    // Update paged area when camera has moved, also re-add agents that walked out of the grid

    mTimeSinceLastUpdate += timeSinceLastFrame;
    bool update = false;
    NavmeshTileSet newPagedArea = calculatePopulatedArea();


    // DETECT wheter the grid has changed (ie. whether tiles should be unloaded and others loaded)
    if(newPagedArea.xMin != mCurrentlyPagedArea.xMin || newPagedArea.yMin != mCurrentlyPagedArea.yMin) {
        // Paged area has changed

        if((int)Ogre::Math::Abs(newPagedArea.xMin - mCurrentlyPagedArea.xMin) < 2
                &&
           (int)Ogre::Math::Abs(newPagedArea.yMin - mCurrentlyPagedArea.yMin) < 2) {
            // Paged area has slid by one tile, only perform one update per time delta
            if(mTimeSinceLastUpdate > CROWD_PAGE_UPDATE_DELTA)
                update = true;
            else
                return false;   // Don't update now, also don't re-add agents that walked out of the grid (this could be useless as the grid will soon change)
        } else {
            // Paged area has moved somewhere else, update immediately
            update = true;
        }
    }


    if(!update) {
        // Unload agents that walked of the grid, add them somewhere in some random border tile
        for(std::vector<Character*>::iterator iter = mAssignedCharacters.begin(); iter != mAssignedCharacters.end(); iter++) {
            Character *character = *iter;
            if(walkedOffGrid(character)) {
                // Place agent in new random border tile
                placeAgentOnRandomBorderTile(character);
                debugPrint("Placed agent on random tile "+tileToStr(mDetourTileCache->getTileAtPos(character->getPosition()))+"  "+Ogre::StringConverter::toString(character->getPosition()));
            }
        }
    }


    // Remove all agents outside of the new paged area
    unloadAgentsOutsideArea(newPagedArea);


    // Grid has changed: unload tiles, load others
    if(update) {
        updatePagedAreaDebug(newPagedArea);

        // Add agents to newly loaded tiles
        for (int x = newPagedArea.xMin; x <= newPagedArea.xMax; x++) {
            if(x < mCurrentlyPagedArea.xMin) {
                for(int y = newPagedArea.yMin; y <= newPagedArea.yMax; y++)
                    loadAgents(x,y, mCrowdSize/mNbPagedTiles);
            } else if(x > mCurrentlyPagedArea.xMax) {
                for(int y = newPagedArea.yMin; y <= newPagedArea.yMax; y++)
                    loadAgents(x,y, mCrowdSize/mNbPagedTiles);
            } else /*if(x >= mCurrentlyPagedArea.xMin && x <= mCurrentlyPagedArea.xMax)*/ {
                for (int y = newPagedArea.yMin; y <= newPagedArea.yMax; y++) {
                    if(y < mCurrentlyPagedArea.yMin) {
                        loadAgents(x,y, mCrowdSize/mNbPagedTiles);
                    } else if(y > mCurrentlyPagedArea.yMax) {
                        loadAgents(x,y, mCrowdSize/mNbPagedTiles);
                    }// else x,y were also in current paged area
                }
            }
        }

        mTimeSinceLastUpdate = 0;
        mCurrentlyPagedArea = newPagedArea;


        // Update list with existing border tiles
        updateBorderTiles();
    }


    updateDebugInfo();

    return update;
}

void OgreRecastPagedCrowdApplication::updateBorderTiles()
{
    // TODO is it a good idea to use existing area, or really only use outer borders of area? the disadvantage of this approach is that agents could appear closer to the camera, the advantage that new agents can always appear, even if there is only one tile available in the current grid
    NavmeshTileSet existingArea = getExistingArea(mCurrentlyPagedArea);
    mBorderTiles.clear();
    for(int y = existingArea.yMin; y <= existingArea.yMax; y++) {
        if(tileExists(existingArea.xMin, y))
            mBorderTiles.push_back(Ogre::Vector2(existingArea.xMin, y));
        if(tileExists(existingArea.xMax, y))
            mBorderTiles.push_back(Ogre::Vector2(existingArea.xMax, y));
    }
}

OgreRecastPagedCrowdApplication::NavmeshTileSet OgreRecastPagedCrowdApplication::getExistingArea(NavmeshTileSet area)
{
    TileSelection bounds = mDetourTileCache->getBounds();

    if(area.xMin < bounds.minTx)
        area.xMin = bounds.minTx;
    if(area.yMin < bounds.minTy)
        area.yMin = bounds.minTy;
    if(area.xMax > bounds.maxTx)
        area.xMax = bounds.maxTx;
    if(area.yMax > bounds.maxTy)
        area.yMax = bounds.maxTy;

    return area;    // Note: there can still be non-existing tiles in-between tiles. Use tileExists() to test.
}

void OgreRecastPagedCrowdApplication::updateDebugInfo()
{
    if(mDebugPanel->isVisible()) {
        mDebugPanel->setParamValue(0, Ogre::StringConverter::toString(mCrowdSize));                 // Total agents
        mDebugPanel->setParamValue(1, Ogre::StringConverter::toString(mAssignedCharacters.size())); // Loaded agents
        Ogre::String dimensionStr = Ogre::StringConverter::toString(mDimension);
        mDebugPanel->setParamValue(2, dimensionStr+"x"+dimensionStr);                               // Grid size
        mDebugPanel->setParamValue(3, Ogre::StringConverter::toString(getNbLoadedTiles()));         // Loaded tiles
        mDebugPanel->setParamValue(4, Ogre::StringConverter::toString(getNbBorderTiles()));         // Border tiles
        mDebugPanel->setParamValue(5, mTopDownCamera?"Top-down":"Free");                            // Camera type
    }
}

int OgreRecastPagedCrowdApplication::getNbLoadedTiles()
{
    int count = 0;
    for (int x = mCurrentlyPagedArea.xMin; x <= mCurrentlyPagedArea.xMax; x++) {
        for (int y = mCurrentlyPagedArea.yMin; y <= mCurrentlyPagedArea.yMax; y++) {
            if(tileExists(x,y))
                count++;
        }
    }

    return count;
}

int OgreRecastPagedCrowdApplication::getNbBorderTiles()
{
    return mBorderTiles.size();
}

void OgreRecastPagedCrowdApplication::placeAgentOnRandomBorderTile(Character *character)
{
    if(mBorderTiles.size() == 0 )
        return;

    int borderTile = (int)Ogre::Math::RangeRandom(0, mBorderTiles.size());
    Ogre::Vector2 tile = mBorderTiles[borderTile];

    placeAgent(character, tile.x,tile.y);
}

bool OgreRecastPagedCrowdApplication::tileExists(int tx, int ty)
{
    return mDetourTileCache->tileExists(tx, ty);
}

bool OgreRecastPagedCrowdApplication::walkedOffGrid(const Character *character)
{
// TODO detect whether agent is at border tile, at the outer edge, and is pointing towards walking off (or has no velocity anymore, but beware of recently added agents that were not yet assigned a velocity)
// TODO also think about how to assign waypoint destinations, could be a problem if they are to locations of which no navmesh is loaded. Then agents will navigate to the closest point on the navmesh and stop, this is not necessarily the right path. Maybe a higher-level checkpoint graph.

    // Detect whether an agent has moved to a tile outside of the current area
    // NOTE: for this to work, the navmesh tiles that extend at least one tile outside of the currently populated area
    // have to be loaded in the tilecache.
    Ogre::Vector2 tpos = mDetourTileCache->getTileAtPos(character->getPosition());

    if(tpos.x < mCurrentlyPagedArea.xMin || tpos.x > mCurrentlyPagedArea.xMax)
        return true;

    if(tpos.y < mCurrentlyPagedArea.yMin || tpos.y > mCurrentlyPagedArea.yMax)
        return true;


    return false;
}

void OgreRecastPagedCrowdApplication::unloadAgents(int tx, int ty)
{
    if(! tileExists(tx,ty))
        return;

    u_int i = 0;
    int agentsRemoved = 0;
    while(i < mAssignedCharacters.size()) {
        Character *character = mAssignedCharacters[i];
        Ogre::Vector2 tilePos = mDetourTileCache->getTileAtPos(character->getPosition());
        if(tilePos.x == tx && tilePos.y == ty) {    //TODO Is this safe? tile positions are ints, but they were stored in a float
            agentsRemoved++;
            character->unLoad();
            mUnassignedCharacters.push_back(character);
            mAssignedCharacters.erase(mAssignedCharacters.begin()+i);
            // Don't advance i, current position contains the next element
        } else {
            i++;
        }
    }

    debugPrint("Unloaded "+Ogre::StringConverter::toString(agentsRemoved)+" agents from tile "+tileToStr(tx,ty)+".");
}

void OgreRecastPagedCrowdApplication::unloadAgentsOutsideArea(NavmeshTileSet tileSet)
{
    u_int i = 0;
    int agentsRemoved = 0;
    while(i < mAssignedCharacters.size()) {
        Character *character = mAssignedCharacters[i];
        Ogre::Vector2 tilePos = mDetourTileCache->getTileAtPos(character->getPosition());

        if( tilePos.x < tileSet.xMin || tilePos.x > tileSet.xMax
         || tilePos.y < tileSet.yMin || tilePos.y > tileSet.yMax) {
            agentsRemoved++;
            character->unLoad();
            mUnassignedCharacters.push_back(character);
            mAssignedCharacters.erase(mAssignedCharacters.begin()+i);

            // Don't advance i, current position contains the next element
        } else {
            i++;
        }
    }

    if(agentsRemoved)
        debugPrint("Unloaded "+Ogre::StringConverter::toString(agentsRemoved)+" agents.");
}

void OgreRecastPagedCrowdApplication::debugPrint(Ogre::String message)
{
    if(mDebugDraw)
        Ogre::LogManager::getSingletonPtr()->logMessage(message);
}

Ogre::String OgreRecastPagedCrowdApplication::tileToStr(Ogre::Vector2 tilePos)
{
    return tileToStr(tilePos.x, tilePos.y);
}

Ogre::String OgreRecastPagedCrowdApplication::tileToStr(int tx, int ty)
{
    return "("+Ogre::StringConverter::toString(tx)+", "+Ogre::StringConverter::toString(ty)+")";
}

void OgreRecastPagedCrowdApplication::loadAgents(int tx, int ty, int nbAgents)
{
    if(! tileExists(tx,ty)) {
        debugPrint("Will not load agents on tile "+tileToStr(tx,ty)+": does not exist.");
        return;
    }

    // Iterate over free agent list and distribute evenly
// TODO allow other distributions

    //for(std::vector<Character*>::iterator iter = mUnassignedCharacters.begin(); iter != mUnassignedCharacters.end(); iter++) {
    int agentsPlaced = 0;
//    Ogre::String agentsString = "  ";
    while(mUnassignedCharacters.size() != 0 && agentsPlaced < nbAgents) {
        Character *character = mUnassignedCharacters[mUnassignedCharacters.size()-1];
        mUnassignedCharacters.pop_back();

        Ogre::Vector3 pos = placeAgent(character, tx, ty);
        mAssignedCharacters.push_back(character);
        Ogre::Vector2 tilePos = mDetourTileCache->getTileAtPos(pos);
//        agentsString += Ogre::StringConverter::toString(character->getPosition())+" "+tileToStr(tilePos.x, tilePos.y)+" ";

        agentsPlaced++;
    }

    debugPrint("Loaded "+Ogre::StringConverter::toString(agentsPlaced)+" agents on tile "+tileToStr(tx,ty)+"." /*+agentsString*/);
}

Ogre::Vector3 OgreRecastPagedCrowdApplication::getRandomPositionInNavmeshTile(int tx, int ty)
{
    Ogre::AxisAlignedBox tileBounds = mDetourTileCache->getTileBounds(tx, ty);
    Ogre::Vector3 center = tileBounds.getCenter();  // Center of the specified tile
    //center.y = tileBounds.getMinimum().y;   // Place on the ground
        // TODO centering probably has the biggest change of the point clipping to the navmesh

    // Get random point in tile (in circle in the middle of the tile with radius of tilesize/2)
    Ogre::Real radius = mDetourTileCache->getTileSize()/2;
    return mRecast->getRandomNavMeshPointInCircle(center, radius-RADIUS_EPSILON);   // TODO I could also make RADIUS_EPSILON be a fraction of the tileSize
}

Ogre::Vector3 OgreRecastPagedCrowdApplication::placeAgent(Character* character, int tx, int ty)
{
    Ogre::Vector3 rndPos = getRandomPositionInNavmeshTile(tx, ty);

    character->load(rndPos);

    assignAgentDestination(character);

    return rndPos;
}

Ogre::AxisAlignedBox OgreRecastPagedCrowdApplication::getNavmeshTileSetBounds(NavmeshTileSet tileSet)
{
    // TODO if I declare NavmeshTileSet struct in OgreDetourTileCache I can move this method to the tilecache
    Ogre::AxisAlignedBox tileMinBounds = mDetourTileCache->getTileBounds(tileSet.xMin, tileSet.yMin);
    Ogre::AxisAlignedBox tileMaxBounds = mDetourTileCache->getTileBounds(tileSet.xMax, tileSet.yMax);

    Ogre::AxisAlignedBox tileSetBounds;
    tileSetBounds.setMinimum(tileMinBounds.getMinimum());
    tileSetBounds.setMaximum(tileMaxBounds.getMaximum());

    return tileSetBounds;
}

Ogre::Vector3 OgreRecastPagedCrowdApplication::getRandomPositionInNavmeshTileSet(NavmeshTileSet tileSet)
{
    Ogre::AxisAlignedBox tileSetBounds = getNavmeshTileSetBounds(tileSet);
    Ogre::Vector3 center = tileSetBounds.getCenter();
    //center.y = tileSetBounds.getMinimum().y;
        // TODO centering probably has the biggest change of the point clipping to the navmesh

    Ogre::Real radius = ( tileSet.getNbTiles()*mDetourTileCache->getTileSize() )/2;
    return mRecast->getRandomNavMeshPointInCircle(center, radius - RADIUS_EPSILON);
}

void OgreRecastPagedCrowdApplication::updatePagedAreaDebug(NavmeshTileSet pagedArea)
{
    /*
    if(mAreaDebug) {
        mAreaDebug->detachFromParent();
        mSceneMgr->destroyManualObject(mAreaDebug);
    }
    */

    Ogre::AxisAlignedBox areaBounds = getNavmeshTileSetBounds(pagedArea);

    if(! mAreaDebug) {
        Ogre::SceneNode *areaSn = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        mAreaDebug = mSceneMgr->createEntity("AreaDemarkationDebug", "Demarkation.mesh");
        areaSn->attachObject(mAreaDebug);
        Ogre::Vector3 scale = areaBounds.getSize();
        if(scale.y < 5)
            scale.y = 10;
        areaSn->setScale(scale);
    }

    Ogre::Vector3 position = areaBounds.getCenter();
    position.y = areaBounds.getMinimum().y;
    mAreaDebug->getParentSceneNode()->setPosition(position);
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
