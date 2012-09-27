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

#include "include/CrowdManager.h"
#include "OgreDetourCrowd.h"
#include "Character.h"
#include "AnimateableCharacter.h"
#include "TestCharacter.h"
#if OGRE_VERSION_MINOR >= 8
#include "InstancedCharacter.h"
#endif


const Ogre::Real CrowdManager::CROWD_PAGE_UPDATE_DELTA = 1;

bool CrowdManager::HUMAN_CHARACTERS = true;
bool CrowdManager::INSTANCED_CROWD = false;

// TODO this can also be DetourCrowd::MAX_AGENTS or allow setting of Max agents in detourCrowd
const Ogre::Real CrowdManager::MAX_CROWD_SIZE = 100; // TODO make this a cfg parameter
const Ogre::Real CrowdManager::RADIUS_EPSILON = 1;


CrowdManager::CrowdManager(OgreDetourTileCache *tileCache, Ogre::SceneManager *sceneManager, Ogre::Camera *camera)
    : mDetourTileCache(tileCache)
    , mRecast(mDetourTileCache->getRecast())
    , mSceneMgr(sceneManager)
    , mCamera(camera)
    , mDetourCrowd(0)
    , mCharacters()
    , mCurrentlyPagedArea()
    , mPagedAreaDistance(2)
    , mNbPagedTiles(0)
    , mNbTilesInBorder(0)
    , mAreaDebug(0)
    , mBorderTiles()
#if OGRE_VERSION_MINOR >= 8
    , mInstanceManager(0)
#endif
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

// TODO make sure crowdSize is a multiple of nbPagedTiles?

    mCharacters.reserve(mCrowdSize);
    mUnassignedCharacters.reserve(mCrowdSize);
    mAssignedCharacters.reserve(mCrowdSize);
    mBorderTiles.reserve(mNbTilesInBorder);


    // DetourCrowd component managed by this CrowdManager
    mDetourCrowd = new OgreDetourCrowd(mRecast);        // TODO add option of specifying max crowd size?


#if OGRE_VERSION_MINOR >= 8
    if(INSTANCED_CROWD) {
        // Most compatible SM2+ technique
        Ogre::InstanceManager::InstancingTechnique instanceTechnique = Ogre::InstanceManager::ShaderBased;

        // Create instance manager for managing instances of the robot mesh
        mInstanceManager = mSceneMgr->createInstanceManager(
                    "CrowdCharacter_1_InstanceMgr", "Gamechar-male.mesh",
                    Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME, instanceTechnique,
                    mCrowdSize); // TODO experiment with batch size
    }
#endif

    initAgents();
}

void CrowdManager::initAgents()
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
                    //Ogre::Vector3 position = getRandomPositionInNavmeshTileSet(NavmeshTileSet);
                    Ogre::Vector3 position = getRandomPositionInNavmeshTile(x, y);
                    Character *character;
// TODO make configurable which type of character is exactly instanced (maybe allow keeping sets of different populations)
#if OGRE_VERSION_MINOR >= 8
                    if(INSTANCED_CROWD) {
                        character = new InstancedCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, mInstanceManager, false, position);
                    } else if(HUMAN_CHARACTERS) {
#else
                    if(HUMAN_CHARACTERS) {
#endif
                        character = new AnimateableCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, false, position);
                    } else {
                        character = new TestCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, position);
                    }
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
#if OGRE_VERSION_MINOR >= 8
        if(INSTANCED_CROWD) {
            character = new InstancedCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd, mInstanceManager);
        } else if(HUMAN_CHARACTERS) {
#else
        if(HUMAN_CHARACTERS) {
#endif
            character = new AnimateableCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd);
        } else {
            character = new TestCharacter("Character_"+Ogre::StringConverter::toString(nbAgents), mSceneMgr, mDetourCrowd);
        }
        character->unLoad();
        mCharacters.push_back(character);
        mUnassignedCharacters.push_back(character);
        nbAgents++;
    }
}

Ogre::Vector3 CrowdManager::assignAgentDestination(Character* character)
{
    // Assign a random destination (wandering)
    Ogre::Vector3 rndPt = mRecast->getRandomNavMeshPoint();
    character->updateDestination(rndPt);

    return rndPt;
}

void CrowdManager::setDebugVisibility(bool visible)
{
    mDebugDraw = visible;
    if(mAreaDebug)
        mAreaDebug->setVisible(mDebugDraw);

    for(std::vector<Character*>::iterator iter = mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;
        character->setDebugVisibility(visible);
    }
}

void CrowdManager::update(Ogre::Real timeSinceLastFrame)
{
    // Update crowd agents
    mDetourCrowd->updateTick(timeSinceLastFrame);

    // Then update all characters controlled by the agents
    for(std::vector<Character*>::iterator iter=mCharacters.begin(); iter != mCharacters.end(); iter++) {
        Character *character = *iter;

        // Update character (position, animations, state)
        character->update(timeSinceLastFrame);

        // Set new destinations when agents reach their current destination (random wander)
        if ( character->destinationReached() ) {
            character->updateDestination( mRecast->getRandomNavMeshPoint() );
        }
    }

    // Update paged crowd
    updatePagedCrowd(timeSinceLastFrame);
}

CrowdManager::NavmeshTileSet CrowdManager::calculatePopulatedArea()
{
    NavmeshTileSet result;

    Ogre::Vector3 cameraPos = mCamera->getRealPosition();
    Ogre::Vector2 camTilePos = mDetourTileCache->getTileAtPos(cameraPos);

    result.xMin = camTilePos.x - (mPagedAreaDistance);
    result.xMax = camTilePos.x + (mPagedAreaDistance);

    result.yMin = camTilePos.y - (mPagedAreaDistance);
    result.yMax = camTilePos.y + (mPagedAreaDistance);

    return result;
}

bool CrowdManager::updatePagedCrowd(Ogre::Real timeSinceLastFrame)
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
                Ogre::Vector3 oldPos = character->getPosition();
                // Place agent in new random border tile
                placeAgentOnRandomBorderTile(character);
                debugPrint("Agent "+Ogre::StringConverter::toString(character->getAgentID())+" walked off grid ("+Ogre::StringConverter::toString(oldPos)+"   "+tileToStr(mDetourTileCache->getTileAtPos(oldPos))+") placed on new random tile "+tileToStr(mDetourTileCache->getTileAtPos(character->getPosition()))+"  "+Ogre::StringConverter::toString(character->getPosition()));
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


    return update;
}

void CrowdManager::updateBorderTiles()
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

CrowdManager::NavmeshTileSet CrowdManager::getExistingArea(NavmeshTileSet area)
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

int CrowdManager::getNbLoadedTiles()
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

int CrowdManager::getNbBorderTiles()
{
    return mBorderTiles.size();
}

void CrowdManager::placeAgentOnRandomBorderTile(Character *character)
{
    if(mBorderTiles.size() == 0 )
        return;

    int borderTile = (int)Ogre::Math::RangeRandom(0, mBorderTiles.size());
    Ogre::Vector2 tile = mBorderTiles[borderTile];

    placeAgent(character, tile.x,tile.y);
}

bool CrowdManager::tileExists(int tx, int ty)
{
    return mDetourTileCache->tileExists(tx, ty);
}

bool CrowdManager::walkedOffGrid(const Character *character)
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

void CrowdManager::unloadAgents(int tx, int ty)
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

void CrowdManager::unloadAgentsOutsideArea(NavmeshTileSet tileSet)
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

void CrowdManager::debugPrint(Ogre::String message)
{
    if(mDebugDraw)
        Ogre::LogManager::getSingletonPtr()->logMessage(message);
}

Ogre::String CrowdManager::tileToStr(Ogre::Vector2 tilePos)
{
    return tileToStr((int)tilePos.x, (int)tilePos.y);
}

Ogre::String CrowdManager::tileToStr(int tx, int ty)
{
    return "("+Ogre::StringConverter::toString(tx)+", "+Ogre::StringConverter::toString(ty)+")";
}

void CrowdManager::loadAgents(int tx, int ty, int nbAgents)
{
    if(! tileExists(tx,ty)) {
        debugPrint("Will not load agents on tile "+tileToStr(tx,ty)+": does not exist.");
        return;
    }

    // Iterate over free agent list and distribute evenly
// TODO allow other distributions
    int agentsPlaced = 0;
    while(mUnassignedCharacters.size() != 0 && agentsPlaced < nbAgents) {
        Character *character = mUnassignedCharacters[mUnassignedCharacters.size()-1];
        mUnassignedCharacters.pop_back();

        Ogre::Vector3 pos = placeAgent(character, tx, ty);
        mAssignedCharacters.push_back(character);

        agentsPlaced++;
    }

    debugPrint("Loaded "+Ogre::StringConverter::toString(agentsPlaced)+" agents on tile "+tileToStr(tx,ty)+"." /*+agentsString*/);
}

Ogre::Vector3 CrowdManager::getRandomPositionInNavmeshTile(int tx, int ty)
{
    Ogre::AxisAlignedBox tileBounds = mDetourTileCache->getTileBounds(tx, ty);
    Ogre::Vector3 center = tileBounds.getCenter();  // Center of the specified tile
    //center.y = tileBounds.getMinimum().y;   // Place on the ground
        // TODO centering probably has the biggest change of the point clipping to the navmesh

    // Get random point in tile (in circle in the middle of the tile with radius of tilesize/2)
    Ogre::Real radius = mDetourTileCache->getTileSize()/2;
    return mRecast->getRandomNavMeshPointInCircle(center, radius-RADIUS_EPSILON);   // TODO I could also make RADIUS_EPSILON be a fraction of the tileSize
}

Ogre::Vector3 CrowdManager::placeAgent(Character* character, int tx, int ty)
{
    Ogre::Vector3 rndPos = getRandomPositionInNavmeshTile(tx, ty);

    character->load(rndPos);

    // Start walking animation at random position to avoid obvious synchronized movement
#if OGRE_VERSION_MINOR >= 8
    if(INSTANCED_CROWD)
        ((InstancedCharacter*) character)->randomizeAnimationPosition();
    else if(HUMAN_CHARACTERS)
#else
    if(HUMAN_CHARACTERS) {
#endif
        ((AnimateableCharacter*) character)->randomizeAnimationPosition();
// TODO this code replication is stupid. Fix up character classes with a better inheritance scheme, abstracting out demo specific and reusable classes

    assignAgentDestination(character);

    return rndPos;
}

Ogre::AxisAlignedBox CrowdManager::getNavmeshTileSetBounds(NavmeshTileSet tileSet)
{
    // TODO if I declare NavmeshTileSet struct in OgreDetourTileCache I can move this method to the tilecache
    Ogre::AxisAlignedBox tileMinBounds = mDetourTileCache->getTileBounds(tileSet.xMin, tileSet.yMin);
    Ogre::AxisAlignedBox tileMaxBounds = mDetourTileCache->getTileBounds(tileSet.xMax, tileSet.yMax);

    Ogre::AxisAlignedBox tileSetBounds;
    tileSetBounds.setMinimum(tileMinBounds.getMinimum());
    tileSetBounds.setMaximum(tileMaxBounds.getMaximum());

    return tileSetBounds;
}

Ogre::Vector3 CrowdManager::getRandomPositionInNavmeshTileSet(NavmeshTileSet tileSet)
{
    Ogre::AxisAlignedBox tileSetBounds = getNavmeshTileSetBounds(tileSet);
    Ogre::Vector3 center = tileSetBounds.getCenter();
    //center.y = tileSetBounds.getMinimum().y;
        // TODO centering probably has the biggest change of the point clipping to the navmesh

    Ogre::Real radius = ( tileSet.getNbTiles()*mDetourTileCache->getTileSize() )/2;
    return mRecast->getRandomNavMeshPointInCircle(center, radius - RADIUS_EPSILON);
}

void CrowdManager::updatePagedAreaDebug(NavmeshTileSet pagedArea)
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
        mAreaDebug->setVisible(mDebugDraw);
    }

    if(!mDebugDraw)
        return;

    Ogre::Vector3 position = areaBounds.getCenter();
    position.y = areaBounds.getMinimum().y;
    mAreaDebug->getParentSceneNode()->setPosition(position);
}
