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

#ifndef CROWDMANAGER_H
#define CROWDMANAGER_H

#include <OIS.h>
#include <Ogre.h>

class Character;
class OgreRecast;
class OgreDetourCrowd;
class OgreDetourTileCache;

// TODO maybe add support for "persistent" characters, eg. those that are always visible, or at least of which the agent and AI remains when out of view distance

/**
  * Manages and instances a crowd of characters in the vicinity of the camera.
  *
  **/
class CrowdManager
{
public:
    CrowdManager(OgreDetourTileCache *tileCache, Ogre::SceneManager *sceneManager, Ogre::Camera *camera);

    void setCamera(Ogre::Camera *camera) { mCamera = camera; }

    Ogre::Camera* getCamera(void) { return mCamera; }

    void update(Ogre::Real timeSinceLastFrame);

    int getNbLoadedTiles(void);

    int getNbBorderTiles(void);

    void setDebugVisibility(bool visible);

    /**
      * The size of the crowd
      **/
    int getSize(void) { return mCrowdSize; }

    int getNbAssignedAgents(void) { mAssignedCharacters.size(); }

    int getGridDimensions(void) { return mDimension; }



    // TODO define setters for these?
    static const Ogre::Real CROWD_PAGE_UPDATE_DELTA;
    static const Ogre::Real MAX_CROWD_SIZE;
    static const Ogre::Real RADIUS_EPSILON;

    static bool HUMAN_CHARACTERS;
    static bool INSTANCED_CROWD;

    // TODO move this struct to OgreDetourTileCache??
    // TODO functionality corresponds largely to OgreDetourTileCache::TileSelection, merge them without breaking anything
    struct NavmeshTileSet
    {
        int xMin;   // Min tile X index (inclusive)
        int yMin;   // Min tile Y index (inclusive)
        int xMax;   // Max tile X index (inclusive)
        int yMax;   // Min tile Y index (inclusive)

        int getXWidth(void) { return 1+ xMax - xMin ; }
        int getYWidth(void) { return 1+ yMax - yMin; }
        int getNbTiles(void) { return getXWidth()*getYWidth(); }
    };

protected:
    /**
      * Calculate the navmesh tile-aligned bounding area around the
      * current camera position that has to be populated with crowd agents.
      **/
    NavmeshTileSet calculatePopulatedArea(void);

    bool updatePagedCrowd(Ogre::Real timeSinceLastFrame);

    void loadAgents(int tx, int ty, int nbAgents);

    void unloadAgents(int tx, int ty);

    // Make sure tileExists(tx,ty) !!
    Ogre::Vector3 placeAgent(Character* character, int tx, int ty);

    void placeAgentOnRandomBorderTile(Character *character);

    Ogre::Vector3 getRandomPositionInNavmeshTile(int tx, int ty);

    Ogre::Vector3 getRandomPositionInNavmeshTileSet(NavmeshTileSet tileSet);

    Ogre::AxisAlignedBox getNavmeshTileSetBounds(NavmeshTileSet tileSet);

    /**
      * Determines whether the tile with specified grid coordinates exists
      * and is loaded (in the tilecache).
     **/
    virtual bool tileExists(int tx, int ty);

    void updatePagedAreaDebug(NavmeshTileSet pagedArea);

    void debugPrint(Ogre::String message);

    Ogre::String tileToStr(int tx, int ty);

    Ogre::String tileToStr(Ogre::Vector2 tilePos);

    Ogre::Vector3 assignAgentDestination(Character* character);

    void unloadAgentsOutsideArea(NavmeshTileSet area);

    NavmeshTileSet getExistingArea(NavmeshTileSet area);

    void updateBorderTiles(void);

    virtual bool walkedOffGrid(const Character* character);

    virtual void initAgents(void);


    OgreDetourTileCache *mDetourTileCache;
    OgreRecast *mRecast;
    OgreDetourCrowd *mDetourCrowd;
    Ogre::SceneManager *mSceneMgr;

    /**
      * Camera around which the crowd is intantiated
      **/
    Ogre::Camera *mCamera;

    /**
      * All characters in the scene that represent an agent.
      **/
    std::vector<Character*> mCharacters;
        // TODO do I need this list?

    std::vector<Character*> mAssignedCharacters;    // TODO a linked list might be better
    std::vector<Character*> mUnassignedCharacters;


    NavmeshTileSet mCurrentlyPagedArea;

    /**
      * Size (in number of navmesh tiles) in x and y direction
      * that the paged area centered around camera position will
      * continue and will be populated with agents.
      * Defines the size of the area to be populated with agents.
      **/
    int mPagedAreaDistance;
        // TODO allow other methods of selecting area to page? Like a circle around camera position

    int mNbPagedTiles;

    int mNbTilesInBorder;

    int mCrowdSize;

    int mDimension;

    Ogre::Real mTimeSinceLastUpdate;

    Ogre::Entity *mAreaDebug;

    /**
      * Current visibility of recast visual debug structures.
      * True renders them in the scene, false hides them.
      **/
    bool mDebugDraw;


    std::vector<Ogre::Vector2> mBorderTiles;

#if OGRE_VERSION_MINOR >= 8
    Ogre::InstanceManager* mInstanceManager;
#endif

};

#endif // CROWDMANAGER_H
