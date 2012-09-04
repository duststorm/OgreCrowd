#ifndef OGRERECASTPAGEDCROWDAPPLICATION_H
#define OGRERECASTPAGEDCROWDAPPLICATION_H

#include "BaseApplication.h"
#include "Character.h"

/**
  * Demo showcasing paging of crowds, allowing to render crowds of unlimited
  * size over virtually infinite worlds. A good example case for this is a
  * large city, where only the area around the camera is populated by crowd
  * agents at one time.
  **/
class OgreRecastPagedCrowdApplication : public BaseApplication
{
public:
    OgreRecastPagedCrowdApplication();

    /**
      * Sets recast visual debugging geometry in the scene to visible (true) or hide
      * it (false).
      **/
    virtual void setDebugVisibility(bool visible);


// TODO move this struct to OgreDetourTileCache??
// TODO functionality corresponds largely to OgreDetourTileCache::TileSelection, merge them without breaking anything
    struct NavmeshTileSet
    {
        int xMin;   // Min tile X index (inclusive)
        int yMin;   // Min tile Y index (inclusive)
        int xMax;   // Max tile X index (inclusive)
        int yMax;   // Min tile Y index (inclusive)

        /*
        int getTilesInSet(int* result)
        {
            u_int size = getXWidth()*getYWidth();
            u_int i = 0;
            for (int x = tileXMin; x < tileXMax+1; x++) {
                for (int y = tileYMin; x < tileYMax+1; x++) {
                    result[i] = ;
                    i++;
                }
            }

            for (u_int i=0; i < size; i++) {
                result[i] =
            }
        }
        */

        int getXWidth(void) { return 1+ xMax - xMin ; }
        int getYWidth(void) { return 1+ yMax - yMin; }
        int getNbTiles(void) { return getXWidth()*getYWidth(); }
    };

    static const Ogre::Real CROWD_PAGE_UPDATE_DELTA;
    static const Ogre::Real MAX_CROWD_SIZE;
    static const Ogre::Real RADIUS_EPSILON;
    static const Ogre::Real TOPDOWN_CAMERA_HEIGHT;

protected:
    /**
      * Initialise the scene and everything needed for pathfinding and steering.
      * Setup demo specific constructions.
      **/
    virtual void createScene(void);

    /**
      * Update state for rendering a new frame.
      **/
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual void createFrameListener(void);

    virtual bool walkedOffGrid(const Character* character);

    virtual bool keyPressed(const OIS::KeyEvent &arg);

    virtual bool keyReleased(const OIS::KeyEvent &arg);

    /**
      * Determines whether the tile with specified grid coordinates exists
      * and is loaded (in the tilecache).
     **/
    virtual bool tileExists(int tx, int ty);

    virtual void initAgents(void);


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

    void updatePagedAreaDebug(NavmeshTileSet pagedArea);

    void debugPrint(Ogre::String message);

    Ogre::String tileToStr(int tx, int ty);

    Ogre::String tileToStr(Ogre::Vector2 tilePos);

    Ogre::Vector3 assignAgentDestination(Character* character);

    void updateDebugInfo(void);

    int getNbLoadedTiles(void);

    int getNbBorderTiles(void);

    void unloadAgentsOutsideArea(NavmeshTileSet area);

    NavmeshTileSet getExistingArea(NavmeshTileSet area);

    void updateBorderTiles(void);


    /**
      * Ogre version of the code of the original Recast demo that comes with the recast distribution.
      **/
    OgreRecast* mRecast;

    /**
      * Ogre version of DetourTileCache.
      * Only used when SINGLE_NAVMESH is false.
      **/
    OgreDetourTileCache *mDetourTileCache;

    /**
      * Ogre wrapper around the detourCrowd library
      **/
    OgreDetourCrowd *mDetourCrowd;

    /**
      * All characters in the scene that represent an agent.
      **/
    std::vector<Character*> mCharacters;
        // TODO do I need this list?

    std::vector<Character*> mAssignedCharacters;    // TODO a linked list might be better
    std::vector<Character*> mUnassignedCharacters;

    /**
      * Current visibility of recast visual debug structures.
      * True renders them in the scene, false hides them.
      **/
    bool mDebugDraw;

    /**
      * The scenenode containing the visual representation of the navmesh
      * and other static debug drawing geometry.
      **/
    Ogre::SceneNode *mNavMeshNode;

    /**
      * List of all entities used for debugging.
      * There are hidden when debug drawing is disabled.
      **/
    std::vector<Ogre::Entity*> mDebugEntities;

    /**
      * List of all entities in the scene that are used to construct a navmesh from.
      * (the dungeon and the pots)
      **/
    std::vector<Ogre::Entity*> mNavmeshEnts;

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

    OgreBites::ParamsPanel* mDebugPanel;     // sample details panel

    bool mTopDownCamera;
    bool mGoingUp;
    bool mGoingDown;
    bool mGoingLeft;
    bool mGoingRight;

    std::vector<Ogre::Vector2> mBorderTiles;

};

#endif // OGRERECASTPAGEDCROWDAPPLICATION_H
