#ifndef OGRERECASTTERRAINAPPLICATION_H
#define OGRERECASTTERRAINAPPLICATION_H

#include "OgreRecastApplication.h"
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include <Terrain/OgreTerrainPrerequisites.h>


/**
  * The second of the Ogre Recast demos. Demonstrates navmesh building on terrain.
  * This demo builds heavily on the original Dungeon demo, and reuses as much features as possible.
  * It is therefore recommended to start with the other demo before digging into this one.
  **/
class OgreRecastTerrainApplication : public OgreRecastApplication
{
public:
    /**
      * Create an instance of the Terrain Demo.
      **/
    OgreRecastTerrainApplication(void);
    virtual ~OgreRecastTerrainApplication(void);

    /**
      * @see{OgreRecastApplication::setDebugVisibility(bool)}
      **/
    virtual void setDebugVisibility(bool visible);


    /**
      * The number of terrain tiles to generate in the demo
      * in X dimension. Should be >= 1.
      **/
    static size_t TERRAIN_TILES_X;
    /**
      * The number of terrain tiles to generate in the demo
      * in Z dimension.
      * Should be >= 1.
      **/
    static size_t TERRAIN_TILES_Z;
    /**
      * World size of one terrain tile.
      **/
    static float TERRAIN_TILE_SIZE;
    /**
      * The size of each terrain down one edge in vertices (2^n+1).
      **/
    static Ogre::uint16 TERRAIN_TILE_RESOLUTION;
    /**
      * Scale of terrain height (input scale of heightmap), relative
      * to TERRAIN_TILE_SIZE.
      * Set to 1 for regular scale, higher for higher mountains, lower
      * for flatter terrain.
      * Must be a value > 0.
      * I recommend values between 0.1 and 10.
      **/
    static float TERRAIN_HEIGHT_SCALE;

protected:

    /**
      * @see{OgreRecastApplication::createScene()}
      **/
    virtual void createScene(void);

    /**
      * @see{OgreRecastApplication::createCharacter(Ogre::String, Ogre::Vector3)}
      *
      * With the small difference that it enables terrain clipping for characters.
      **/
    virtual Character* createCharacter(Ogre::String name, Ogre::Vector3 position);

    /**
      * @see{OgreRecastApplication::destroyScene()}
      **/
    virtual void destroyScene(void);


    /**
      * Handle keyboard input when keys are pressed.
      **/
    virtual bool keyPressed( const OIS::KeyEvent &arg );

    /**
      * @see{OgreRecastApplication::frameRenderingQueued(const Ogre::FrameEvent&)}
      **/
    virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);

    /**
      * @see{OgreRecastApplication::queryCursorPosition(Ogre::Vector3&, unsigned long, bool, Ogre::MovableObject**)}
      *
      * With the difference that if static geometry is used for debug drawing the navmesh (OgreRecast::STATIC_GEOM_DEBUG)
      * then the terrain will be queried. Also if RAYCAST_SCENE is true the terrain will be queried.
      **/
    virtual bool queryCursorPosition(Ogre::Vector3 &rayHitPoint, unsigned long queryflags = NAVMESH_MASK, bool clipToNavmesh = true, Ogre::MovableObject **rayHitObject = NULL);


private:
    /**
      * Build terrain. This is the exact same code as intermediate tutorial about using terrain.
      **/
    void defineTerrain(long x, long y);

    /**
      * Terrain building, copied from terrain tutorial.
      **/
    void initBlendMaps(Ogre::Terrain* terrain);

    /**
      * Terrain config, copied from terrain tutorial.
      **/
    void configureTerrainDefaults(Ogre::Light* light);

    /**
      * Terrain config parameters
      **/
    Ogre::TerrainGlobalOptions* mTerrainGlobals;

    /**
      * Terrain group. Stores all terrain pages in the scene.
      **/
    Ogre::TerrainGroup* mTerrainGroup;

    /**
      * True when all terrain is loaded.
      **/
    bool mTerrainsImported;

    /**
      * Recast compatible input geometry. The terrain an all extra entities on it are
      * converted to this format to be able to build a navmesh from it.
      **/
    InputGeom *mGeom;

};

#endif // OGRERECASTTERRAINAPPLICATION_H
