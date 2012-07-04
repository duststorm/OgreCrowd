#ifndef OGRERECASTTERRAINAPPLICATION_H
#define OGRERECASTTERRAINAPPLICATION_H

#include "OgreRecastApplication.h"
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include <Terrain/OgreTerrainPrerequisites.h>


class OgreRecastTerrainApplication : public OgreRecastApplication
{
public:
    OgreRecastTerrainApplication(void);
    virtual ~OgreRecastTerrainApplication(void);

    virtual void setDebugVisibility(bool visible);

protected:
    virtual void createScene(void);

    virtual Character* createCharacter(Ogre::String name, Ogre::Vector3 position);

//    virtual void createFrameListener(void);
    virtual void destroyScene(void);
//    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    /**
      * Handle keyboard input when keys are pressed.
      **/
    virtual bool keyPressed( const OIS::KeyEvent &arg );

    virtual bool frameRenderingQueued(const Ogre::FrameEvent &evt);

    virtual bool queryCursorPosition(Ogre::Vector3 &rayHitPoint, unsigned long queryflags = NAVMESH_MASK, bool clipToNavmesh = true, Ogre::MovableObject **rayHitObject = NULL);


private:
    void defineTerrain(long x, long y);
    void initBlendMaps(Ogre::Terrain* terrain);
    void configureTerrainDefaults(Ogre::Light* light);

    Ogre::TerrainGlobalOptions* mTerrainGlobals;
    Ogre::TerrainGroup* mTerrainGroup;
    bool mTerrainsImported;

    InputGeom *mGeom;

};

#endif // OGRERECASTTERRAINAPPLICATION_H
