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

//-------------------------------------------------------------------------------------
OgreRecastApplication::OgreRecastApplication(void)
        : mRecastDemo(0)
{
}
//-------------------------------------------------------------------------------------
OgreRecastApplication::~OgreRecastApplication(void)
{
}

//-------------------------------------------------------------------------------------
void OgreRecastApplication::createScene(void)
{
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight( "MainLight" );
    light->setPosition(20, 80, 50);

    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    Ogre::SceneNode* mapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MapNode");
    mapNode->attachObject(mapE);

    // RECAST
    // Create the navmesh and show it
    mRecastDemo = new OgreRecastDemo(mSceneMgr);
    if(mRecastDemo->NavMeshBuild(mapE)) {
        mRecastDemo->drawNavMesh();
    } else {
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not generate useable navmesh from mesh.");
        return;
    }

    // DETOUR
    // Do a pathing on the navmesh and draw the path
    int pathNb = 0;     // The index number for the slot in which the found path is to be stored
    int targetId = 0;   // Number identifying the target the path leads to
    float startPos[3]; mRecastDemo->OgreVect3ToFloatA(Ogre::Vector3(0,0,0), startPos);
    float endPos[3]; mRecastDemo->OgreVect3ToFloatA(Ogre::Vector3(5,0,5), endPos);

    int ret = mRecastDemo->FindPath(startPos, endPos, pathNb, targetId) ;
    if( ret >= 0)
        mRecastDemo->CreateRecastPathLine(0) ; // Draw a line showing path at slot 0
    else
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a path. ("+Ogre::StringConverter::toString(ret)+")");
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
        // Create application object
        OgreRecastApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
