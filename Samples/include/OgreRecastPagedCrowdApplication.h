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

#ifndef OGRERECASTPAGEDCROWDAPPLICATION_H
#define OGRERECASTPAGEDCROWDAPPLICATION_H

#include "BaseApplication.h"
#include "Character.h"
class CrowdManager;

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


    static const Ogre::Real TOPDOWN_CAMERA_HEIGHT;
    static const bool EXTRACT_WALKABLE_AREAS;

protected:
    /**
      * Initialise the scene and everything needed for pathfinding and steering.
      * Setup demo specific constructions.
      **/
    virtual void createScene(void);

    virtual bool keyPressed(const OIS::KeyEvent &arg);

    virtual bool keyReleased(const OIS::KeyEvent &arg);

    /**
      * Update state for rendering a new frame.
      **/
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual void createFrameListener(void);

    void updateDebugInfo(void);



    /**
      * Ogre version of the code of the original Recast demo that comes with the recast distribution.
      **/
    OgreRecast* mRecast;

    /**
      * Ogre version of DetourTileCache.
      * Only used when SINGLE_NAVMESH is false.
      **/
    OgreDetourTileCache *mDetourTileCache;

    CrowdManager *mCrowdManager;

    OgreBites::ParamsPanel* mDebugPanel;     // sample details panel


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

    /**
      * Related to Top-down camera mode
      **/
    bool mTopDownCamera;
    bool mGoingUp;
    bool mGoingDown;
    bool mGoingLeft;
    bool mGoingRight;

};

#endif // OGRERECASTPAGEDCROWDAPPLICATION_H
