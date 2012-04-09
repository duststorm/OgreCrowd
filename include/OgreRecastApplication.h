/*
-----------------------------------------------------------------------------
Filename:    OgreRecastApplication.h
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
#ifndef __OgreRecastApplication_h_
#define __OgreRecastApplication_h_

#include "BaseApplication.h"
#include "OgreRecast.h"
#include "OgreRecastDemo.h"
#include "OgreDetourCrowd.h"

class OgreRecastApplication : public BaseApplication
{
public:
    OgreRecastApplication(void);
    virtual ~OgreRecastApplication(void);

    void func(int a);
    bool func2(Ogre::Ray a);

    bool rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject& foundMovable);
    Ogre::SceneNode* getOrCreateMarker(Ogre::String name, Ogre::String materialName="");

    enum QueryFlags {
       DEFAULT_MASK = 0u,
       NAVMESH_MASK = 5u,
    };


protected:
    virtual void createScene(void);
    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    virtual bool keyPressed( const OIS::KeyEvent &arg );
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
    void drawPathBetweenMarkers(int pathNb, int targetId);
    void setPathAndBeginMarkerVisibility(bool visibility);
    void setRandomTargetsForCrowd(void);
    void setFollowTargetForCrowd(Ogre::Vector3 targetDestination);

    Ogre::Vector3 getFirstAgentPosition(void);

    enum ApplicationState
    {
       SIMPLE_PATHFIND,
       CROWD_WANDER,
       FOLLOW_TARGET
    } mApplicationState;

    float mMinSquaredDistanceToGoal;

    Ogre::Vector3 mDestinations[OgreDetourCrowd::MAX_AGENTS];

private:
        OgreRecastDemo* mRecastDemo;
        Ogre::RaySceneQuery* mRayScnQuery;

        OgreDetourCrowd *mDetourCrowd;
};


#endif // #ifndef __OgreRecastApplication_h_
