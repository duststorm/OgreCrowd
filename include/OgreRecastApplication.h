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

class OgreRecastApplication : public BaseApplication
{
public:
    OgreRecastApplication(void);
    virtual ~OgreRecastApplication(void);

protected:
    virtual void createScene(void);

private:
        OgreRecastDemo* mRecastDemo;
};

#endif // #ifndef __OgreRecastApplication_h_
