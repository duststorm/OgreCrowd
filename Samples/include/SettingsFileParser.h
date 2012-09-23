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

#ifndef SETTINGSFILEPARSER_H
#define SETTINGSFILEPARSER_H



#include <Ogre.h>

/**
  * Simple settings parser for the config file.
  * Quickly put together mainly for purposes of this demo, so that settings
  * can be changed without recompiling.
  **/
class SettingsFileParser
{
public:
    /**
      * Create a parser and parse the specified settings file.
      **/
    SettingsFileParser(Ogre::String filePath);

    bool addOption(Ogre::String sectionName, Ogre::String name, Ogre::String value);

    bool addGeneralOption(Ogre::String optionName, Ogre::String optionValue);
    bool addNetworkOption(Ogre::String optionName, Ogre::String optionValue);
    bool addFeaturesOption(Ogre::String optionName, Ogre::String optionValue);
    bool addDebugOption(Ogre::String optionName, Ogre::String optionValue);
    bool addCameraOption(Ogre::String optionName, Ogre::String optionValue);
    bool addTrackOption(Ogre::String optionName, Ogre::String optionValue);
    bool addRunnersOption(Ogre::String optionName, Ogre::String optionValue);
    bool addCaelumOption(Ogre::String optionName, Ogre::String optionValue);
    bool addTrackOverlayOption(Ogre::String optionName, Ogre::String optionValue);
    bool addVideostreamerOption(Ogre::String optionName, Ogre::String optionValue);

    bool equals(Ogre::String str1, Ogre::String str2);
    bool getBool(Ogre::String value);
    Ogre::Real getReal(Ogre::String value);

    Ogre::String mFilePath;


    bool mDebugDraw;
    bool mHumanChars;
    bool mObstacles;
    bool mSingleNavmesh;
    bool mRaycastScene;
    bool mTempObstacleSteering;
    bool mComplexObstacles;
    bool mTerrain;
    bool mPaged;
    bool mInstancedCrowd;

    bool mRestoreConfig;
    bool mLockMouse;

    int mTerrainTilesX;
    int mTerrainTilesZ;
    float mTerrainTileSize;
    int mTerrainTileResolution;
    float mTerrainHeightScale;
};


#endif // SETTINGSFILEPARSER_H
