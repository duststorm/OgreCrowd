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

    bool mRestoreConfig;
    bool mLockMouse;

    int mTerrainTilesX;
    int mTerrainTilesZ;
    float mTerrainTileSize;
    int mTerrainTileResolution;
    float mTerrainHeightScale;
};


#endif // SETTINGSFILEPARSER_H
