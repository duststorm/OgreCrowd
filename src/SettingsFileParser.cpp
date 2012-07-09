#include "SettingsFileParser.h"

SettingsFileParser::SettingsFileParser(Ogre::String filePath)
    : mFilePath(filePath)
{
    // Set default settings
    mRestoreConfig = false;
    mLockMouse = true;

    mDebugDraw = true;
    mHumanChars = true;
    mObstacles = true;
    mSingleNavmesh = false;
    mRaycastScene = false;
    mTempObstacleSteering = true;
    mComplexObstacles = true;
    mTerrain = false;

    mTerrainTilesX = 1;
    mTerrainTilesZ = 1;
    mTerrainTileSize = 12000.0f;
    mTerrainTileResolution = 513;
    mTerrainHeightScale = 1.0f;

    // Load settings from config file
    Ogre::ConfigFile settingsFile = Ogre::ConfigFile();
    settingsFile.loadDirect(filePath);

    Ogre::ConfigFile::SectionIterator sectIt = settingsFile.getSectionIterator();
    Ogre::String sectionName, optionName, optionValue;
    while(sectIt.hasMoreElements())
    {
        sectionName = sectIt.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *sectSettings = sectIt.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator it;
        for(it = sectSettings->begin(); it != sectSettings->end(); it++) {
            optionName = it->first;
            optionValue = it->second;
            addOption(sectionName, optionName, optionValue);
        }
    }
}

bool SettingsFileParser::equals(Ogre::String str1, Ogre::String str2)
{
    Ogre::StringUtil::toLowerCase(str1);
    Ogre::StringUtil::toLowerCase(str2);
    return (str1 == str2);
}

bool SettingsFileParser::getBool(Ogre::String value)
{
    return Ogre::StringConverter::parseBool(value);
}

Ogre::Real SettingsFileParser::getReal(Ogre::String value)
{
    return Ogre::StringConverter::parseReal(value);
}

bool SettingsFileParser::addOption(Ogre::String sectionName, Ogre::String optionName, Ogre::String optionValue)
{
    if(equals(optionName, "Restore Config")) {
        mRestoreConfig = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Grab Mouse")) {
        mLockMouse = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Debug Draw")) {
        mDebugDraw = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Human Characters")) {
        mHumanChars = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Obstacles")) {
        mObstacles = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Single Navmesh")) {
        mSingleNavmesh = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Raycast Scene")) {
        mRaycastScene = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Temp Obstacle Steering")) {
        mTempObstacleSteering = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Complex Obstacles")) {
        mComplexObstacles = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Terrain Demo")) {
        mTerrain = getBool(optionValue);
        return true;


    } else if(equals(optionName, "Terrain Tiles X")) {
        mTerrainTilesX = (int) getReal(optionValue);
    } else if(equals(optionName, "Terrain Tiles Z")) {
        mTerrainTilesZ = (int) getReal(optionValue);
    } else if(equals(optionName, "Terrain Tile Size")) {
        mTerrainTileSize = getReal(optionValue);
    } else if(equals(optionName, "Terrain Height Scale")) {
        mTerrainHeightScale = getReal(optionValue);
    } else if(equals(optionName, "Terrain Tile Resolution")) {
        mTerrainTileResolution = getReal(optionValue);
    }


    std::cout << "Warning: Unknown option \"" << optionName << "\" specified in " << mFilePath << " file." << std::endl;
    return false;
}

