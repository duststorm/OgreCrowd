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
    mPaged = false;
    mInstancedCrowd = false;

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
    } else if(equals(optionName, "Paged Crowds Demo")) {
        mPaged = getBool(optionValue);
        return true;
    } else if(equals(optionName, "Instanced Crowd Characters")) {
        mInstancedCrowd = getBool(optionValue);
        return true;



    } else if(equals(optionName, "Terrain Tiles X")) {
        mTerrainTilesX = (int) getReal(optionValue);
        return true;
    } else if(equals(optionName, "Terrain Tiles Z")) {
        mTerrainTilesZ = (int) getReal(optionValue);
        return true;
    } else if(equals(optionName, "Terrain Tile Size")) {
        mTerrainTileSize = getReal(optionValue);
        return true;
    } else if(equals(optionName, "Terrain Height Scale")) {
        mTerrainHeightScale = getReal(optionValue);
        return true;
    } else if(equals(optionName, "Terrain Tile Resolution")) {
        mTerrainTileResolution = getReal(optionValue);
        return true;
    }


    std::cout << "Warning: Unknown option \"" << optionName << "\" specified in " << mFilePath << " file." << std::endl;
    return false;
}

