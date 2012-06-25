# -------------------------------------------------
# QtCreator project for OgreRecast demo
# -------------------------------------------------
TARGET = OgreRecastDemo
TEMPLATE = app

# Enable Gprof profiling
QMAKE_CXXFLAGS_DEBUG += -pg
QMAKE_LFLAGS_DEBUG += -pg
unix { 
    message("--- Unix specific configuration ---")
    
    # Location of Ogre main libs
    # You may need to change this include directory
    INCLUDEPATH += /usr/include/OGRE
    
    # Include Ogre Terrain
    INCLUDEPATH += /usr/include/OGRE/Terrain
    LIBS += -L/usr/lib/ \
     -lOgreTerrain
    # Include Ogre Camera Control System addon (needs to be compiled and manually installed separately)
    
    # Include OIS
    INCLUDEPATH += /usr/include/OIS
    LIBS += -L/usr/lib/ \
        -lOIS
    
    # Include CEGUI libs (Crazy Eddys GUI)
    INCLUDEPATH += /usr/local/include/CEGUI
    LIBS += -L/usr/local/lib/CEGUI \
        -L/usr/local/lib # \
    
    # INCLUDEPATH += dependencies/Caelum/include
    # INCLUDEPATH += dependencies/PagedGeometry/include
    # INCLUDEPATH += dependencies/CameraControlSystem/include
    # Include Berkelium libs
    # INCLUDEPATH += dependencies/Berkelium/include
    # LIBS += -Ldependencies/Berkelium/berkelium-linux32/lib \
    # -llibberkelium
    CONFIG += link_pkgconfig
    PKGCONFIG += OGRE
}
win32 { 
    message("--- Windows specific configuration ---")
    
    # Location of Ogre main libs (SDK)
    # You may need to change this include directory
    OGREHOME = D:\ogrehead\sdk
    LIBS += -LC:\boost\boost_1_40\lib
    Release:LIBS += -L$$OGREHOME\lib\release
    Debug:LIBS += -L$$OGREHOME\lib\debug
    INCLUDEPATH += $$OGREHOME\include\OGRE
    INCLUDEPATH += C:\boost\boost_1_40
    
    # Application icon
    RC_FILE = res/resource.rc
}

# TODO might need altering, copied from other project and untested
macx { 
    message("--- Mac OS X specific configuration ---")
    CONFIG += x86 \
        ppc
    LIBS += -framework \
        Ogre \
        -framework \
        AGL
    INCLUDEPATH += /Library/Frameworks/Ogre.framework/Headers
    
    # create link to data dir in the build dir
    system(mkdir build)
    system(ln -fs ../Data build/Data)
}

# Load project internal includes
INCLUDEPATH += include

# Load external dependencies included in project folder
# INCLUDEPATH += dependencies/include
# Helper functions
CONFIG(debug, debug|release) { 
    TARGET = $$join(TARGET,,,d)
    
    # LIBS *= -lOgreMain_d
    # Werk altijd met non-debug ogre libs:
    LIBS *= -lOgreMain
}
CONFIG(release, debug|release):LIBS *= -lOgreMain

# Project files
message("--- Project Input Files ---")
SOURCES += src/BaseApplication.cpp \
    src/OgreRecastApplication.cpp \
    src/OgreRecast.cpp \
    src/Detour/DetourAlloc.cpp \
    src/Detour/DetourNavMeshQuery.cpp \
    src/Detour/DetourNavMesh.cpp \
    src/Detour/DetourNavMeshBuilder.cpp \
    src/Detour/DetourCommon.cpp \
    src/Detour/DetourNode.cpp \
    src/DetourCrowd/DetourCrowd.cpp \
    src/DetourCrowd/DetourPathQueue.cpp \
    src/DetourCrowd/DetourPathCorridor.cpp \
    src/DetourCrowd/DetourObstacleAvoidance.cpp \
    src/DetourCrowd/DetourLocalBoundary.cpp \
    src/DetourCrowd/DetourProximityGrid.cpp \
    src/Recast/RecastAlloc.cpp \
    src/Recast/RecastRasterization.cpp \
    src/Recast/RecastMeshDetail.cpp \
    src/Recast/RecastMesh.cpp \
    src/Recast/RecastLayers.cpp \
    src/Recast/RecastFilter.cpp \
    src/Recast/Recast.cpp \
    src/Recast/RecastContour.cpp \
    src/Recast/RecastArea.cpp \
    src/Recast/RecastRegion.cpp \
    src/OgreDetourCrowd.cpp \
    src/Character.cpp \
    src/TestCharacter.cpp \
    src/AnimateableCharacter.cpp \
    src/DetourTileCache/DetourTileCacheBuilder.cpp \
    src/DetourTileCache/DetourTileCache.cpp \
    src/OgreDetourTileCache.cpp \
    src/RecastContrib/fastlz/fastlz.c \
    src/RecastInputGeom.cpp \
    src/RecastConvexHull.cpp \
    src/Obstacle.cpp \
    src/CylinderObstacle.cpp \
    src/ConvexShapeObstacle.cpp \
    src/OgreRecastTerrainApplication.cpp \
    src/SettingsFileParser.cpp
HEADERS += include/BaseApplication.h \
    include/OgreRecastApplication.h \
    include/OgreRecastDefinitions.h \
    include/OgreRecast.h \
    include/Detour/DetourCommon.h \
    include/Detour/DetourAlloc.h \
    include/Detour/DetourNode.h \
    include/Detour/DetourNavMeshQuery.h \
    include/Detour/DetourNavMesh.h \
    include/Detour/DetourNavMeshBuilder.h \
    include/Detour/DetourCommon.h \
    include/Detour/DetourAssert.h \
    include/Detour/DetourStatus.h \
    include/Recast/RecastAlloc.h \
    include/Recast/RecastAssert.h \
    include/Recast/Recast.h \
    include/DetourCrowd/DetourCrowd.h \
    include/DetourCrowd/DetourPathQueue.h \
    include/DetourCrowd/DetourPathCorridor.h \
    include/DetourCrowd/DetourObstacleAvoidance.h \
    include/DetourCrowd/DetourLocalBoundary.h \
    include/DetourCrowd/DetourProximityGrid.h \
    include/OgreDetourCrowd.h \
    include/Character.h \
    include/TestCharacter.h \
    include/AnimateableCharacter.h \
    include/DetourTileCache/DetourTileCacheBuilder.h \
    include/DetourTileCache/DetourTileCache.h \
    include/OgreDetourTileCache.h \
    include/RecastContrib/fastlz/fastlz.h \
    include/RecastInputGeom.h \
    include/RecastConvexHull.h \
    include/Obstacle.h \
    include/CylinderObstacle.h \
    include/ConvexShapeObstacle.h \
    include/OgreRecastTerrainApplication.h \
    include/SettingsFileParser.h
