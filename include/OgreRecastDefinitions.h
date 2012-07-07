#ifndef __OgreRecastDefinitions_h_
#define __OgreRecastDefinitions_h_



/**
  * This file sets up all definitions needed by Recast/Detour.
  * Most of it is just taken from the official demo application.
  **/

// recast/detour stuff
#include "Recast/Recast.h"
#include "Detour/DetourNavMesh.h"
#include "Detour/DetourNavMeshBuilder.h"
#include "Detour/DetourNavMeshQuery.h"

#define MAX_PATHSLOT      128 // how many paths we can store
#define MAX_PATHPOLY      256 // max number of polygons in a path
#define MAX_PATHVERT      512 // most verts in a path 

// structure for storing output straight line paths
typedef struct
{
   float PosX[MAX_PATHVERT] ;
   float PosY[MAX_PATHVERT] ;
   float PosZ[MAX_PATHVERT] ;
   int MaxVertex ;
   int Target ;
}
PATHDATA ;

// These are just sample areas to use consistent values across the samples.
// The use should specify these base on his needs.

// bzn most aren't used yet, just SAMPLE_POLYAREA_GROUND and SAMPLE_POLYFLAGS_WALK
enum SamplePolyAreas
{
   SAMPLE_POLYAREA_GROUND,
   SAMPLE_POLYAREA_WATER,
   SAMPLE_POLYAREA_ROAD,
   SAMPLE_POLYAREA_DOOR,
   SAMPLE_POLYAREA_GRASS,
   SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
   SAMPLE_POLYFLAGS_WALK = 0x01,      // Ability to walk (ground, grass, road)
   SAMPLE_POLYFLAGS_SWIM = 0x02,      // Ability to swim (water).
   SAMPLE_POLYFLAGS_DOOR = 0x04,      // Ability to move through doors.
   SAMPLE_POLYFLAGS_JUMP = 0x08,      // Ability to jump.
   SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
   SAMPLE_POLYFLAGS_ALL = 0xffff      // All abilities.
};


#endif // #ifndef __OgreRecastDefinitions_h_
