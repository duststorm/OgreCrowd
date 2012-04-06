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
        : mRecastDemo(0),
        mRayScnQuery(0)
{
}
//-------------------------------------------------------------------------------------
OgreRecastApplication::~OgreRecastApplication(void)
{
}

//-------------------------------------------------------------------------------------
void OgreRecastApplication::createScene(void)
{
    Ogre::MovableObject::setDefaultQueryFlags(NAVMESH_MASK);
    mSceneMgr->getRootSceneNode()->_update(true, false);

    // Create navigateable dungeon
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    Ogre::Light* light = mSceneMgr->createLight( "MainLight" );
    light->setPosition(20, 80, 50);

    Ogre::Entity* mapE = mSceneMgr->createEntity("Map", "dungeon.mesh");
    mapE->setMaterialName("dungeon");
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
    Ogre::Vector3 beginPos = mRecastDemo->getRandomNavMeshPoint();
    Ogre::Vector3 endPos = mRecastDemo->getRandomNavMeshPoint();

    int ret = mRecastDemo->FindPath(beginPos, endPos, pathNb, targetId) ;
    if( ret >= 0)
        mRecastDemo->CreateRecastPathLine(0) ; // Draw a line showing path at slot 0
    else
        Ogre::LogManager::getSingletonPtr()->logMessage("ERROR: could not find a path. ("+Ogre::StringConverter::toString(ret)+")");


    // SETUP RAY SCENE QUERYING
    // Add navmesh to separate querying group
    Ogre::SceneNode *navMeshNode = (Ogre::SceneNode*)mSceneMgr->getRootSceneNode()->getChild("RecastSN");
    navMeshNode->getAttachedObject("RecastMOWalk")->setQueryFlags(NAVMESH_MASK);
    // Exclude other meshes from navmesh queries
    navMeshNode->getAttachedObject("RecastMONeighbour")->setQueryFlags(DEFAULT_MASK);
    navMeshNode->getAttachedObject("RecastMOBoundary")->setQueryFlags(DEFAULT_MASK);
    mapE->setQueryFlags(DEFAULT_MASK);

    // TODO: Voeg steering toe
    // TODO: Agents plaatsen
    // TODO: voeg meerdere paden toe
    // TODO: DetourCrowd toevoegen
}


bool OgreRecastApplication::rayQueryPointInScene(Ogre::Ray ray, unsigned long queryMask, Ogre::Vector3 &result, Ogre::MovableObject &foundMovable)
{
    //if(!mRayScnQuery)
        mRayScnQuery = mSceneMgr->createRayQuery(Ogre::Ray(), queryMask);

    mRayScnQuery->setRay(ray);
    Ogre::RaySceneQueryResult& query_result = mRayScnQuery->execute();


        // at this point we have raycast to a series of different objects bounding boxes.
        // we need to test these different objects to see which is the first polygon hit.
        // there are some minor optimizations (distance based) that mean we wont have to
        // check all of the objects most of the time, but the worst case scenario is that
        // we need to test every triangle of every object.
        Ogre::Real closest_distance = -1.0f;
        Ogre::Vector3 closest_result;
        Ogre::MovableObject *closest_movable;
        for (size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++)
        {
            Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getName());
            Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getMovableType());


            // stop checking if we have found a raycast hit that is closer
            // than all remaining entities
            if ((closest_distance >= 0.0f) &&
                (closest_distance < query_result[qr_idx].distance))
            {
                 break;
            }

            // only check this result if its a hit against an entity
            if ((query_result[qr_idx].movable != NULL) &&
                ((query_result[qr_idx].movable->getMovableType().compare("Entity") == 0)
                ||query_result[qr_idx].movable->getMovableType().compare("ManualObject") == 0))
            {
                // mesh data to retrieve
                size_t vertex_count;
                size_t index_count;
                Ogre::Vector3 *vertices;
                unsigned long *indices;

                // get the mesh information
                if(query_result[qr_idx].movable->getMovableType().compare("Entity") == 0) {
                    // For entities
                    // get the entity to check
                    Ogre::Entity *pentity = static_cast<Ogre::Entity*>(query_result[qr_idx].movable);

                    mRecastDemo->getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                                  pentity->getParentNode()->_getDerivedPosition(),
                                  pentity->getParentNode()->_getDerivedOrientation(),
                                  pentity->getParentNode()->_getDerivedScale());
                } else {
                    // For manualObjects
                    // get the entity to check
                    Ogre::ManualObject *pmanual = static_cast<Ogre::ManualObject*>(query_result[qr_idx].movable);

                    mRecastDemo->getManualMeshInformation(pmanual, vertex_count, vertices, index_count, indices,
                                  pmanual->getParentNode()->_getDerivedPosition(),
                                  pmanual->getParentNode()->_getDerivedOrientation(),
                                  pmanual->getParentNode()->_getDerivedScale());
                }

                // test for hitting individual triangles on the mesh
                bool new_closest_found = false;
                for (int i = 0; i < static_cast<int>(index_count); i += 3)
                {
                    // check for a hit against this triangle
                    std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
                        vertices[indices[i+1]], vertices[indices[i+2]], true, false);

                    // if it was a hit check if its the closest
                    if (hit.first)
                    {
                        if ((closest_distance < 0.0f) ||
                            (hit.second < closest_distance))
                        {
                            // this is the closest so far, save it off
                            closest_distance = hit.second;
                            new_closest_found = true;
                        }
                    }
                }


             // free the verticies and indicies memory
                delete[] vertices;
                delete[] indices;

                // if we found a new closest raycast for this object, update the
                // closest_result before moving on to the next object.
                if (new_closest_found)
                {
                    closest_result = ray.getPoint(closest_distance);
                    if(query_result[qr_idx].movable != NULL)
                        closest_movable = query_result[qr_idx].movable;
                }
            }
        }

        // return the result
        if (closest_distance >= 0.0f)
        {
            // raycast success
            result = closest_result;
            //foundMovable = *closest_movable;  // TODO fix this!
            return (true);
        }
        else
        {
            // raycast failed
            return (false);
        }
}

bool OgreRecastApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    // Do ray scene query
    //send a raycast straight out from the camera at the center position
    Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(0.5, 0.5);

    Ogre::Vector3 rayHitPoint;
    Ogre::MovableObject *rayHitObject;
    if (rayQueryPointInScene(mouseRay, NAVMESH_MASK, rayHitPoint, *rayHitObject)) {
        Ogre::SceneNode *markerNode = NULL;

        if(id == OIS::MB_Left) {
            markerNode = getOrCreateMarker("BeginPos", "Cylinder/Wires/DarkGreen");
        }

        if(id == OIS::MB_Right) {
            markerNode = getOrCreateMarker("EndPos", "Cylinder/Wires/Brown");
        }

        if(markerNode != NULL) {
            markerNode->setPosition(rayHitPoint);
        }
    }

    BaseApplication::mousePressed(arg, id);
}

Ogre::SceneNode* OgreRecastApplication::getOrCreateMarker(Ogre::String name, Ogre::String materialName)
{
    Ogre::SceneNode *result  = NULL;
    try {
        result = (Ogre::SceneNode*)(mSceneMgr->getRootSceneNode()->getChild(name+"Node"));
    } catch(Ogre::Exception ex) {
        result = mSceneMgr->getRootSceneNode()->createChildSceneNode(name+"Node");
        Ogre::Entity* ent = mSceneMgr->createEntity(name, "Cylinder.mesh");
        if(materialName.compare("") != 0)
            ent->setMaterialName(materialName);
        result->attachObject(ent);
        ent->setQueryFlags(DEFAULT_MASK);   // Exclude from ray queries
        // Set marker scale to size of agent
        result->setScale(mRecastDemo->m_agentRadius*2, mRecastDemo->m_agentHeight,mRecastDemo->m_agentRadius*2);
    }

    return result;
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
