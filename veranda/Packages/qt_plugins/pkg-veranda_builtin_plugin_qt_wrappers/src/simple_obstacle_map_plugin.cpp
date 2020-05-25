#include "simple_obstacle_map_plugin.h"
#include "veranda_perception/simple_obstacle_map.h"

WorldObjectComponent *SimpleObstacleMap_Plugin::createComponent()
{
    return new SimpleObstacleMap(SIMPLEOBSTACLEMAP_IID);
}
