#include "PathFindingMaps.h"

using namespace PathFinding;

GridMap* GridMap::CreateRandomGridMap(int numX, int numY)
{
  GridMap* grid = new GridMap(numX, numY);
  return grid;
}