#include "PFMapGridNode.h"
#include "PFMapGrid.h"

namespace PathFinding
{
  // get the neighbours for this cell.
  // here will will just throw the responsibility
  // to get the neighbours to the grid.
  std::vector<PFNode*> PFMapGridNode::GetNeighbours()
  {
    return mGridMap.GetNeighbourCells(*this);
  }
}