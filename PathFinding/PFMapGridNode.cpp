#include "PFMapGridNode.h"
#include "PFMapGrid.h"

namespace PathFinding
{
  // get the neighbours for this cell.
  // here will will just throw the responsibility
  // to get the neighbours to the grid.
  std::vector<const PFNode*> PFMapGridNode::GetNeighbours() const
  {
    return mGridMap.GetNeighbourCells(*this);
  }
}