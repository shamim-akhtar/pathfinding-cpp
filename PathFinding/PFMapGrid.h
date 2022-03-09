#pragma once
#include "PathFinder.h"
#include "PFMapGridNode.h"
#include <memory>
#include <cassert>

namespace PathFinding
{
  class PATHFINDING_EXPORT PFMapGrid
  {
  public:
    // helper methods.
    static PFMapGrid* CreateRandomGridMap(int numX, int numY);
  public:
    typedef std::vector<PFMapGridNode*> PFMapNodes;

    explicit PFMapGrid(unsigned int numX, unsigned int numY);

    inline unsigned int GetNumX() const
    {
      return mX;
    }

    inline unsigned int GetNumY() const
    {
      return mY;
    }

    bool AllowDiagonalMovement;

    inline PFMapGridNode* GetCell(unsigned int i, unsigned int j)
    {
      assert(i < mX&& j < mY);
      unsigned int x = i * mY + j;
      return mCells[x];
    }

    inline const PFMapGridNode* GetCell(unsigned int i, unsigned int j) const
    {
      assert(i < mX&& j < mY);
      unsigned int x = i * mY + j;
      return mCells[x];
    }

    std::vector<PFNode*> GetNeighbourCells(const PFMapGridNode& loc);
    inline const PFMapNodes& GetMapNodes() const
    {
      return mCells;
    }
    inline PFMapNodes& GetMapNodes()
    {
      return mCells;
    }

  private:
    unsigned int mX;
    unsigned int mY;
    PFMapNodes mCells;
  };

}
