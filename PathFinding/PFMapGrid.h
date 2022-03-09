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
    typedef std::vector<std::shared_ptr<PFMapGridNode>> PFMapNodes;

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

    inline PFMapGridNode* GetMapNode(unsigned int i, unsigned int j)
    {
      assert(i < mX&& j < mY);
      unsigned int x = i * mY + j;
      return mCells[x].get();
    }

    inline const PFMapGridNode* GetMapNode(unsigned int i, unsigned int j) const
    {
      assert(i < mX&& j < mY);
      unsigned int x = i * mY + j;
      return mCells[x].get();
    }

    std::vector<const PathFinder::Node*> GetNeighbourCells(const PFMapGridNode& loc) const;
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
