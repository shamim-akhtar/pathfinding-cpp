#include "PFMapGrid.h"

namespace PathFinding
{
  PFMapGrid* PFMapGrid::CreateRandomGridMap(int numX, int numY)
  {
    return new PFMapGrid(numX, numY);
  }
  PFMapGrid::PFMapGrid(unsigned int numX, unsigned int numY)
    : mX(numX)
    , mY(numY)
    , AllowDiagonalMovement(true)
  {
    mCells.reserve(mX * mY);
    for (unsigned int i = 0; i < mX; ++i)
    {
      for (unsigned int j = 0; j < mY; ++j)
      {
        PFMapGridNode* sp = new PFMapGridNode(*this, Point2di(i, j));
        mCells.push_back(sp);
      }
    }
  }

  std::vector<PFNode*> PFMapGrid::GetNeighbourCells(const PFMapGridNode& loc)
  {
    std::vector<PFNode*> neighbours;

    int x = loc.Point.x;
    int y = loc.Point.y;

    // Check up.
    if (y < mY - 1)
    {
      int i = x;
      int j = y + 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check top-right
    if (AllowDiagonalMovement && (y < mY - 1 && x < mX - 1))
    {
      int i = x + 1;
      int j = y + 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check right
    if (x < mX - 1)
    {
      int i = x + 1;
      int j = y;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check right-down
    if (AllowDiagonalMovement && (x < mX - 1 && y > 0))
    {
      int i = x + 1;
      int j = y - 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check down
    if (y > 0)
    {
      int i = x;
      int j = y - 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check down-left
    if (AllowDiagonalMovement && (y > 0 && x > 0))
    {
      int i = x - 1;
      int j = y - 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check left
    if (x > 0)
    {
      int i = x - 1;
      int j = y;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    // Check left-top
    if (AllowDiagonalMovement && (x > 0 && y < mY - 1))
    {
      int i = x - 1;
      int j = y + 1;

      if (GetCell(i, j)->GetIsWalkable())
      {
        neighbours.push_back(GetCell(i, j));
      }
    }
    return neighbours;
  }
}