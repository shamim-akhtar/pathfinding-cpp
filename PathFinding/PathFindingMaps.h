#pragma once
#include "PathFinder.h"
#include <memory>
#include <cassert>

/// <summary>
/// This file comprises classes associated with 
/// pathfinding map representations.
/// </summary>
namespace PathFinding
{
  struct Point2di
  {
  public:
    Point2di()
      : x(0)
      , y(0)
    {
    }
    Point2di(int _x, int _y)
      : x(_x)
      , y(_y)
    {
    }
    int x;
    int y;
  };

  class GridMap
  {
  public:

    class GridCell : public Node<Point2di>
    {
    public:
      // Is this cell walkable?
      bool GetIsWalkable()
      {
        return mIsWalkable;
      }

      void SetWalkable(bool flag)
      {
        mIsWalkable = flag;
      }

      // construct the node with the grid and the location.
      GridCell(GridMap& gridMap, Point2di value)
        : Node<Point2di>(value)
        , mGridMap(gridMap)
        , mIsWalkable(true)
      {
      }

      // get the neighbours for this cell.
      // here will will just throw the responsibility
      // to get the neighbours to the grid.
      std::vector<std::shared_ptr<Node<Point2di>>>
        GetNeighbours() override
      {
        return mGridMap.GetNeighbourCells(*this);
      }

    private:
      bool mIsWalkable;
      GridMap& mGridMap;
    };

    // helper methods.
    static GridMap* CreateRandomGridMap(int numX, int numY);
  public:
    explicit GridMap(int numX, int numY)
      : mX(numX)
      , mY(numY)
      , AllowDiagonalMovement(true)
    {
      mCells.reserve((long)(mX * mY));
      for (int i = 0; i < mX; ++i)
      {
        for (int j = 0; j < mY; ++j)
        {
          std::shared_ptr<GridCell> sp(
            new GridCell(*this, Point2di(i, j)));
          mCells.push_back(sp);
        }
      }
    }

    inline int GetNumX() const
    {
      return mX;
    }

    inline int GetNumY() const
    {
      return mY;
    }

    bool AllowDiagonalMovement;

    std::shared_ptr<GridCell> GetCell(int i, int j)
    {
      assert(i < mX && j < mY);
      return mCells[(long)(i * mY + j)];
    }

    std::vector<std::shared_ptr<Node<Point2di>>> 
      GetNeighbourCells(const GridCell& loc)
    {
      std::vector<std::shared_ptr<Node<Point2di>>> neighbours;

      int x = loc.Value.x;
      int y = loc.Value.y;

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
        std::shared_ptr<GridCell> cell = GetCell(i, j);
        if (cell->GetIsWalkable())
        {
          neighbours.push_back(cell);
        }
      }
      return neighbours;
    }

  private:
    int mX;
    int mY;
    typedef std::vector<std::shared_ptr<GridCell>> GridCells;
    GridCells mCells;
  };

}
