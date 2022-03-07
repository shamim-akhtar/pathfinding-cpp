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

  class GridMap : public osg::Referenced
  {
  public:

    class GridCell : public Node
    {
    public:
      Point2di Value;

      virtual bool operator==(const Node& other)
      {
        const GridCell* b = dynamic_cast<const GridCell*>(&other);
        if (b == 0)
          return false;

        return b->Value.x == Value.x && b->Value.y == Value.y;
      }
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
        : Node()
        , mGridMap(gridMap)
        , mIsWalkable(true)
        , Value(value)
      {
      }

      // get the neighbours for this cell.
      // here will will just throw the responsibility
      // to get the neighbours to the grid.
      std::vector<osg::ref_ptr<Node>>
        GetNeighbours() override
      {
        return mGridMap.GetNeighbourCells(*this);
      }

    private:
      bool mIsWalkable;
      GridMap& mGridMap;
    };

    class EuclideancCost : public CostFunction
    {
    public:
      float operator()(const Node& a, const Node& b)
      {
        assert(dynamic_cast<const GridCell*>(&a));
        assert(dynamic_cast<const GridCell*>(&b));
        return Distance(static_cast<const GridCell&>(a).Value, static_cast<const GridCell&>(b).Value);
      }

      float Distance(const Point2di& a, const Point2di& b)
      {
        return sqrtf(
          (a.x - b.x) * (a.x - b.x) +
          (a.y - b.y) * (a.y - b.y)
        );
      }
    };

    class ManhattanCost : public CostFunction
    {
    public:
      float operator()(const Node& a, const Node& b)
      {
        assert(dynamic_cast<const GridCell*>(&a));
        assert(dynamic_cast<const GridCell*>(&b));
        return ManhattanDistance(static_cast<const GridCell&>(a).Value, static_cast<const GridCell&>(b).Value);
      }

      float ManhattanDistance(const Point2di& a, const Point2di& b)
      {
        return abs(a.x - b.x) + abs(a.y - b.y);
      }
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
          osg::ref_ptr<GridCell> sp(
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

    osg::ref_ptr<GridCell> GetCell(int i, int j)
    {
      assert(i < mX && j < mY);
      return mCells[(long)(i * mY + j)];
    }

    std::vector<osg::ref_ptr<Node>> 
      GetNeighbourCells(const GridCell& loc)
    {
      std::vector<osg::ref_ptr<Node>> neighbours;

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
        osg::ref_ptr<GridCell> cell = GetCell(i, j);
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
    typedef std::vector<osg::ref_ptr<GridCell>> GridCells;
    GridCells mCells;
  };

}
