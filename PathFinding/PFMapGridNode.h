#pragma once
#include "PathFinder.h"
#include "Point2di.h"
#include "Export.h"

#include <cassert>

namespace PathFinding
{
  class PFMapGrid;
  const float NON_WALKABLE_COST = 999999.0f;
  class PATHFINDING_EXPORT PFMapGridNode : public PFNode
  {
  public:
    Point2di Point;
    /// <summary>
    /// The cost of this node. Higher cost mean less
    /// traversable. For example a cost of 0.0f means 
    /// the node is easily traversable. A cost of 999999.0f 
    /// would mean a high cost for traversing this node (this could 
    /// also mean that there is a vertical wall or that the node
    /// is completely not walkable).
    /// </summary>
    float Cost;

    virtual bool operator==(const PFNode& other) const
    {
      const PFMapGridNode* b = dynamic_cast<const PFMapGridNode*>(&other);
      if (b == 0)
        return false;

      return b->Point.x == Point.x && b->Point.y == Point.y;
    }

    inline float GetHeuristicCost(const PFNode& other) const
    {
      const PFMapGridNode* b = dynamic_cast<const PFMapGridNode*>(&other);
      assert(b);
      return ManhattanDistance(Point, b->Point);
    }

    inline float GetNodeTraversalCost(const PFNode& other) const
    {
      const PFMapGridNode* b = dynamic_cast<const PFMapGridNode*>(&other);
      assert(b);
      float d = EuclideanDistance(Point, b->Point);
      return d + Cost;
    }

    inline float EuclideanDistance(const Point2di& a, const Point2di& b) const
    {
      return sqrtf(
        static_cast<float>((a.x - b.x) * (a.x - b.x)) +
        static_cast<float>((a.y - b.y) * (a.y - b.y))
      );
    }

    inline float ManhattanDistance(const Point2di& a, const Point2di& b) const
    {
      return abs(a.x - b.x) + abs(a.y - b.y);
    }
    // Is this cell walkable?
    inline bool GetIsWalkable() const
    {
      return Cost < NON_WALKABLE_COST;
    }

    // construct the node with the grid and the location.
    PFMapGridNode(PFMapGrid& gridMap, Point2di value)
      : PFNode()
      , mGridMap(gridMap)
      , Point(value)
      , Cost(0.0f)
    {
    }

    std::vector<const PFNode*> GetNeighbours() const;

  private:
    PFMapGrid& mGridMap;
  };
}