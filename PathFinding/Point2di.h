#pragma once

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
}