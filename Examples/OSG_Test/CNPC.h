#pragma once

#include "PathFinder.h"
#include "PFMapGridNode.h"
#include <osg/PositionAttitudeTransform>
#include <deque>

namespace Faramira
{
  class CGridMapOSG;
  class CNPC : public osg::Referenced
  {
  public:
    CNPC();
    void SetMap(CGridMapOSG* gridMap);

    void SetEnablePathFinding(bool flag)
    {
      mEnablePathFinding = flag;
    }

    bool GetEnablePathFinding() const
    {
      return mEnablePathFinding;
    }

    //void MoveTo(osg::Vec3 pos);
    void MoveToGridNode(PathFinding::PFMapGridNode* pfGridNode);

    void SetFrameStamp(const osg::FrameStamp* fs)
    {
      mFrameStamp = fs;
    }
    const osg::FrameStamp* GetFrameStamp() const
    {
      return mFrameStamp;
    }

    osg::Node* GetNode()
    {
      return mNode.get();
    }

    void update();

  protected:

    class UpdateCallback : public osg::NodeCallback
    {
      CNPC& mNpc;
    public:
      UpdateCallback(CNPC& npc);
      void operator()(osg::Node* node, osg::NodeVisitor* nv);
    };

    void MoveToGridNodeWithPathFinding(PathFinding::PFMapGridNode* pfGridNode);
    virtual ~CNPC()
    {
    }
    std::deque<osg::Vec3> mWayPoints;
    osg::ref_ptr<osg::PositionAttitudeTransform> mNode;
    const osg::FrameStamp* mFrameStamp;

    PathFinding::AStarPathFinder* mPathFinder;
    osg::ref_ptr<CGridMapOSG> _map;

    //
  private:
    //for movement.
    osg::Vec3 _lastPoint;
    double _lastTime;
    osg::Vec3 _direction;
    float _speed;
    bool mEnablePathFinding;
    PathFinding::PFNode* mStart;
  };
}