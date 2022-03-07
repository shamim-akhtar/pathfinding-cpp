#pragma once

#include <osg/PositionAttitudeTransform>
#include "PathFinder.h"
#include "PathFindingMaps.h"
#include <deque>

namespace Faramira
{
  class CGridMapOSG;
  class CNPC : public osg::Referenced
  {
  public:
    CNPC();
    void SetMap(CGridMapOSG* gridMap);

    void MoveTo(osg::Vec3 pos);
    void MoveToWithPathFinding(osg::Vec3 pos);

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

    class UpdateCallback : public osg::NodeCallback
    {
      CNPC& mNpc;
    public:
      UpdateCallback(CNPC& npc);
      void operator()(osg::Node* node, osg::NodeVisitor* nv);
    };

  protected:
    virtual ~CNPC()
    {
    }
    std::deque<osg::Vec3> mWayPoints;
    osg::ref_ptr<osg::PositionAttitudeTransform> mNode;
    osg::ref_ptr<osg::AnimationPath> mAnimationPath;
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
  };
}