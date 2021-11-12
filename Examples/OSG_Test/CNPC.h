#pragma once

#include <osg/PositionAttitudeTransform>
#include <deque>

namespace Faramira
{
  class CNPC : public osg::Referenced
  {
  public:
    CNPC();

    void AddWayPoint(osg::Vec3 pos);
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

  protected:
    //friend class UpdateCallback;
    //class UpdateCallback : public osg::NodeCallback
    //{
    //  CNPC& mNpc;
    //  double mLastSimTime = 0.0;
    //public:
    //  UpdateCallback(CNPC& npc);
    //  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
    //};

    //void Update(double dt);

    virtual ~CNPC()
    {

    }
    std::deque<osg::Vec3> mWayPoints;
    osg::ref_ptr<osg::PositionAttitudeTransform> mNode;
    osg::ref_ptr<osg::AnimationPath> mAnimationPath;
    const osg::FrameStamp* mFrameStamp;
  };
}