#include "CNPC.h"
#include <osg/Timer>
#include <iostream>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>

using namespace Faramira;

//CNPC::UpdateCallback::UpdateCallback(CNPC& npc)
//	: mNpc(npc)
//{
//
//}
//void CNPC::UpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
//{
//	double simTime = nv->getFrameStamp()->getSimulationTime();
//	std::cout << simTime << "\n";
//	mNpc.Update(simTime - mLastSimTime);
//	mLastSimTime = simTime;
//	traverse(node, nv);
//}

CNPC::CNPC()
  : osg::Referenced()
	, mAnimationPath(new osg::AnimationPath())
{
  mNode = new osg::PositionAttitudeTransform();
	//mNode->setUpdateCallback(new UpdateCallback(*this));

	// create a simple shape.
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.2f), 0.2f)));
	mNode->addChild(geode);

	mAnimationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
	mNode->setUpdateCallback(new osg::AnimationPathCallback(mAnimationPath.get()));
}

void createAnimationPath(
	osg::AnimationPath* animationPath, 
	double time,
	const osg::Vec3& currPos, 
	const osg::Vec3& nextPos, 
	float dt = 0.01f)
{
	osg::Vec3 unitVec = nextPos - currPos;
	float distance = unitVec.normalize();

	int numSamples = distance / dt;
	osg::Vec3 pos = currPos;
	for (int i = 0; i < numSamples; ++i)
	{
		osg::Vec3 position(pos);

		animationPath->insert(time, osg::AnimationPath::ControlPoint(position));
		pos += unitVec * dt;
		time += dt;
	}
}

void CNPC::AddWayPoint(osg::Vec3 pos)
{
	osg::Vec3 currPos = mNode->getPosition();
	if (mWayPoints.size() > 0)
	{
		currPos = mWayPoints.back();
	}
	double lastTime = mAnimationPath->getLastTime();
	double time = mFrameStamp->getSimulationTime();
	if (lastTime > mFrameStamp->getSimulationTime())
	{
		time = lastTime;
	}
	else
	{
		mAnimationPath->clear();
	}
	createAnimationPath(mAnimationPath.get(), time, currPos, pos);
	mWayPoints.push_back(pos);
}