#include "CNPC.h"
#include <osg/Timer>
#include <iostream>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include "CGridMapOSG.h"

using namespace Faramira;

CNPC::UpdateCallback::UpdateCallback(CNPC& npc)
	: mNpc(npc)
{

}
void CNPC::UpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	//double simTime = nv->getFrameStamp()->getSimulationTime();
	//std::cout << simTime << "\n";
	//mNpc.Update(simTime - mLastSimTime);
	//mLastSimTime = simTime;
	//traverse(node, nv);
	mNpc.update();
	traverse(node, nv);
}

CNPC::CNPC()
  : osg::Referenced()
	, mAnimationPath(new osg::AnimationPath())
{
	mPathFinder = new PathFinding::AStarPathFinder();
	mPathFinder->SetHeuristicCost(new PathFinding::GridMap::ManhattanCost);
	mPathFinder->SetNodeTraversalCost(new PathFinding::GridMap::EuclideancCost);
  mNode = new osg::PositionAttitudeTransform();
	//mNode->setUpdateCallback(new UpdateCallback(*this));

	// create a simple shape.
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.2f), 0.2f)));
	mNode->addChild(geode);

	mAnimationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
	//mNode->setUpdateCallback(new osg::AnimationPathCallback(mAnimationPath.get()));
	mNode->setUpdateCallback(new UpdateCallback(*this));
	_speed = 1.0f;
	_lastPoint.set(0.0f, 0.0f, 0.0f);
	_direction.set(0.0f, 0.0f, 0.0f);
	_lastTime = 0.0f;
}

void CNPC::MoveTo(osg::Vec3 pos)
{
	MoveToWithPathFinding(pos);
	//mWayPoints.push_back(pos);
}

void CNPC::MoveToWithPathFinding(osg::Vec3 pos)
{
	osg::Vec3 currPos = mNode->getPosition();
	if (mWayPoints.size() > 0)
	{
		currPos = mWayPoints.front();
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

	// 
	if (mPathFinder->GetStatus() != PathFinding::PathFinderStatus::RUNNING)
	{
		PathFinding::GridMap::GridCell* start = _map->GetCell(currPos);
		PathFinding::GridMap::GridCell* goal = _map->GetCell(pos);

		mPathFinder->Initialize(start, goal);
		while (mPathFinder->GetStatus() == PathFinding::PathFinderStatus::RUNNING)
		{
			mPathFinder->Step();
		}
		if (mPathFinder->GetStatus() == PathFinding::PathFinderStatus::SUCCESS)
		{
			std::vector<osg::ref_ptr<PathFinding::GridMap::GridCell>> nodes;
			PathFinding::PathFinderNode* currNode = mPathFinder->GetCurrentNode();
			while (currNode != 0)
			{
				nodes.push_back(dynamic_cast<PathFinding::GridMap::GridCell*>(currNode->Location.get()));
				currNode = currNode->Parent;
			}

			for (int i = nodes.size() - 1; i >= 0; i--)
			{
				mWayPoints.push_back(osg::Vec3(nodes[i]->Value.x, nodes[i]->Value.y, 0.0f));
				std::cout << "x: " << nodes[i]->Value.x << ", y: " << nodes[i]->Value.y << "\n";
			}
		}
		if (mPathFinder->GetStatus() == PathFinding::PathFinderStatus::FAILURE)
		{
			std::cout << "Cannot find path .. \n";
		}
	}
}

void CNPC::SetMap(CGridMapOSG* gridMap)
{
	_map = gridMap;
}

void CNPC::update()
{
	float dt = GetFrameStamp()->getReferenceTime() - _lastTime;
	_lastTime = GetFrameStamp()->getReferenceTime();
	if (mWayPoints.empty()) return;

	osg::Vec3 nextPoint = mWayPoints.front();
	if ((nextPoint - _lastPoint).length() < 0.01f)
	{
		mWayPoints.pop_front();
		if (mWayPoints.empty()) return;
		nextPoint = mWayPoints.front();
	}

	// interpolate between last point to the next point.
	// new direction.
	_direction = nextPoint - _lastPoint;
	_direction.normalize();

	osg::Vec3 pos = _lastPoint + _direction * (dt * _speed);
	mNode->setPosition(pos);

	_lastPoint = pos;
}