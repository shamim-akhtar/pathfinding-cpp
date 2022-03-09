#include "CNPC.h"
#include "CGridMapOSG.h"

#include <osg/Timer>
#include <iostream>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>

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
	, mEnablePathFinding(true)
	, mStart(0)
{
	mPathFinder = new PathFinding::AStarPathFinder();
  mNode = new osg::PositionAttitudeTransform();

	// create a simple shape.
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.2f), 0.2f)));
	mNode->addChild(geode);

	mNode->setUpdateCallback(new UpdateCallback(*this));
	_speed = 1.0f;
	_lastPoint.set(0.0f, 0.0f, 0.0f);
	_direction.set(0.0f, 0.0f, 0.0f);
	_lastTime = 0.0f;
}

void CNPC::MoveToGridNode(PathFinding::PFMapGridNode* pfGridNode)
{
	if (mEnablePathFinding)
	{
		MoveToGridNodeWithPathFinding(pfGridNode);
	}
	else
	{
		mWayPoints.push_back(osg::Vec3(pfGridNode->Point.x, pfGridNode->Point.y, 0.0f));
	}
}

void CNPC::MoveToGridNodeWithPathFinding(PathFinding::PFMapGridNode* pfGridNode)
{
	if (mStart == 0) return;

	// 
	if (mPathFinder->GetStatus() != PathFinding::PathFinderStatus::RUNNING)
	{
		PathFinding::PFNode* start = mStart;
		PathFinding::PFMapGridNode* goal = pfGridNode;

		mPathFinder->Initialize(start, goal);
		while (mPathFinder->GetStatus() == PathFinding::PathFinderStatus::RUNNING)
		{
			mPathFinder->Step();
		}
		if (mPathFinder->GetStatus() == PathFinding::PathFinderStatus::SUCCESS)
		{
			std::vector<PathFinding::PFNode*> path = mPathFinder->GetReversePath();

			for (int i = path.size() - 1; i >= 0; i--)
			{
				PathFinding::PFMapGridNode* node = dynamic_cast<PathFinding::PFMapGridNode*>(path[i]);
				assert(node);
				mWayPoints.push_back(osg::Vec3(node->Point.x, node->Point.y, 0.0f));
			}
			// set the goal to be the new start.
			mStart = goal;
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
	mStart = _map->GetMapGrid()->GetCell(0, 0);
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