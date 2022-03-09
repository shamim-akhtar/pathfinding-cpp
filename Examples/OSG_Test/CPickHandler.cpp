#include "CPickHandler.h"
#include "CGridMapOSG.h"
#include "CNPC.h"

#include <iostream>

#include <osg/PositionAttitudeTransform>
#include <osgViewer/View>
#include <iostream>
#include <sstream>
#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>

#include <osg/Vec4>

using namespace Faramira;

CPickHandler::CPickHandler(CGridMapOSG& gridMapOSG, CNPC& npc)
  : osgGA::GUIEventHandler()
  , mGridMapOSG(gridMapOSG)
  , mNpc(npc)
  , mMask(0xffffffff)
{
}

CPickHandler::~CPickHandler()
{
}

bool CPickHandler::handle(const osgGA::GUIEventAdapter& ea,  osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {
  case(osgGA::GUIEventAdapter::PUSH):
  {
    if (ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
    {
      osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
      if (view)
      {
        Pick(view, ea);
      }
    }
    if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
    {
      osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
      if (view)
      {
        SelectPosition(view, ea);
      }
    }
    return false;
  }
  default:
    return false;
  }
}

void CPickHandler::Pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  if (view->computeIntersections(ea, intersections, mMask))
  {
    for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
      hitr != intersections.end();
      ++hitr)
    {
      if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
      {
        osg::Node* node = hitr->nodePath.back();
        if (node)
        {
          mGridMapOSG.ToggleWalkable(node);
        }
      }
    }
  }
}

void CPickHandler::SelectPosition(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  if (view->computeIntersections(ea, intersections, mMask))
  {
    for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
      hitr != intersections.end();
      ++hitr)
    {
      if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
      {
        osg::Node* node = hitr->nodePath.back();
        PathFinding::PFMapGridNode* gridNode = mGridMapOSG.GetGridNodeFromNode(node);
        if (gridNode)
        {
          //osg::Vec3 pos(gridNode->Point.x, gridNode->Point.y, 0.0f);
          mNpc.MoveToGridNode(gridNode);
        }
      }
    }
  }
}