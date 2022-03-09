#pragma once
#include "PathFinder.h"
#include "PFMapGrid.h"
#include "CSprite.h"
#include "CNPC.h"
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

namespace Faramira
{

class CGridMapOSG : public osg::Referenced
{
public:
  static const unsigned int NODE_MASK = 0x0001;
  CGridMapOSG(PathFinding::PFMapGrid* grid);

  osg::Node* GetNode()
  {
    return mNode.get();
  }

  PathFinding::PFMapGridNode* GetGridNodeFromNode(osg::Node* node);
  PathFinding::PFMapGrid* GetMapGrid()
  {
    return mGrid;
  }

  //PathFinding::PFMapGridNode* GetGridNodeFromPosition(const osg::Vec3& pos)
  //{
  //  return mGrid->GetCell((int)pos.x(), (int)pos.y());
  //}

  void ToggleWalkable(osg::Node* node);
protected:
  virtual ~CGridMapOSG();

private:
  osg::ref_ptr<osg::Group> mNode;
  PathFinding::PFMapGrid* mGrid;
};
}