#pragma once
#include "PathFinder.h"
#include "PathFindingMaps.h"
#include "CSprite.h"
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

namespace Faramira
{
class CGridMapOSG : public osg::Referenced
{
public:
  static const unsigned int NODE_MASK = 0x0001;
  CGridMapOSG(PathFinding::GridMap* grid);

  osg::Node* GetNode()
  {
    return mNode.get();
  }

  void ToggleWalkable(osg::Node* node, PathFinding::GridMap::GridCell* cell);

  class CPickHandler : public osgGA::GUIEventHandler
  {
  public:
    CPickHandler(CGridMapOSG& gridMapOSG);

    virtual bool handle(
      const osgGA::GUIEventAdapter& ea,
      osgGA::GUIActionAdapter& aa); 

  protected:
    void Pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
    virtual ~CPickHandler();
    CGridMapOSG& mGridMapOSG;
  };

  osgGA::GUIEventHandler* GetEventHandler()
  {
    return mPickHandler.get();
  }

protected:
  virtual ~CGridMapOSG();

private:
  osg::ref_ptr<osg::Group> mNode;
  osg::ref_ptr<CPickHandler> mPickHandler;
  //osg::ref_ptr<CSprite> mSprite;
  //std::vector<osg::ref_ptr<CSprite>> mSprites;
};
}