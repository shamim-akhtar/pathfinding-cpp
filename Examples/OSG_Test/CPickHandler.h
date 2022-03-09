#pragma once
#include "PathFinder.h"
#include "PFMapGrid.h"
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>

namespace Faramira
{
  class CGridMapOSG;
  class CNPC;

  class CPickHandler : public osgGA::GUIEventHandler
  {
  public:
    CPickHandler(CGridMapOSG& gridMapOSG, CNPC& npc);

    void SetNodeMask(unsigned int mask)
    {
      mMask = mask;
    }

    virtual bool handle(
      const osgGA::GUIEventAdapter& ea,
      osgGA::GUIActionAdapter& aa); 

  protected:
    void Pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
    void SelectPosition(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
    virtual ~CPickHandler();
    CGridMapOSG& mGridMapOSG;
    CNPC& mNpc;
    unsigned int mMask;
  };
}