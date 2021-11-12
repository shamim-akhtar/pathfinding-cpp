#include "CGridMapOSG.h"

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

namespace
{
  struct GridCellUserData : public osg::Referenced
  {
    GridCellUserData(std::shared_ptr<PathFinding::GridMap::GridCell> cell)
      : mCell(cell)
    {

    }
    std::shared_ptr<PathFinding::GridMap::GridCell> mCell;
  };

  class CcolorVisitor : public osg::NodeVisitor 
  {
  public:
    CcolorVisitor() : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN) 
    {
      // ---------------------------------------------------------------
      //
      // Default Ctors overide the default node visitor mode so all
      // children are visited
      //
      // ---------------------------------------------------------------
      //

      // Default to a white color

      //

      m_color.set(1.0, 1.0, 1.0, 1.0);
      m_colorArrays = new osg::Vec4Array;
      m_colorArrays->push_back(m_color);
    };

    CcolorVisitor(const osg::Vec4& color) : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN) 
    {
      // -------------------------------------------------------------------
      //
      // Overloaded Ctor initialised with the Color
      // Also override the visitor to traverse all the nodes children
      //
      // -------------------------------------------------------------------
      m_color = m_color;
      m_colorArrays = new osg::Vec4Array;
      m_colorArrays->push_back(m_color);
    };

    virtual ~CcolorVisitor() {};
    virtual void apply(osg::Node& node) 
    {
      // --------------------------------------------
      //
      //  Handle traversal of osg::Node node types
      //
      // --------------------------------------------
      traverse(node);
    } // apply( osg::Node &node )
    virtual void apply(osg::Geode& geode) 
    {
      geode.setDataVariance(osg::Object::DataVariance::DYNAMIC);
      // ------------------------------------------------
      //
      //  Handle traversal of osg::Geode node types
      //
      // ------------------------------------------------
      osg::StateSet* state = NULL;

      unsigned int    vertNum = 0;
      //  
      //  We need to iterate through all the drawables check if
      //  the contain any geometry that we will need to process
      //

      unsigned int numGeoms = geode.getNumDrawables();

      for (unsigned int geodeIdx = 0; geodeIdx < numGeoms; geodeIdx++)
      {
        //
        // Use 'asGeometry' as its supposed to be faster than a dynamic_cast
        // every little saving counts
        //
        osg::Geometry* curGeom = geode.getDrawable(geodeIdx)->asGeometry();
        curGeom->dirtyDisplayList();
        
        //
        // Only process if the drawable is geometry
        //
        if (curGeom) 
        {

          osg::Vec4Array* colorArrays = dynamic_cast<osg::Vec4Array*>(curGeom->getColorArray());
          if (colorArrays) 
          {
            for (unsigned int i = 0; i < colorArrays->size(); i++) 
            {
              osg::Vec4* color = &colorArrays->operator [](i);
              //
              // could also use *color = m_color
              //

              color->set(m_color._v[0], m_color._v[1], m_color._v[2], m_color._v[3]);
            }
          }

          else 
          {
            curGeom->setColorArray(m_colorArrays.get());
            curGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
          }
        }
      }
    } // apply( osg::Geode   
    void setColor(const float r, const float g, const float b, const float a = 1.0f) 
    {
      // -------------------------------------------------------------------
      //
      // Set the color to change apply to the nodes geometry
      //
      // -------------------------------------------------------------------

      osg::Vec4* c = &m_colorArrays->operator [](0);
      m_color.set(r, g, b, a);
      *c = m_color;
    } // setColor( r,g,b,a )

    void setColor(const osg::Vec4& color) 
    {
      // -------------------------------------------------------------------
      //
      // Set the color to change apply to the nodes geometry
      //
      // -------------------------------------------------------------------
      osg::Vec4* c = &m_colorArrays->operator [](0);
      m_color = color;
      *c = m_color;
    } // setColor( vec4 )
  private:
    osg::Vec4 m_color;
    osg::ref_ptr< osg::Vec4Array > m_colorArrays;
  }; // class CcolorVisitor
}

CGridMapOSG::CGridMapOSG(PathFinding::GridMap* grid)
  : mNode(new osg::Group)
{
  for (int i = 0; i < grid->GetNumX(); ++i)
  {
    for (int j = 0; j < grid->GetNumY(); ++j)
    {
      osg::ref_ptr<osg::PositionAttitudeTransform> pat =
        new osg::PositionAttitudeTransform();
      pat->setPosition(osg::Vec3(i, j, 0.0f));
      CSprite* sprite = new CSprite();
      pat->setUserData(sprite);
      pat->addChild(sprite->GetNode());
      mNode->addChild(pat);

      std::shared_ptr<PathFinding::GridMap::GridCell> cell = grid->GetCell(i, j);
      osg::ref_ptr<GridCellUserData> cell_data = new GridCellUserData(cell);
      sprite->GetNode()->setUserData(cell_data);
    }
  }
  mNode->setNodeMask(NODE_MASK);
  mPickHandler = new CPickHandler(*this);
}

CGridMapOSG::~CGridMapOSG()
{
}

CGridMapOSG::CPickHandler::CPickHandler(CGridMapOSG& gridMapOSG)
  : osgGA::GUIEventHandler()
  , mGridMapOSG(gridMapOSG)
{
}

void CGridMapOSG::ToggleWalkable(osg::Node* node, PathFinding::GridMap::GridCell* cell)
{
  cell->SetWalkable(!cell->GetIsWalkable());
  if (cell->GetIsWalkable())
  {
    if (node)
    {
      CcolorVisitor cv;
      cv.setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
      node->accept(cv);
    }
  }
  else
  {
    if (node)
    {
      CcolorVisitor cv;
      cv.setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
      node->accept(cv);
    }
  }
}

CGridMapOSG::CPickHandler::~CPickHandler()
{
}

bool CGridMapOSG::CPickHandler::handle(
  const osgGA::GUIEventAdapter& ea, 
  osgGA::GUIActionAdapter& aa)
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

void CGridMapOSG::CPickHandler::Pick(
  osgViewer::View* view, 
  const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  if (view->computeIntersections(ea, intersections, NODE_MASK))
  {
    for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
      hitr != intersections.end();
      ++hitr)
    {
      //std::ostringstream os;
      if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
      {
        // the geodes are identified by name.
        //os << "Object \"" << hitr->nodePath.back()->getName() << "\"" << std::endl;
        osg::Node* node = hitr->nodePath.back();
        GridCellUserData* data = dynamic_cast<GridCellUserData*>(node->getUserData());
        if (data)
        {
          mGridMapOSG.ToggleWalkable(node, data->mCell.get());
        }
      }
      if (hitr->drawable.valid())
      {
        osg::Drawable* drawable = hitr->drawable;
      }
      //std::cout << os.str() << "\n";
    }
  }
}

void CGridMapOSG::CPickHandler::SelectPosition(
  osgViewer::View* view,
  const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  if (view->computeIntersections(ea, intersections, NODE_MASK))
  {
    for (osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
      hitr != intersections.end();
      ++hitr)
    {
      osg::Vec3 pos = hitr->getWorldIntersectPoint();
      mGridMapOSG.GetNPC()->AddWayPoint(pos);
    }
  }
}