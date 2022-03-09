#include "CGridMapOSG.h"
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

namespace
{
  struct GridCellUserData : public osg::Referenced
  {
    GridCellUserData(PathFinding::PFMapGridNode* cell)
      : mCell(cell)
    {
    }
    PathFinding::PFMapGridNode* mCell;

  protected:
    virtual ~GridCellUserData() {}
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

CGridMapOSG::CGridMapOSG(PathFinding::PFMapGrid* grid)
  : mNode(new osg::Group)
{
  mGrid = grid;
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

      PathFinding::PFMapGridNode* cell = grid->GetMapNode(i, j);
      osg::ref_ptr<GridCellUserData> cell_data = new GridCellUserData(cell);
      sprite->GetNode()->setUserData(cell_data);
    }
  }
  mNode->setNodeMask(NODE_MASK);
}

CGridMapOSG::~CGridMapOSG()
{
}

PathFinding::PFMapGridNode* CGridMapOSG::GetGridNodeFromNode(osg::Node* node)
{
  GridCellUserData* data = dynamic_cast<GridCellUserData*>(node->getUserData());
  if (data) return data->mCell;
  return 0;
}

void CGridMapOSG::ToggleWalkable(osg::Node* node)
{
  GridCellUserData* data = dynamic_cast<GridCellUserData*>(node->getUserData());
  if (data == 0)
    return;

  PathFinding::PFMapGridNode* cell = data->mCell;
  cell->Cost = cell->GetIsWalkable() ? PathFinding::NON_WALKABLE_COST: 0.0f;
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
