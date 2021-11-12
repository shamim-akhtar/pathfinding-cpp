#include "CSprite.h"
#include <osg/Geode>
#include <osg/Geometry>

using namespace Faramira;

CSprite::CSprite()
  : osg::Referenced()
{
  _quadGeode = new osg::Geode();
  _texture2d = new osg::Texture2D();
  CreateQuad(0.0f, 0.0f, 1.0f, 1.0f);
}

CSprite::~CSprite()
{

}

void CSprite::SetImage(osg::Image* image)
{
  _texture2d->setImage(image);
}

void CSprite::CreateQuad(float l, float b, float r, float t)
{
  osg::Geode* geode = _quadGeode.get();
  geode->setName("CSprite - Geode");

  // create Geometry object to store all the vertices and lines primitive.
  osg::Geometry* polyGeom = new osg::Geometry();
  polyGeom->setName("Sprite - Geom");
  _quad = polyGeom;

  osg::ref_ptr<osg::Vec4Array> shared_colors = new osg::Vec4Array;
  shared_colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));

  // Same trick for shared normal.
  osg::ref_ptr<osg::Vec3Array> shared_normals = new osg::Vec3Array;
  shared_normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));

  // note, anticlockwise ordering.
  osg::Vec3 myCoords[] =
  {
      osg::Vec3(-0.5f, -0.5f, 0.0f),
      osg::Vec3(0.5f, -0.5f, 0.0f),
      osg::Vec3(0.5f,  0.5f, 0.0f),
      osg::Vec3(-0.5f,  0.5f, 0.0f),
  };

  int numCoords = sizeof(myCoords) / sizeof(osg::Vec3);

  osg::Vec3Array* vertices = new osg::Vec3Array(numCoords, myCoords);

  // pass the created vertex array to the points geometry object.
  polyGeom->setVertexArray(vertices);

  osg::Vec2Array* tcoords = new osg::Vec2Array(4);
  (*tcoords)[0].set(l, t);
  (*tcoords)[1].set(l, b);
  (*tcoords)[2].set(r, b);
  (*tcoords)[3].set(r, t);
  polyGeom->setTexCoordArray(0, tcoords);

  // use the shared color array.
  polyGeom->setColorArray(shared_colors.get(), osg::Array::BIND_OVERALL);


  // use the shared normal array.
  polyGeom->setNormalArray(shared_normals.get(), osg::Array::BIND_OVERALL);


  // This time we simply use primitive, and hardwire the number of coords to use
  // since we know up front,
  polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, numCoords));

  polyGeom->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture2d.get(), osg::StateAttribute::ON);

  // add the points geometry to the geode.
  geode->addDrawable(polyGeom);
}