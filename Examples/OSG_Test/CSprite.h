#pragma once

#include <osg/Referenced>
#include <osg/Image>
#include <osg/Texture2D>
#include <osg/Geode>

namespace Faramira
{
  class CSprite : public osg::Referenced
  {
  public:
    CSprite();
    void SetImage(osg::Image* image);
    osg::Geode* GetNode()
    {
      return _quadGeode.get();
    }

  protected:
    virtual ~CSprite();
    void CreateQuad(float l, float b, float r, float t);

  private:
    osg::ref_ptr<osg::Geometry> _quad;
    osg::ref_ptr<osg::Geode> _quadGeode;
    osg::ref_ptr<osg::Texture2D> _texture2d;
  };
}
