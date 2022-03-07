#include <ostream>

#include "GL/glew.h"
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/PolygonStipple>
#include <osg/TemplatePrimitiveFunctor>
#include <osg/TemplatePrimitiveIndexFunctor>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osg/Math>
#include <osgText/Font>
#include <osgText/Text>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include "PathFinder.h"
#include "PathFindingMaps.h"
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/config/SingleWindow>

#include "CSprite.h"
#include "CGridMapOSG.h"
#include "CNPC.h"

using namespace Faramira;


int main(int argc, char** argv)
{
  // use an ArgumentParser object to manage the program arguments.
  osg::ArgumentParser arguments(&argc, argv); 
  
  osgViewer::Viewer viewer;
  viewer.apply(new osgViewer::SingleWindow(100, 100, 640, 480));

  //viewer.setRealizeOperation(new GlewInitOperation);
  viewer.realize();


  //viewer.setUpViewInWindow(50, 50, 800, 600);
  // set up the camera manipulators.
  {
    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

    keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
    keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
    keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
    keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());
    keyswitchManipulator->addMatrixManipulator('5', "Orbit", new osgGA::OrbitManipulator());
    keyswitchManipulator->addMatrixManipulator('6', "FirstPerson", new osgGA::FirstPersonManipulator());
    keyswitchManipulator->addMatrixManipulator('7', "Spherical", new osgGA::SphericalManipulator());

    viewer.setCameraManipulator(keyswitchManipulator.get());
  }

  // add the state manipulator
  viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

  // add the thread model handler
  viewer.addEventHandler(new osgViewer::ThreadingHandler);

  // add the window size toggle handler
  viewer.addEventHandler(new osgViewer::WindowSizeHandler);

  // add the stats handler
  viewer.addEventHandler(new osgViewer::StatsHandler);

  // add the record camera path handler
  viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

  // add the LOD Scale handler
  viewer.addEventHandler(new osgViewer::LODScaleHandler);

  // add the screen capture handler
  viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
  //viewer.addEventHandler(new ImGuiDemo);

  osg::ElapsedTime elapsedTime;

  // load the data
  osg::ref_ptr<osg::Group> loadedModel = new osg::Group();

  // report any errors if they have occurred when parsing the program arguments.
  if (arguments.errors())
  {
    arguments.writeErrorMessages(std::cout);
    return 1;
  }

  PathFinding::GridMap* gridMap = PathFinding::GridMap::CreateRandomGridMap(10, 10);

  osg::ref_ptr<CGridMapOSG> grid = new CGridMapOSG(gridMap);
  osg::ref_ptr<CNPC> npc = new CNPC();
  npc->SetMap(grid.get());

  npc->SetFrameStamp(viewer.getFrameStamp());
  grid->SetNPC(npc.get());

  viewer.addEventHandler(grid->GetEventHandler());

  loadedModel->addChild(grid->GetNode());
  loadedModel->addChild(npc->GetNode());
  viewer.setSceneData(loadedModel);

  return viewer.run();
}