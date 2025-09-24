#include "aruco_transforms/config.hpp"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace aruco_object_manager
{

//                                                                                                //
// ========================================= Chessboard ========================================= //
//                                                                                                //

const double CHESSBOARD_SIZE = 374.0 / 1000.0;
const double CHESSBOARD_ARUCO_SIZE = 18.5 / 1000.0;
const double HALF_CB = CHESSBOARD_SIZE / 2.0;

const ArucoObjectManager::Params CHESSBOARD_PARAMS = {
  // SolvePnP method.
  SOLVEPNP_IPPE,

  // Minimum number of markers.
  4,

  // 3D object markers.
  {
    Marker3d(0, make_shared<Marker3d::SinglePoint>(
                  Corner::TOP_LEFT, Point3d(-HALF_CB, HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE, true)),
    Marker3d(1, make_shared<Marker3d::SinglePoint>(
                  Corner::TOP_RIGHT, Point3d(HALF_CB, HALF_CB, 0.0), CHESSBOARD_ARUCO_SIZE, true)),
    Marker3d(2, make_shared<Marker3d::SinglePoint>(Corner::BOTTOM_RIGHT,
                                                   Point3d(HALF_CB, -HALF_CB, 0.0),
                                                   CHESSBOARD_ARUCO_SIZE, true)),
    Marker3d(3, make_shared<Marker3d::SinglePoint>(Corner::BOTTOM_LEFT,
                                                   Point3d(-HALF_CB, -HALF_CB, 0.0),
                                                   CHESSBOARD_ARUCO_SIZE, true)),
  },

  // Whether or not to construct a Board.
  true,

  // 2D Warped corners.
  {
    Marker2d(0, Corner::TOP_LEFT, Point2f(0, 0)),
    Marker2d(1, Corner::TOP_RIGHT, Point2f(CHESSBOARD_SIZE, 0)),
    Marker2d(2, Corner::BOTTOM_RIGHT, Point2f(CHESSBOARD_SIZE, CHESSBOARD_SIZE)),
    Marker2d(3, Corner::BOTTOM_LEFT, Point2f(0, CHESSBOARD_SIZE)),
  },
};

//                                                                                                //
// =========================================== Table ============================================ //
//                                                                                                //

const double TABLE_ARUCO_SIZE = 30.0 / 1000.0;
const double TABLE_WIDTH = 1171.575 / 1000.0;
const double TABLE_HEIGHT = 682.625 / 1000.0;
const double TABLE_ARUCO_OFFSET = 2.5 / 1000.0;
const double TABLE_OUTER_ARUCO_X = (TABLE_WIDTH / 2) - TABLE_ARUCO_OFFSET - (TABLE_ARUCO_SIZE / 2);
const double TABLE_INNER_ARUCO_X = TABLE_OUTER_ARUCO_X - (400.0 / 1000.0);
const double TABLE_ARUCO_Y = (TABLE_HEIGHT / 2) - TABLE_ARUCO_OFFSET - (TABLE_ARUCO_SIZE / 2);

const ArucoObjectManager::Params TABLE_PARAMS = {
  // SolvePnP method.
  SOLVEPNP_IPPE,

  // Minimum number of markers.
  4,

  // 3D object markers.
  {
    Marker3d(
      4, make_shared<Marker3d::SinglePoint>(
           Corner::CENTER, Point3d(-TABLE_OUTER_ARUCO_X, TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
    Marker3d(
      5, make_shared<Marker3d::SinglePoint>(
           Corner::CENTER, Point3d(-TABLE_INNER_ARUCO_X, TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
    Marker3d(6, make_shared<Marker3d::SinglePoint>(Corner::CENTER,
                                                   Point3d(TABLE_INNER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
                                                   TABLE_ARUCO_SIZE)),
    Marker3d(7, make_shared<Marker3d::SinglePoint>(Corner::CENTER,
                                                   Point3d(TABLE_OUTER_ARUCO_X, TABLE_ARUCO_Y, 0.0),
                                                   TABLE_ARUCO_SIZE)),
    Marker3d(
      8, make_shared<Marker3d::SinglePoint>(
           Corner::CENTER, Point3d(TABLE_OUTER_ARUCO_X, -TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
    Marker3d(
      9, make_shared<Marker3d::SinglePoint>(
           Corner::CENTER, Point3d(TABLE_INNER_ARUCO_X, -TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
    Marker3d(
      10, make_shared<Marker3d::SinglePoint>(
            Corner::CENTER, Point3d(-TABLE_INNER_ARUCO_X, -TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
    Marker3d(
      11, make_shared<Marker3d::SinglePoint>(
            Corner::CENTER, Point3d(-TABLE_OUTER_ARUCO_X, -TABLE_ARUCO_Y, 0.0), TABLE_ARUCO_SIZE)),
  },

  // Whether or not to construct a Board.
  true,

  // 2D Warped corners.
  {
    Marker2d(4, Corner::TOP_LEFT, Point2f(0, 0)),
    Marker2d(7, Corner::TOP_RIGHT, Point2f(TABLE_WIDTH, 0)),
    Marker2d(8, Corner::BOTTOM_RIGHT, Point2f(TABLE_WIDTH, TABLE_HEIGHT)),
    Marker2d(11, Corner::BOTTOM_LEFT, Point2f(0, TABLE_HEIGHT)),
  },
};

}  // namespace aruco_object_manager