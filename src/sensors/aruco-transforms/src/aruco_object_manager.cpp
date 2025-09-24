#include "aruco_transforms/aruco_object_manager.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace cv;

using geometry_msgs::msg::TransformStamped;

namespace aruco_object_manager
{

//                                                                                                //
// ========================================== Marker3d ========================================== //
//                                                                                                //

Marker3d::Marker3d(int marker_id, std::shared_ptr<CornerWrapper> location)
  : marker_id(marker_id), location(std::move(location))
{
}

// The offsets of the marker's corners from each point. Index 4 is the center point. Scale this by
// the marker size to get the actual corner points.
const array<array<Point3d, 4>, 5> CORNER_OFFSETS = {
  { // Top-left corner.
    {
      Point3d(0, 0, 0),
      Point3d(1, 0, 0),
      Point3d(1, -1, 0),
      Point3d(0, -1, 0),
    },
    // Top-right corner.
    {
      Point3d(-1, 0, 0),
      Point3d(0, 0, 0),
      Point3d(0, -1, 0),
      Point3d(-1, -1, 0),
    },
    // Bottom-right corner.
    {
      Point3d(-1, 1, 0),
      Point3d(0, 1, 0),
      Point3d(0, 0, 0),
      Point3d(-1, 0, 0),
    },
    // Bottom-left corner.
    {
      Point3d(0, 1, 0),
      Point3d(1, 1, 0),
      Point3d(1, 0, 0),
      Point3d(0, 0, 0),
    },
    // Center point.
    {
      Point3d(-0.5, 0.5, 0),
      Point3d(0.5, 0.5, 0),
      Point3d(0.5, -0.5, 0),
      Point3d(-0.5, -0.5, 0),
    } },
};

Marker3d::SinglePoint::SinglePoint(Corner corner, Point3d corner_point, double marker_size,
                                   bool extrapolate_corners)
  : corner_(corner)
  , corner_point_(corner_point)
  , marker_size_(marker_size)
  , extrapolate_corners_(extrapolate_corners)
{
}

void Marker3d::SinglePoint::emplace_points(const vector<Point2f>& img_marker_corners,
                                           vector<Point2f>& img_points,
                                           vector<Point3d>& obj_points) const
{
  const size_t corner_idx = static_cast<size_t>(corner_);
  if (extrapolate_corners_) {
    for (size_t i = 0; i < 4; ++i) {
      img_points.emplace_back(img_marker_corners[i]);
      obj_points.emplace_back(corner_point_ + CORNER_OFFSETS[corner_idx][i] * marker_size_);
    }
  } else if (corner_ != Corner::CENTER) {
    img_points.emplace_back(img_marker_corners[corner_idx]);
    obj_points.emplace_back(corner_point_);
  } else {
    Point2f img_center(0.0f, 0.0f);
    for (auto& c : img_marker_corners) {
      img_center += c * 0.25f;
    }
    img_points.emplace_back(img_center);
    obj_points.emplace_back(corner_point_);
  }
}

vector<cv::Point3f> Marker3d::SinglePoint::get_corners_for_board() const
{
  if (!extrapolate_corners_) return {};
  const size_t corner_idx = static_cast<size_t>(corner_);
  vector<Point3f> points;
  for (size_t i = 0; i < 4; ++i) {
    const auto new_point = corner_point_ + CORNER_OFFSETS[corner_idx][i] * marker_size_;
    points.emplace_back(Point3f(static_cast<float>(new_point.x), static_cast<float>(new_point.y),
                                static_cast<float>(new_point.z)));
  }
  return points;
}

Marker3d::AllCorners::AllCorners(const array<Point3d, 4>& corners) : corners_(corners) {}

void Marker3d::AllCorners::emplace_points(const vector<Point2f>& img_marker_corners,
                                          vector<Point2f>& img_points,
                                          vector<Point3d>& obj_points) const
{
  for (size_t i = 0; i < 4; ++i) {
    img_points.emplace_back(img_marker_corners[i]);
    obj_points.emplace_back(corners_[i]);
  }
}

vector<cv::Point3f> Marker3d::AllCorners::get_corners_for_board() const
{
  vector<Point3f> points;
  for (size_t i = 0; i < 4; ++i) {
    const auto new_point = corners_[i];
    if (!points.empty() && new_point.z != points.back().z) return {};
    points.emplace_back(Point3f(static_cast<float>(new_point.x), static_cast<float>(new_point.y),
                                static_cast<float>(new_point.z)));
  }
  return points;
}

//                                                                                                //
// ========================================== Marker2d ========================================== //
//                                                                                                //

Marker2d::Marker2d(int marker_id, Corner corner_id, cv::Point2f corner_point)
  : marker_id(marker_id), corner_id(corner_id), corner_point(corner_point)
{
}

//                                                                                                //
// ================================= ArucoObjectManager Public ================================== //
//                                                                                                //

ArucoObjectManager::ArucoObjectManager(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::TransformBroadcaster>& transform_broadcaster,
  image_transport::ImageTransport& image_transport, const string& tf_frame,
  const string& warped_image_topic, const Params& aruco_params, int warped_img_width,
  bool invert_transform)
  : node_(node)
  , tf_broadcaster_(transform_broadcaster)
  , tf_frame_(tf_frame)
  , warped_img_width_(warped_img_width)
  , invert_transform_(invert_transform)
  , pnp_method_(aruco_params.method)
  , markers_(aruco_params.markers)
  , min_markers_(aruco_params.min_markers)
  , object_corners_2d_(aruco_params.object_corners_2d)
  , enable_board_(aruco_params.enable_board)
  , pose_pub_(nullptr)
  , pose_topic_("")
{
  // Make the aruco board;
  if (enable_board_) make_board_();

  // The rest of this function deals with the 2D perspective warp. If we don't have points for that,
  // we return.
  if (aruco_params.object_corners_2d.size() != 4) {
    warped_img_pub_ = nullptr;
    return;
  }

  // Calculate the bounding box of the object in the warped image.
  auto [warp_top_left, warp_btm_right] = [&aruco_params]() -> pair<Point2f, Point2f> {
    const auto corners = aruco_params.object_corners_2d;
    Point2f tl(corners[0].corner_point);
    Point2f br(corners[0].corner_point);
    for (int i = 1; i < 4; ++i) {
      const auto corner = corners[i];
      if (corner.corner_point.x < tl.x) tl.x = corner.corner_point.x;
      if (corner.corner_point.y < tl.y) tl.y = corner.corner_point.y;
      if (corner.corner_point.x > br.x) br.x = corner.corner_point.x;
      if (corner.corner_point.y > br.y) br.y = corner.corner_point.y;
    }
    return { tl, br };
  }();

  // Calculate warped image height based on aspect ratio.
  warped_img_height_ = [&] {
    const double w = warp_btm_right.x - warp_top_left.x;
    const double h = warp_btm_right.y - warp_top_left.y;
    return warped_img_width_ * h / w;
  }();

  // Scale the object corners to the size of the warped image.
  [&] {
    const double scale_x = warped_img_width_ / (warp_btm_right.x - warp_top_left.x);
    const double scale_y = warped_img_height_ / (warp_btm_right.y - warp_top_left.y);
    for (auto& corner : object_corners_2d_) {
      corner.corner_point.x = (corner.corner_point.x - warp_top_left.x) * scale_x;
      corner.corner_point.y = (corner.corner_point.y - warp_top_left.y) * scale_y;
    }
  }();

  // Setup the image publisher.
  warped_img_pub_ =
    make_unique<image_transport::Publisher>(image_transport.advertise(warped_image_topic, 1));
}

ArucoObjectManager::ArucoObjectManager(rclcpp::Node::SharedPtr node, const string& pose_topic,
                                       const Params& aruco_params)
  : node_(node)
  , pose_topic_(pose_topic)
  , pnp_method_(aruco_params.method)
  , markers_(aruco_params.markers)
  , min_markers_(aruco_params.min_markers)
  , object_corners_2d_(aruco_params.object_corners_2d)
  , enable_board_(aruco_params.enable_board)
  , tf_broadcaster_(nullptr)
  , tf_frame_("")
  , warped_img_pub_(nullptr)
  , invert_transform_(false)
{
  // Make the aruco board;
  if (enable_board_) make_board_();

  // Setup the pose publisher.
  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1);
}

void ArucoObjectManager::refine_markers(const aruco::ArucoDetector& detector,
                                        const Mat& input_image, vector<int>& marker_ids,
                                        vector<vector<Point2f>>& marker_points,
                                        vector<vector<Point2f>>& rejected, const Mat& camera_matrix,
                                        const Mat& dist_coeffs) const
{
  if (!enable_board_ || !board_) return;
  detector.refineDetectedMarkers(input_image, *board_, marker_points, marker_ids, rejected,
                                 camera_matrix, dist_coeffs);
}

void ArucoObjectManager::process(const Mat& input_image, const vector<int> marker_ids,
                                 const vector<vector<Point2f>> marker_points,
                                 const string& camera_frame, const Mat& camera_matrix,
                                 const Mat& dist_coeffs)
{
  const auto now = rclcpp::Clock().now();

  // Solve the transform between the camera and the object.
  if (tf_broadcaster_ || pose_pub_) {
    tf2::Transform transform;
    const bool tf_success = solve_transform_(marker_ids, marker_points, camera_matrix, dist_coeffs,
                                             transform, invert_transform_);
    if (tf_success) {
      if (tf_broadcaster_) {
        TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = invert_transform_ ? tf_frame_ : camera_frame;
        tf_msg.child_frame_id = invert_transform_ ? camera_frame : tf_frame_;
        tf_msg.transform = tf2::toMsg(transform);
        tf_broadcaster_->sendTransform(tf_msg);
      }

      if (pose_pub_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = camera_frame;
        pose_msg.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.position.z = transform.getOrigin().z();
        pose_msg.pose.orientation = tf2::toMsg(transform.getRotation());
        pose_pub_->publish(pose_msg);
      }
    }
  }

  // Warp the input image to isolate the object.
  if (warped_img_pub_) {
    Mat warped_image;
    const bool warp_success =
      warp_perspective_(input_image, warped_image, marker_ids, marker_points,
                        Size(warped_img_width_, warped_img_height_));
    if (warp_success) {
      std_msgs::msg::Header header;
      header.stamp = now;
      const auto warped_image_msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, warped_image).toImageMsg();
      warped_img_pub_->publish(warped_image_msg);
    }
  }
}

//                                                                                                //
// ================================= ArucoObjectManager Private ================================= //
//                                                                                                //

bool ArucoObjectManager::solve_transform_(const vector<int>& img_marker_ids,
                                          const vector<vector<Point2f>> img_marker_corners,
                                          InputArray camera_matrix, InputArray dist_coeffs,
                                          tf2::Transform& transform, bool invert) const
{
  // Find matching markers between the image and the object.
  vector<Point2f> img_points;
  vector<Point3d> obj_points;
  size_t marker_count = 0;
  for (const auto& marker : markers_) {
    auto it = find(img_marker_ids.begin(), img_marker_ids.end(), marker.marker_id);
    if (it != img_marker_ids.end()) {
      const size_t index = distance(img_marker_ids.begin(), it);
      marker.location->emplace_points(img_marker_corners[index], img_points, obj_points);
      ++marker_count;
    }
  }

  // Solve the PnP problem.
  if (img_points.size() < 4) return false;
  if (marker_count < min_markers_) return false;
  Vec3d rvec, tvec;
  if (!solvePnP(obj_points, img_points, camera_matrix, dist_coeffs, rvec, tvec, false,
                pnp_method_)) {
    RCLCPP_WARN(get_logger(), "Failed to solve the PnP problem");
    return false;
  }

  // Refine the PnP solution.
  solvePnPRefineVVS(obj_points, img_points, camera_matrix, dist_coeffs, rvec, tvec);

  // Convert the rotation vector to a rotation matrix.
  Mat rmat;
  Rodrigues(rvec, rmat);

  // Convert the rotation and translation vectors to a tf2 transform.
  tf2::Vector3 tvec_tf(tvec[0], tvec[1], tvec[2]);
  tf2::Matrix3x3 rmat_tf(rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
                         rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
                         rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));
  transform = tf2::Transform(rmat_tf, tvec_tf);
  if (invert) transform = transform.inverse();

  return true;
}

rclcpp::Logger ArucoObjectManager::get_logger() const
{
  return node_->get_logger();
}

bool ArucoObjectManager::warp_perspective_(const Mat& input, Mat& output,
                                           const vector<int>& img_marker_ids,
                                           const vector<vector<Point2f>> img_marker_corners,
                                           const Size& size, bool fallback,
                                           bool outline_on_fallback)
{
  if (object_corners_2d_.size() != 4) return false;

  // Find matching markers between the image and the object.
  vector<Point2f> img_points;
  vector<Point2f> obj_points;
  for (const auto& obj_corner : object_corners_2d_) {
    const auto aruco_corner_idx = static_cast<size_t>(obj_corner.corner_id);
    auto it = find(img_marker_ids.begin(), img_marker_ids.end(), obj_corner.marker_id);
    if (it != img_marker_ids.end()) {
      const size_t index = distance(img_marker_ids.begin(), it);
      img_points.emplace_back(img_marker_corners[index][aruco_corner_idx]);
      obj_points.emplace_back(obj_corner.corner_point);
    }
  }

  bool has_4_corners = img_points.size() == 4;
  if (!has_4_corners && (!fallback || !has_previous_warp_matrix_)) {
    return false;
  }

  // Calculate the warp matrix.
  const Mat warp_matrix = [&] {
    if (has_4_corners) {
      Mat m = getPerspectiveTransform(img_points, obj_points);
      previous_warp_matrix_ = m.clone();
      has_previous_warp_matrix_ = true;
      return m;
    } else {
      return previous_warp_matrix_;
    }
  }();

  // Warp the image.
  warpPerspective(input, output, warp_matrix, size);

  // Draw an outline on the output image if the markers are not detected.
  if (!has_4_corners && outline_on_fallback)
    cv::rectangle(output, cv::Point(0, 0), size, cv::Scalar(255, 0, 0), 16);

  return true;
}

void ArucoObjectManager::make_board_()
{
  if (!enable_board_) return;

  vector<int> ids;
  vector<vector<Point3f>> corners;
  for (const auto& marker : markers_) {
    const vector<Point3f> marker_corners = marker.location->get_corners_for_board();
    if (marker_corners.size() == 0) continue;
    if (!corners.empty() && corners.back().back().z != marker_corners.back().z) {
      RCLCPP_WARN(get_logger(),
                  "Tried making a board out of markers that are not coplanar (%d and %d)",
                  ids.back(), marker.marker_id);
      continue;
    }
    corners.emplace_back(marker_corners);
    ids.emplace_back(marker.marker_id);
  }

  board_ =
    make_shared<aruco::Board>(corners, aruco::getPredefinedDictionary(aruco::DICT_4X4_50), ids);
}

};  // namespace aruco_object_manager