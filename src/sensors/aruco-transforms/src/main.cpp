#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "aruco_transforms/aruco_object_manager.hpp"
#include "aruco_transforms/config.hpp"

using namespace aruco_object_manager;
using namespace aruco_transforms_params;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

// List of object managers. This is populated in main.
vector<ArucoObjectManager> object_managers;

/**
 * Callback for the camera. This will be called whenever a new image is received and it will update
 * all of the object managers.
 *
 * @param[in] node ROS node, used for logging.
 * @param[in] image_msg The image message.
 * @param[in] camera_info_msg The camera info message.
 */
void camera_callback(rclcpp::Node::SharedPtr node,
                     const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("aruco_transforms");

  ParamListener param_listener(node);
  Params params = param_listener.get_params();

  image_transport::ImageTransport it(node);
  auto tf_broadcaster = make_shared<tf2_ros::TransformBroadcaster>(node);

  // Chessboard.
  object_managers.emplace_back(ArucoObjectManager(node, tf_broadcaster, it, params.chessboard.frame,
                                                  params.chessboard.warped.topic, CHESSBOARD_PARAMS,
                                                  params.chessboard.warped.size, false));

  // Table.
  object_managers.emplace_back(ArucoObjectManager(node, tf_broadcaster, it, params.table.frame,
                                                  params.table.warped.topic, TABLE_PARAMS,
                                                  params.table.warped.width, true));

  // Derive camera_info topic from base topic (same convention as subscribeCamera).
  const string& base = params.camera_base_topic;
  const auto last_slash = base.rfind('/');
  const string info_topic = (last_slash == string::npos)
    ? "camera_info"
    : base.substr(0, last_slash + 1) + "camera_info";

  // Use ApproximateTime sync instead of image_transport's ExactTime to tolerate
  // small timestamp mismatches between image and camera_info from usb_cam.
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(
    node, params.camera_base_topic);
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub(
    node, info_topic);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(bind(camera_callback, node, _1, _2));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

void camera_callback(rclcpp::Node::SharedPtr node,
                     const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg)
{
  // Convert the image to an OpenCV image.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Extract camera info into OpenCV format.
  string camera_frame = camera_info_msg->header.frame_id;
  cv::Mat camera_matrix(3, 3, CV_64F, (void*)camera_info_msg->k.data());
  cv::Mat dist_coeffs = camera_info_msg->d.empty()
    ? cv::Mat::zeros(1, 5, CV_64F)
    : cv::Mat(1, (int)camera_info_msg->d.size(), CV_64F, (void*)camera_info_msg->d.data());

  // Find the Aruco markers in the image.
  static const auto aruco_detector_params = [] {
    auto params = cv::aruco::DetectorParameters();
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params.minMarkerPerimeterRate = 0.01;  // detect smaller markers (default 0.03)
    params.adaptiveThreshWinSizeMax = 53;  // wider threshold range for varied lighting
    return params;
  }();
  static const auto aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  static const cv::aruco::ArucoDetector aruco_detector(aruco_dict, aruco_detector_params);

  vector<int> marker_ids;
  vector<vector<cv::Point2f>> marker_points, rejected;
  aruco_detector.detectMarkers(cv_ptr->image, marker_points, marker_ids, rejected);
  RCLCPP_INFO_THROTTLE(node->get_logger(), *rclcpp::Clock::make_shared(), 10000,
                       "Detected %zu markers: [%s]", marker_ids.size(),
                       [&] {
                         string s;
                         for (int id : marker_ids) s += to_string(id) + " ";
                         return s;
                       }().c_str());

  // Process each object manager.
  for (auto& manager : object_managers) {
    manager.refine_markers(aruco_detector, cv_ptr->image, marker_ids, marker_points, rejected,
                           camera_matrix, dist_coeffs);
    manager.process(cv_ptr->image, marker_ids, marker_points, camera_frame, camera_matrix,
                    dist_coeffs);
  }
}
