#ifndef ARUCO_TRANSFORMS__ARUCO_OBJECT_MANAGER_HPP_
#define ARUCO_TRANSFORMS__ARUCO_OBJECT_MANAGER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <aruco_transforms_params.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <vector>

namespace aruco_object_manager
{

/**
 * The corner of an object, numbered following the convention of aruco markers.
 */
enum class Corner { TOP_LEFT = 0, TOP_RIGHT = 1, BOTTOM_RIGHT = 2, BOTTOM_LEFT = 3, CENTER = 4 };

/**
 * The location of a marker in 3D space. This can be specified by:
 *
 *  - A single point, either in the center or on a corner of the marker.
 *  - All four corners of the marker, extrapolated from a single point.
 *  - All four corners of the marker, directly specified.
 *
 * If corners are being extrapolated, they will be assumed to all lie on the XY place (Z=0).
 */
class Marker3d
{
public:
  /**
   * Pure abstract class the wrap the corners of the marker.
   */
  class CornerWrapper
  {
  public:
    /**
     * Match image and object points, emplacing them into the provided vectors.
     *
     * @param[in] img_marker_corners The corners of the marker in the image.
     * @param[out] img_points Destination for the image points.
     * @param[out] obj_points Destination for the object points.
     */
    virtual void emplace_points(const std::vector<cv::Point2f>& img_marker_corners,
                                std::vector<cv::Point2f>& img_points,
                                std::vector<cv::Point3d>& obj_points) const = 0;

    /**
     * Get a vector or corners to add to the aruco board. Returns an empty vector if invalid.
     *
     * @return The corners, or an empty vector.
     */
    virtual std::vector<cv::Point3f> get_corners_for_board() const = 0;
  };
  /**
   * The location of the marker, defined by a single point.
   */
  class SinglePoint : public CornerWrapper
  {
  public:
    /**
     * Construct a new Single Point object.
     *
     * @param[in] corner The corner of the marker that the point is located at.
     * @param[in] corner_point The corner point of the marker.
     * @param[in] marker_size The length of one side of the marker, in meters.
     * @param[in] extrapolate_corners Whether to extrapolate the corners from the center point.
     */
    SinglePoint(Corner corner, cv::Point3d corner_point, double marker_size,
                bool extrapolate_corners = false);

    void emplace_points(const std::vector<cv::Point2f>& img_marker_corners,
                        std::vector<cv::Point2f>& img_points,
                        std::vector<cv::Point3d>& obj_points) const override;

    std::vector<cv::Point3f> get_corners_for_board() const override;

  private:
    Corner corner_;
    cv::Point3d corner_point_;
    double marker_size_;
    bool extrapolate_corners_;
  };

  /**
   * The location of the marker, defined by all four corners.
   */
  class AllCorners : public CornerWrapper
  {
  public:
    /**
     * Construct a new All Corners object.
     *
     * @param[in] corners The corners of the marker.
     */
    AllCorners(const std::array<cv::Point3d, 4>& corners);

    void emplace_points(const std::vector<cv::Point2f>& img_marker_corners,
                        std::vector<cv::Point2f>& img_points,
                        std::vector<cv::Point3d>& obj_points) const override;

    std::vector<cv::Point3f> get_corners_for_board() const override;

  private:
    std::array<cv::Point3d, 4> corners_;
  };

  /**
   * Construct a new Marker 3d object.
   *
   * @param[in] marker_id The ID of the marker.
   * @param[in] location The location of the marker in 3D space.
   */
  Marker3d(int marker_id, std::shared_ptr<CornerWrapper> location);

  // The dictionary ID of the marker.
  int marker_id;

  // The location of the marker in 3D space.
  std::shared_ptr<CornerWrapper> location;
};

/**
 * A point on a marker in 2D space. This is used to warp the image to a top-down view.
 */
class Marker2d
{
public:
  /**
   * Construct a new Marker 2d object.
   *
   * @param[in] marker_id The ID of the marker.
   * @param[in] corner_id The corner of the marker that the point is located at.
   * @param[in] corner_point The corner point of the marker.
   */
  Marker2d(int marker_id, Corner corner_id, cv::Point2f corner_point);

  // The ID of the marker.
  int marker_id;

  // The corner of the marker that the point is located at.
  Corner corner_id;

  // The corner point of the marker.
  cv::Point2f corner_point;
};

class ArucoObjectManager
{
public:
  /**
   * Parameters used to define an object using Aruco markers.
   */
  struct Params {
    // The method used to solve the PnP problem.
    cv::SolvePnPMethod method;

    // The minimum number of markers required to solve the transform.
    size_t min_markers;

    // The markers that define the object.
    std::vector<Marker3d> markers;

    // Whether or not to construct a Board. All markers MUST be on a plane if true.
    bool enable_board;

    // The 4 corners of the object in object space, used to warp the image. Leave empty to disable.
    std::vector<Marker2d> object_corners_2d;
  };

  /**
   * Construct a new Aruco Object Manager object that publishes a TF2 transform and a warped image.
   *
   * @param[in] node The node to attach to.
   * @param[in] transform_broadcaster The TF broadcaster to use for publishing transforms.
   * @param[in] image_transport The image transport object to use for publishing images.
   * @param[in] tf_frame The TF frame to use for the object.
   * @param[in] warped_image_topic The topic to publish the warped image to.
   * @param[in] aruco_params The parameters for the Aruco object.
   * @param[in] warped_img_width The width, in pixels, of the warped image to publish.
   * @param[in] invert_transform If true, the transform is inverted.
   */
  explicit ArucoObjectManager(rclcpp::Node::SharedPtr node,
                              std::shared_ptr<tf2_ros::TransformBroadcaster>& transform_broadcaster,
                              image_transport::ImageTransport& image_transport,
                              const std::string& tf_frame, const std::string& warped_image_topic,
                              const Params& aruco_params, int warped_img_width,
                              bool invert_transform = false);

  /**
   * Construct a new Aruco Object Manager object that publishes a PoseStamped message relative to
   * the camera.
   *
   * @param[in] node The node to attach to.
   * @param[in] pose_topic The topic to publish the pose to.
   * @param[in] aruco_params The parameters for the Aruco object.
   */
  explicit ArucoObjectManager(rclcpp::Node::SharedPtr node, const std::string& pose_topic,
                              const Params& aruco_params);

  /**
   * Use our aruco board to refine our markers and try to find missing ones.
   *
   * @param[in] detector An Aruco detector.
   * @param[in] input_image The input image to search in.
   * @param[out] marker_ids The IDs of the markers that have been detected.
   * @param[out] marker_points The corners of the markers that have been detected.
   * @param[in] rejected Rejected marker corners.
   * @param[in] camera_matrix The camera matrix.
   * @param[in] dist_coeffs The distortion coefficients of the camera.
   */
  void refine_markers(const cv::aruco::ArucoDetector& detector, const cv::Mat& input_image,
                      std::vector<int>& marker_ids,
                      std::vector<std::vector<cv::Point2f>>& marker_points,
                      std::vector<std::vector<cv::Point2f>>& rejected, const cv::Mat& camera_matrix,
                      const cv::Mat& dist_coeffs) const;

  /**
   * Process a new image. This will solve the transform between the camera and the object, and warp
   * the image to isolate the object. The warped image will be published to the topic specified in
   * the constructor, and the transform will be broadcasted.
   *
   * @param[in] input_image The input image to process.
   * @param[in] marker_ids The IDs of the markers that were detected.
   * @param[in] marker_points The corners of the markers that were detected.
   * @param[in] camera_frame The name of the camera's TF frame.
   * @param[in] camera_matrix The camera matrix.
   * @param[in] dist_coeffs The distortion coefficients of the camera.
   * @return The transform between the camera and the object.
   */
  void process(const cv::Mat& input_image, const std::vector<int> marker_ids,
               const std::vector<std::vector<cv::Point2f>> marker_points,
               const std::string& camera_frame, const cv::Mat& camera_matrix,
               const cv::Mat& dist_coeffs);

  /**
   * Get the node's logger.
   *
   * @return The node's logger.
   */
  rclcpp::Logger get_logger() const;

private:
  /**
   * Solve the transform between the camera and the object.
   *
   * @param[in] img_marker_ids The IDs of the markers that were detected.
   * @param[in] img_marker_corners The corners of the markers that were detected.
   * @param[in] camera_matrix The camera matrix.
   * @param[in] dist_coeffs The distortion coefficients of the camera.
   * @param[out] transform The transform between the camera and the object.
   * @param[in] invert If true, the transform is inverted.
   * @return True if the transform was solved, false otherwise.
   */
  bool solve_transform_(const std::vector<int>& img_marker_ids,
                        const std::vector<std::vector<cv::Point2f>> img_marker_corners,
                        cv::InputArray camera_matrix, cv::InputArray dist_coeffs,
                        tf2::Transform& transform, bool invert = true) const;

  /**
   * Warp the input image to isolate the object.
   *
   * @param[in] input The input image to warp.
   * @param[out] output The output image.
   * @param[in] img_marker_ids The IDs of the markers that were detected.
   * @param[in] img_marker_corners The corners of the markers that were detected.
   * @param[in] size The size of the output image.
   * @param[in] fallback If true, use the previous warp matrix if markers are not detected.
   * @param[in] outline_on_fallback If true, draw a red outline on the output image if markers are
   *                                not detected.
   * @return
   */
  bool warp_perspective_(const cv::Mat& input, cv::Mat& output,
                         const std::vector<int>& img_marker_ids,
                         const std::vector<std::vector<cv::Point2f>> img_marker_corners,
                         const cv::Size& size, bool fallback = true,
                         bool outline_on_fallback = true);

  /**
   * Creates a new aruco board based on the current config.
   */
  void make_board_();

  rclcpp::Node::SharedPtr node_;
  std::string tf_frame_;
  std::string pose_topic_;
  int warped_img_width_;
  int warped_img_height_;
  bool invert_transform_;

  cv::SolvePnPMethod pnp_method_;
  size_t min_markers_;
  std::vector<Marker3d> markers_;
  std::vector<Marker2d> object_corners_2d_;

  bool enable_board_;
  std::shared_ptr<cv::aruco::Board> board_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<image_transport::Publisher> warped_img_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // The most recent warp matrix. Used when markers are not detected.
  bool has_previous_warp_matrix_ = false;
  cv::Mat previous_warp_matrix_;
};

};  // namespace aruco_object_manager

#endif