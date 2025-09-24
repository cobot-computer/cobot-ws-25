#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <libchess/position.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "chess_player/chess_player_node.hpp"
#include "chess_player/result.hpp"

using namespace std;

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
using moveit::planning_interface::MoveGroupInterfacePtr;

//                                                                                                //
// ===================================== Utility Functions ====================================== //
//                                                                                                //

/**
 * Return the Z value of the chessboard in the planning frame.
 *
 * @param[in] chess_player The chess player node.
 * @return The Z of the chessboard.
 */
double get_chessboard_z(ChessPlayerNode& chess_player)
{
  const auto pose = [] {
    Pose p;
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 0;
    p.orientation.w = 1;

    return p;
  }();

  // Transform the pose to the planning frame.
  const Pose base_link_pose = [&] {
    while (chess_player.chessboard_transform.header.frame_id.empty()) {
      rclcpp::sleep_for(5ms);
    }

    Pose base_link_pose;
    tf2::doTransform(pose, base_link_pose, chess_player.chessboard_transform);
    return base_link_pose;
  }();

  return base_link_pose.position.z;
}

/**
 * Return the pose of the end effector over a square on the chessboard.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] square The square on the chessboard.
 * @param[in] dist_above_board The distance above the board to hover.
 * @return The pose of the end effector over the square, in the base_link frame.
 */
Pose get_pose_over_square(ChessPlayerNode& chess_player, const libchess::Square& square,
                          double dist_above_board)
{
  const auto params = chess_player.get_params();
  const double square_size = params.measurements.chessboard_square_size;

  // Extreme rows and columns are offset from the center by 3.5x the square size.
  const double col_zero = -square_size * 3.5;
  const double row_zero = -square_size * 3.5;

  // Pick the correct pose in the chessboard frame.
  const auto pose = [&] {
    const int row = square.rank();
    const int col = square.file();

    tf2::Quaternion q;
    q.setRPY(0, M_PI, -M_PI / 6);

    Pose p;
    p.position.x = col_zero + col * square_size;
    p.position.y = row_zero + row * square_size;
    p.position.z = dist_above_board;
    p.orientation = tf2::toMsg(q);

    return p;
  }();
  RCLCPP_INFO(chess_player.get_logger(), "Pose relative to chessboard: (%0.4f, %0.4f, %0.4f)",
              pose.position.x, pose.position.y, pose.position.z);

  // Transform the pose to the planning frame.
  const auto base_link_pose = [&] {
    while (chess_player.chessboard_transform.header.frame_id.empty()) {
      rclcpp::sleep_for(5ms);
    }

    Pose base_link_pose;
    tf2::doTransform(pose, base_link_pose, chess_player.chessboard_transform);
    return base_link_pose;
  }();
  RCLCPP_INFO(chess_player.get_logger(), "Pose relative to base link: (%0.4f, %0.4f, %0.4f)",
              base_link_pose.position.x, base_link_pose.position.y, base_link_pose.position.z);

  return base_link_pose;
}

/**
 * Move the end effector to a pose.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] pose The pose to move to.
 * @return The result of the operation.
 */
Result move_to_pose(ChessPlayerNode& chess_player, const Pose& pose)
{
  for (size_t i = 0; i < 2; ++i) {
    // Make sure the cobot isn't disabled.
    if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
      return Result::ERR_FATAL;
    }

    // Set the pose target.
    if (!chess_player.main_move_group->setPoseTarget(pose)) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to set pose target");
      return Result::ERR_RETRY;
    }

    // Plan the motion.
    const auto [success, plan] = [&] {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto ok = static_cast<bool>(chess_player.main_move_group->plan(plan));
      return make_pair(ok, plan);
    }();
    if (!success) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to plan motion");
      return Result::ERR_FATAL;
    }

    // Make sure the cobot wasn't disabled during motion planning.
    if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
      RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; discarding planned "
                                                    "motion");
      return Result::ERR_FATAL;
    }

    // Execute the motion.
    RCLCPP_INFO(chess_player.get_logger(), "EXECUTING");
    const auto execute_result = chess_player.main_move_group->execute(plan);
    switch (execute_result.val) {
      case moveit::core::MoveItErrorCode::SUCCESS:
      case moveit::core::MoveItErrorCode::TIMED_OUT:
        continue;
      default:
        RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to execute motion");
        return Result::ERR_FATAL;
    }
  }
  return Result::OK;
}

/**
 * Move the end effector to a pose using Cartesian motion. This is useful for moving the end
 * effector in a straight line to a pose.
 *
 * @param[in] chess_player The chess player node.
 * @param[in] pose The pose to move to.
 * @return The result of the operation.
 */
Result move_to_pose_cartesian(ChessPlayerNode& chess_player, const Pose& pose)
{
  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Set the waypoints. The first waypoint is the current pose, and the second waypoint is the
  // target pose.
  vector<Pose> waypoints;
  waypoints.push_back(chess_player.main_move_group->getCurrentPose().pose);
  waypoints.push_back(pose);

  // Plan the motion.
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
      chess_player.main_move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
  if (fraction < 1.0) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to plan Cartesian motion");
    return Result::ERR_FATAL;
  }

  // Make sure the cobot wasn't disabled during motion planning.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; discarding planned motion");
    return Result::ERR_FATAL;
  }

  // Execute the motion.
  const auto execute_result = chess_player.main_move_group->execute(trajectory);
  switch (execute_result.val) {
    case moveit::core::MoveItErrorCode::SUCCESS:
    case moveit::core::MoveItErrorCode::TIMED_OUT:
      return Result::OK;
    default:
      RCLCPP_ERROR(chess_player.node->get_logger(), "Failed to execute motion");
      return Result::ERR_FATAL;
  }
}

Result move_to_named_pose(ChessPlayerNode& chess_player, MoveGroupInterfacePtr move_group,
                          string target)
{
  move_group->setNamedTarget(target);
  const auto result = move_group->move();
  switch (result.val) {
    case moveit::core::MoveItErrorCode::SUCCESS:
    case moveit::core::MoveItErrorCode::TIMED_OUT:
    case moveit::core::MoveItErrorCode::CONTROL_FAILED:
      return Result::OK;
    default:
      RCLCPP_ERROR(chess_player.get_logger(), "Failed to move to named pose '%s': %s",
                   target.c_str(), moveit::core::error_code_to_string(result).c_str());
      return Result::ERR_FATAL;
  }
}

Result move_to_named_pose_async(ChessPlayerNode& chess_player, MoveGroupInterfacePtr move_group,
                                string target)
{
  move_group->setNamedTarget(target);
  const auto result = move_group->asyncMove();
  switch (result.val) {
    case moveit::core::MoveItErrorCode::SUCCESS:
    case moveit::core::MoveItErrorCode::TIMED_OUT:
    case moveit::core::MoveItErrorCode::CONTROL_FAILED:
      return Result::OK;
    default:
      RCLCPP_ERROR(chess_player.get_logger(), "Failed to move to named pose '%s': %s",
                   target.c_str(), moveit::core::error_code_to_string(result).c_str());
      return Result::ERR_FATAL;
  }
}

//                                                                                                //
// ===================================== Motion Components ====================================== //
//                                                                                                //

Result move_above_square(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  const auto target_pose = get_pose_over_square(
      chess_player, square, chess_player.get_params().measurements.hover_above_board);
  return move_to_pose(chess_player, target_pose);
}

Result align_to_piece(ChessPlayerNode& chess_player)
{
  RCLCPP_INFO(chess_player.get_logger(), "Aligning");
  this_thread::sleep_for(250ms);
  double nearest_piece_dist = numeric_limits<double>::infinity();
  const auto params = chess_player.get_params();
  chess_player.main_move_group->stop();
  double error_x = 0;
  double error_y = 0;
  double error_z = 0;
  double x_cmd = 0;
  double y_cmd = 0;
  double z_cmd = 0;
  int print_it = 0;
  do {
    // Get current pose gripper.
    const auto gripper_pose = chess_player.main_move_group->getCurrentPose().pose;
    const auto gripper_z = gripper_pose.position.z;
    const auto gripper_rot_zyx = [&] {
      tf2::Quaternion tf2_quaternion;
      tf2::fromMsg(gripper_pose.orientation, tf2_quaternion);
      const tf2::Matrix3x3 mat(tf2_quaternion);
      double y, p, r;
      mat.getEulerYPR(y, p, r);
      return make_tuple(y, p, r);
    }();

    // Wait up to 45ms to get pieces.
    rclcpp::sleep_for(20ms);
    for (int i = 0; i < 25; ++i) {
      if (!chess_player.tof_pieces().empty()) break;
      rclcpp::sleep_for(1ms);
    }
    if (chess_player.tof_pieces().empty()) {
      RCLCPP_WARN(chess_player.get_logger(), "No pieces detected; not aligning");
      break;
    }

    // Find the point nearest to the center of the TOF camera.
    const auto [dist, nearest_piece] = [&chess_player] {
      double min_dist = numeric_limits<double>::infinity();
      Point nearest_piece;
      for (auto& piece : chess_player.tof_pieces()) {
        const auto dist = piece.x * piece.x + piece.y * piece.y;
        if (dist < min_dist) {
          min_dist = dist;
          nearest_piece = piece;
        }
      }
      return make_pair(min_dist, nearest_piece);
    }();
    nearest_piece_dist = dist;

    // Servo.
    const auto twist_cmd = [&] {
      const auto x_weight = params.align_params.x_weight;
      const auto y_weight = params.align_params.y_weight;
      const auto z_weight = params.align_params.z_weight;
      const auto angular_weight = params.align_params.angular_weight;
      const auto z_min = params.measurements.hover_above_board + params.align_params.z_min;
      const auto z_max = params.measurements.hover_above_board + params.align_params.z_max;
      const auto z_scale_neutral_point = params.measurements.min_realign_dist * 10.0;
      const auto z_scale = params.align_params.z_min / z_scale_neutral_point;

      // We determine the target Z position of the gripper based linearly on the piece's XY distance
      // to the gripper's center. If the piece is far away, we raise the gripper to get a better
      // view. If the piece is near the center, we zoom in to get a more precise measurement.
      double target_z =
          z_scale * (z_scale_neutral_point - dist) + params.measurements.hover_above_board;
      if (target_z < z_min) target_z = z_min;
      if (target_z > z_max) target_z = z_max;

      error_x = nearest_piece.y;
      error_y = -nearest_piece.x;
      error_z = target_z - gripper_z;

      x_cmd = x_weight * error_x;
      if (x_cmd < -params.align_params.max_speed) x_cmd = -params.align_params.max_speed;
      if (x_cmd > params.align_params.max_speed) x_cmd = params.align_params.max_speed;
      y_cmd = -y_weight * error_y;
      if (y_cmd < -params.align_params.max_speed) y_cmd = -params.align_params.max_speed;
      if (y_cmd > params.align_params.max_speed) y_cmd = params.align_params.max_speed;
      z_cmd = -z_weight * error_z;
      if (z_cmd < -params.align_params.max_speed) z_cmd = -params.align_params.max_speed;
      if (z_cmd > params.align_params.max_speed) z_cmd = params.align_params.max_speed;

      double error_rot_x = 0.0 - get<2>(gripper_rot_zyx);
      while (error_rot_x < -M_PI) error_rot_x += M_2_PI;
      while (error_rot_x > M_PI) error_rot_x -= M_2_PI;
      double error_rot_y = M_PI - get<1>(gripper_rot_zyx);
      while (error_rot_y < -M_PI) error_rot_y += M_2_PI;
      while (error_rot_y > M_PI) error_rot_y -= M_2_PI;
      double error_rot_z = (-M_PI * 0.167) - get<0>(gripper_rot_zyx);
      while (error_rot_z < -M_PI) error_rot_z += M_2_PI;
      while (error_rot_z > M_PI) error_rot_z -= M_2_PI;

      double rot_x_cmd = angular_weight * error_rot_x;
      if (rot_x_cmd < -1) rot_x_cmd = -1;
      if (rot_x_cmd > 1) rot_x_cmd = 1;
      double rot_y_cmd = angular_weight * error_rot_y;
      if (rot_y_cmd < -1) rot_y_cmd = -1;
      if (rot_y_cmd > 1) rot_y_cmd = 1;
      double rot_z_cmd = angular_weight * error_rot_z;
      if (rot_z_cmd < -1) rot_z_cmd = -1;
      if (rot_z_cmd > 1) rot_z_cmd = 1;

      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = chess_player.node->now();
      msg.header.frame_id = "cobot0_link_5";
      msg.twist.linear.x = x_cmd;
      msg.twist.linear.y = y_cmd;
      msg.twist.linear.z = z_cmd;
      // msg.twist.angular.x = rot_x_cmd;
      // msg.twist.angular.y = rot_y_cmd;
      // msg.twist.angular.z = rot_z_cmd;

      return msg;
    }();

    if (print_it % 100 == 0)
      RCLCPP_INFO(chess_player.get_logger(), "Error %0.5f, %0.5f, %0.5f\nCMD %0.5f, %0.5f, %0.5f",
                  error_x, error_y, error_z, x_cmd, y_cmd, z_cmd);
    ++print_it;

    chess_player.servo_twist_cmd_pub->publish(twist_cmd);

  } while (nearest_piece_dist > params.measurements.min_realign_dist || abs(error_z) > 0.005);
  RCLCPP_INFO(chess_player.get_logger(), "Aligned");

  rclcpp::sleep_for(200ms);
  return Result::OK;
}

/**
 * Pick up a piece from a square that the gripper is above.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to pick up a piece from.
 * @return The result of the operation.
 */
Result pick_up_piece(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  const auto chessboard_z = get_chessboard_z(chess_player);

  // Open the gripper.
  {
    const auto result = move_to_named_pose(chess_player, chess_player.gripper_move_group, "open");
    if (result != Result::OK) return result;
  }

  // Move the gripper above the square.
  {
    const auto result = move_above_square(chess_player, square);
    if (result != Result::OK) return result;
  }

  // Align the cobot with the piece.
  {
    const auto result = align_to_piece(chess_player);
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Calculate the pose at the center of the square, on the board surface. This is used to calculate
  // relative poses later.
  const auto pose_at_square = [&] {
    auto pose = chess_player.main_move_group->getCurrentPose().pose;
    pose.position.z = chessboard_z + chess_player.get_params().measurements.hover_above_board;
    return pose;
  }();

  // Move the end effector down to pick up the piece.
  {
    const auto down_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z = chessboard_z + chess_player.get_params().measurements.min_grasp_height;
      return pose;
    }();
    // const auto result = move_to_pose_cartesian(chess_player, down_pose);
    const auto result = move_to_pose(chess_player, down_pose);
    if (result != Result::OK) return result;
  }

  // Close the gripper.
  {
    const auto result = move_to_named_pose(chess_player, chess_player.gripper_move_group, "close");
    if (result != Result::OK) return result;
  }

  // Move the end effector back up.
  {
    const auto up_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z = chessboard_z + chess_player.get_params().measurements.hover_above_board;
      return pose;
    }();
    // const auto result = move_to_pose_cartesian(chess_player, up_pose);
    const auto result = move_to_pose(chess_player, up_pose);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

/**
 * Place a held piece at a square.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @param[in] square The square to place a piece at.
 * @return The result of the operation.
 */
Result place_piece(ChessPlayerNode& chess_player, const libchess::Square& square)
{
  // Move the gripper above the square.
  {
    const auto result = move_above_square(chess_player, square);
    if (result != Result::OK) return result;
  }

  // Calculate the pose at the center of the square, on the board surface.
  const auto pose_at_square = [&] {
    auto pose = chess_player.main_move_group->getCurrentPose().pose;
    pose.position.z = get_chessboard_z(chess_player);
    return pose;
  }();

  // Move the end effector down to place the piece.
  {
    const auto down_pose = [&] {
      auto pose = pose_at_square;
      posae the end effector back up.
  {
    const auto up_pose = [&] {
      auto pose = pose_at_square;
      pose.position.z += chess_player.get_params().measurements.hover_above_board;
      return pose;
    }();
    const auto result = move_to_pose(chess_player, up_pose);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

/**
 * Deposit a captured piece somewhere off of the board.
 *
 * @param[in] chess_player The chess player node wrapper.
 * @return The result of the operation.
 */
Result deposit_captured_piece(ChessPlayerNode& chess_player)
{
  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Move the gripper to the deposit location.
  {
    const auto result = move_to_named_pose(chess_player, chess_player.main_move_group, "deposit");
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (chess_player.get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(chess_player.node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Open the gripper.
  {
    const auto result = move_to_named_pose(chess_player, chess_player.gripper_move_group, "open");
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

//                                                                                                //
// ====================================== Class Functions ======================================= //
//                                                                                                //

Result ChessPlayerNode::capture_at_(const libchess::Square& square)
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Pick up the piece.
  {
    const auto result = pick_up_piece(*this, square);
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Deposit the piece.
  {
    const auto result = deposit_captured_piece(*this);
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_piece_(const libchess::Move& move)
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Pick up the piece at the from square.
  {
    const auto result = pick_up_piece(*this, move.from());
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Place the piece at the to square.
  {
    const auto result = place_piece(*this, move.to());
    if (result != Result::OK) return result;
  }

  // TODO: Promote the piece if necessary.

  return Result::OK;
}

Result ChessPlayerNode::hit_clock_()
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Close the gripper.
  {
    const auto result = move_to_named_pose(*this, gripper_move_group, "close");
    if (result != Result::OK) return result;
  }

  // Move the gripper to the clock location.
  {
    const auto result = move_to_named_pose(*this, main_move_group, "above_clock");
    if (result != Result::OK) return result;
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Servo down in the Z direction until the clock is pressed.
  {
    while (!clock_btn_pressed) {
      const auto twist_cmd = [&] {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = "cobot0_link_5";
        msg.twist.linear.z = 0.2;
        return msg;
      }();
      servo_twist_cmd_pub->publish(twist_cmd);
      rclcpp::sleep_for(25ms);
    }
  }

  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  // Move the gripper back up.
  {
    const auto result = move_to_named_pose(*this, main_move_group, "above_clock");
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_home_()
{
  // Make sure the cobot isn't disabled.
  if (get_state() == ChessPlayerNode::State::DISABLED) {
    RCLCPP_ERROR(node->get_logger(), "Cobot was disabled; not planning motion");
    return Result::ERR_FATAL;
  }

  {
    const auto result = move_to_named_pose(*this, main_move_group, "home");
    if (result != Result::OK) return result;
  }

  return Result::OK;
}

Result ChessPlayerNode::move_out_of_way_()
{
  return move_to_named_pose(*this, main_move_group, "out_of_way");
}