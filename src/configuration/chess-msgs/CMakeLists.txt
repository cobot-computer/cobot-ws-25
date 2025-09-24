cmake_minimum_required(VERSION 3.8)
project(chess_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FullFEN.msg"
  "msg/PartialFEN.msg"
  "msg/ChessMoveUCI.msg"
  "msg/ChessTime.msg"
  "msg/Color.msg"
  "msg/EngineInfo.msg"
  "msg/GameConfig.msg"
  "msg/CobotEnabled.msg"
  "msg/CobotSpeed.msg"
  "msg/CobotState.msg"
  "msg/Point2.msg"
  "msg/CameraPoints.msg"
  "msg/ClockButtons.msg"

  "srv/CorrectCobot.srv"
  "srv/SetGameState.srv"
  "srv/SetTime.srv"
  "srv/RestartGame.srv"
  "srv/RestartClock.srv"

  "action/FindBestMove.action"
)

ament_package()
