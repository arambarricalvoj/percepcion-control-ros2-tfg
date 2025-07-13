cmake_minimum_required(VERSION 3.8)
project(bilbotiks_interfazeak)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Imu.msg"
  "msg/MotorrakMugitu.msg"
  "msg/ServoakMugitu.msg"
  "msg/Aurpegia.msg"
  "msg/Pertzepzioa.msg"
  "msg/Kontrola.msg"
  "action/Bira360.action"
  "srv/KoloreaZenbakia.srv"
  "srv/Argazkia.srv"
  DEPENDENCIES geometry_msgs std_msgs unique_identifier_msgs action_msgs sensor_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
