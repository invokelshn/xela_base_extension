cmake_minimum_required(VERSION 3.8)
project(xela_taxel_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xela_server_ros2 REQUIRED)

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

set(msg_files
  "msg/XTaxelSensor.msg"
  "msg/XTaxelSensorArray.msg"
  "msg/XTaxelSensorT.msg"
  "msg/XTaxelSensorTArray.msg"
)

set(srv_files
  "srv/ToggleModule.srv"
  "srv/ListModuleFilters.srv"
  "srv/ClearModuleFilters.srv"
  "srv/ToggleGroup.srv"        # <-- 추가
  "srv/ListGroups.srv"         # <-- 추가
  "srv/ClearGroups.srv"        # <-- 추가
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs xela_server_ros2
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
