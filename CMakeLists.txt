cmake_minimum_required(VERSION 3.8)
project(robotops_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Optional: Rust message generation (requires rosidl_generator_rs from ros2_rust)
# This is used when building inside the ros2_rust Docker environment
find_package(rosidl_generator_rs QUIET)
if(rosidl_generator_rs_FOUND)
  message(STATUS "Rust message generation enabled")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TraceEvent.msg"
  "msg/TraceContextChange.msg"
  "msg/DiagnosticsReport.msg"
  "msg/StartupDiagnostics.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
