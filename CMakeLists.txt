cmake_minimum_required(VERSION 3.8)
project(robotarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)  # rclpy를 추가합니다.
find_package(sensor_msgs REQUIRED)  # sensor_msgs를 추가합니다.
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB PY_SCRIPTS "src/*.py")

install(PROGRAMS
  ${PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
