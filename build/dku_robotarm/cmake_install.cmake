# Install script for directory: /home/retta/ros2_ws/src/dku_robotarm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/retta/ros2_ws/src/dku_robotarm/install/dku_robotarm")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dku_robotarm" TYPE PROGRAM PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ FILES
    "/home/retta/ros2_ws/src/dku_robotarm/src/Interface.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/Interface_test.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/Interface_w.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/camera/Camera.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/config.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/control/AngleSetter.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/control/ApplicationRobotarm.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/display/JointStatePublisher.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/display/MarkPublisher.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/display/MarkerArrayPublisher.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dku_robotarm/gpt_api" TYPE PROGRAM FILES
    "/home/retta/ros2_ws/src/dku_robotarm/src/gpt_api/ChatHandler.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/gpt_api/DataPreprocessing.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/gpt_api/RobotAction.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/gpt_api/Transfersound.py"
    "/home/retta/ros2_ws/src/dku_robotarm/src/gpt_api/__init__.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/" TYPE DIRECTORY FILES "/home/retta/ros2_ws/src/dku_robotarm/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/" TYPE DIRECTORY FILES "/home/retta/ros2_ws/src/dku_robotarm/meshes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/" TYPE DIRECTORY FILES "/home/retta/ros2_ws/src/dku_robotarm/rviz")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/" TYPE DIRECTORY FILES "/home/retta/ros2_ws/src/dku_robotarm/urdf")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dku_robotarm")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dku_robotarm")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/environment" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/environment" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_index/share/ament_index/resource_index/packages/dku_robotarm")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm/cmake" TYPE FILE FILES
    "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_core/dku_robotarmConfig.cmake"
    "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/ament_cmake_core/dku_robotarmConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dku_robotarm" TYPE FILE FILES "/home/retta/ros2_ws/src/dku_robotarm/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
  file(WRITE "/home/retta/ros2_ws/src/dku_robotarm/build/dku_robotarm/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
