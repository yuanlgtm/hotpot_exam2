# Install script for directory: /media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/install/rmsimulator_listener")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener" TYPE EXECUTABLE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/rmsimulator_listener")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rmsimulator_listener/rmsimulator_listener")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rmsimulator_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rmsimulator_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener/environment" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener/environment" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_index/share/ament_index/resource_index/packages/rmsimulator_listener")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener/cmake" TYPE FILE FILES
    "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_core/rmsimulator_listenerConfig.cmake"
    "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/ament_cmake_core/rmsimulator_listenerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmsimulator_listener" TYPE FILE FILES "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/qing/NEW/RM/hotpot_exam2/1ros_and_tf2/1.3sensor_storage/rmsimulator_listener/build/rmsimulator_listener/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
