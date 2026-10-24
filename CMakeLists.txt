cmake_minimum_required(VERSION 3.0.2)
project(fast_livo2)

# FAST-LIVO2 itself is compatible with C++14. Keep this at the oldest standard
# needed by the copied upstream implementation so an older NUC toolchain can
# build the package without requiring C++17 or C++20.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-pthread -fexceptions)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf_conversions
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP QUIET)

add_message_files(
  FILES
  StampedFloat64.msg
)
generate_messages(DEPENDENCIES std_msgs)

# The copied upstream estimator uses the legacy Sophus ABI and includes
# <sophus/se3.h>. Prefer the tested x86_64 copy shipped with this folder. On a
# different architecture, use a compatible system/locally-built Sophus and
# pass Sophus_DIR or SOPHUS_INCLUDE_DIRS/SOPHUS_LIBRARIES to CMake.
set(FAST_LIVO2_VENDOR_SOPHUS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/sophus_legacy")
if(EXISTS "${FAST_LIVO2_VENDOR_SOPHUS}/lib/libSophus.so" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(SOPHUS_INCLUDE_DIRS "${FAST_LIVO2_VENDOR_SOPHUS}/include")
  set(SOPHUS_LIBRARIES "${FAST_LIVO2_VENDOR_SOPHUS}/lib/libSophus.so")
else()
  find_package(Sophus REQUIRED)
endif()

# Keep native devel-space executables able to resolve the bundled legacy
# library. The Dockerfile also installs it in /usr/local/lib for containers.
if(EXISTS "${FAST_LIVO2_VENDOR_SOPHUS}/lib/libSophus.so")
  set(CMAKE_BUILD_RPATH "${FAST_LIVO2_VENDOR_SOPHUS}/lib")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

catkin_package(
  CATKIN_DEPENDS cv_bridge eigen_conversions geometry_msgs image_transport nav_msgs
                 pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
                 tf2_eigen tf2_geometry_msgs tf2_ros tf_conversions
                 visualization_msgs message_runtime
  DEPENDS Eigen3 PCL OpenCV Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

set(VIKIT_COMMON_SOURCES
  src/vikit_common/atan_camera.cpp
  src/vikit_common/equidistant_camera.cpp
  src/vikit_common/homography.cpp
  src/vikit_common/img_align.cpp
  src/vikit_common/math_utils.cpp
  src/vikit_common/omni_camera.cpp
  src/vikit_common/performance_monitor.cpp
  src/vikit_common/pinhole_camera.cpp
  src/vikit_common/polynomial_camera.cpp
  src/vikit_common/robust_cost.cpp
  src/vikit_common/user_input_thread.cpp
  src/vikit_common/vision.cpp
)

add_library(vikit_common ${VIKIT_COMMON_SOURCES})
target_link_libraries(vikit_common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${SOPHUS_LIBRARIES})

add_library(vikit_ros src/vikit_ros/output_helper.cpp)
add_dependencies(vikit_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(vikit_ros vikit_common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${SOPHUS_LIBRARIES})

add_library(fast_livo2_vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
target_link_libraries(fast_livo2_vio vikit_common ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${SOPHUS_LIBRARIES})

add_library(fast_livo2_lio src/voxel_map.cpp)
target_link_libraries(fast_livo2_lio ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_library(fast_livo2_preprocess src/preprocess.cpp)
target_link_libraries(fast_livo2_preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_library(fast_livo2_imu src/IMU_Processing.cpp)
target_link_libraries(fast_livo2_imu ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_library(fast_livo2_mapping src/LIVMapper.cpp)
add_dependencies(fast_livo2_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_livo2_mapping
  fast_livo2_vio
  fast_livo2_lio
  fast_livo2_preprocess
  fast_livo2_imu
  vikit_ros
  vikit_common
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${SOPHUS_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(fastlivo_mapping src/main.cpp)
add_dependencies(fastlivo_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlivo_mapping fast_livo2_mapping)


add_executable(lidar_relative_failsafe_frontend src/lidar_relative_failsafe_frontend.cpp)
target_link_libraries(lidar_relative_failsafe_frontend ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_relative_failsafe_frontend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(step3_global_acquirer src/step3_global_acquirer.cpp)
add_dependencies(step3_global_acquirer ${catkin_EXPORTED_TARGETS})
target_link_libraries(step3_global_acquirer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(failsafe_map_localizer src/failsafe_map_localizer.cpp)
add_dependencies(failsafe_map_localizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(failsafe_map_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  foreach(fast_livo2_target
          vikit_common
          vikit_ros
          fast_livo2_vio
          fast_livo2_lio
          fast_livo2_preprocess
          fast_livo2_imu
          fast_livo2_mapping)
    target_link_libraries(${fast_livo2_target} OpenMP::OpenMP_CXX)
  endforeach()
endif()

catkin_install_python(PROGRAMS
  scripts/fresh_visual_global_localizer.py
  scripts/fresh_visual_global_learned_bridge.py
  scripts/fresh_multimodal_candidate_observer.py
  scripts/fresh_safe_handoff_review.py
  scripts/fresh_zedx_camera_imu_odom.py
  scripts/deployment_sensor_health_shadow.py
  scripts/deployment_klt_continuation_shadow.py
  scripts/fresh_local_odom_fallback_bridge.py
  scripts/fresh_pose_authority.py
  scripts/fast_livo2_sensor_adapter.py
  scripts/fast_livo2_failure_injector.py
  scripts/record_failsafe_trajectory.py
  scripts/rgbd_failsafe_vo.py
  scripts/visual_place_hint.py
  scripts/fast_livo2_compatibility.py
  scripts/fast_livo2_calibration_guard.py
  scripts/fast_livo2_map_anchor.py
  scripts/fast_livo2_lanelet_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The review executable imports this pure-Python rule module both from source
# workspaces and after a catkin install. It is NOT an executable or ROS node.
install(FILES scripts/safe_handoff_rules.py scripts/zedx_rgbd_imu_odom_core.py
  scripts/deployment_sensor_health_core.py
  scripts/deployment_klt_continuation_core.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(TARGETS
  vikit_common
  vikit_ros
  fast_livo2_vio
  fast_livo2_lio
  fast_livo2_preprocess
  fast_livo2_imu
  fast_livo2_mapping
  fastlivo_mapping
  failsafe_map_localizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz msg maps tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY vendor/sophus_legacy
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vendor
)
