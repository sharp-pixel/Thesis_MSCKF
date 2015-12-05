cmake_minimum_required(VERSION 2.8.3)
project(thesis_MCEKF)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge roscpp rospy sensor_msgs std_msgs nav_msgs geometry_msgs image_transport tf
  tf_conversions tf2_ros eigen_conversions nodelet dynamic_reconfigure message_filters
)

# find_package(catkin REQUIRED COMPONENTS
#              cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
#              image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions
#              pcl_ros nodelet dynamic_reconfigure rviz message_filters class_loader
#              genmsg stereo_msgs move_base_msgs
# )
find_package( PkgConfig )
find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
# include_directories(EIGEN3_INCLUDE_DIR)

SET(Libraries
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

SET(Includes
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${Includes} corelib/include)

add_executable(vio_mcekf
  corelib/src/CoreNode.cpp
  corelib/src/CoreApp.cpp
  corelib/src/SensorData.cpp
  corelib/src/OdometryNode.cpp
  corelib/src/Odometry.cpp
)
target_link_libraries(vio_mcekf ${Libraries})
