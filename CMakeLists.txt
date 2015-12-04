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

find_package(OpenCV REQUIRED)

SET(Libraries
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

include_directories(corelib/include)

add_executable(vio_mcekf corelib/src/CoreNode.cpp corelib/src/CoreApp.cpp corelib/src/ImageData.cpp)
target_link_libraries(vio_mcekf ${Libraries})
