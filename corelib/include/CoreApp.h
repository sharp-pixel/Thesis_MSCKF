/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef COREAPP_H_
#define COREAPP_H_

#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "SensorData.h"
#include "CameraModel.h"
#include "OdometryNode.h"
#include "Odometry.h"

using namespace std;

class CoreApp {
public:
  CoreApp();

private:
  typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyImageExactSyncPolicy;
	message_filters::Synchronizer<MyImageExactSyncPolicy> * imageExactSync_;
  typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
      sensor_msgs::Image> MyDepthApproxSyncPolicy;
	message_filters::Synchronizer<MyDepthApproxSyncPolicy> * depthApproxSync_;
  int voMode;

  image_transport::SubscriberFilter image_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> image_info_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  SensorData* previousFrame;
  SensorData* currentFrame;
  Odometry* odoms;

  void cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
  void depthCallback(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImageConstPtr& depthMsg);
  void getOdomMsg(OdometryNode* odomNode, nav_msgs::Odometry& odomMsg, geometry_msgs::TransformStamped& odomTf);
};


#endif
