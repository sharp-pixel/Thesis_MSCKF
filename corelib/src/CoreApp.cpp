/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/


#include "CoreApp.h"
#include "ImageData.h"

CoreApp::CoreApp(): voMode(0) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::NodeHandle camera_nh(nh, "/camera/rgb");
  ros::NodeHandle camera_pnh(pnh, "/camera/rgb");  // camera/depth_registered/image_raw
  ros::NodeHandle depth_nh(nh, "/camera/depth_registered");
  ros::NodeHandle depth_pnh(pnh, "/camera/depth_registered");

  image_transport::ImageTransport camera_it(camera_nh);
  image_transport::TransportHints hintsCamera("raw", ros::TransportHints(), camera_pnh);
  image_sub_.subscribe(camera_it, camera_nh.resolveName("image_rect_color"), 1, hintsCamera);
  image_info_sub_.subscribe(camera_nh, "camera_info", 1);

  image_transport::ImageTransport depth_it(depth_nh);
  image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);
  depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_raw"), 1, hintsDepth);

  if (voMode == 0) {
    depthExactSync_ = new message_filters::Synchronizer<MyDepthExactSyncPolicy>(MyDepthExactSyncPolicy(10),
      image_sub_, image_info_sub_, depth_sub_);
    depthExactSync_->registerCallback(boost::bind(&CoreApp::depthCallback, this, _1, _2, _3));
  }
  else {
    imageExactSync_ = new message_filters::Synchronizer<MyImageExactSyncPolicy>(MyImageExactSyncPolicy(10),
      image_sub_, image_info_sub_);
    imageExactSync_->registerCallback(boost::bind(&CoreApp::cameraCallback, this, _1, _2));
  }
}

void CoreApp::cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg) {
  ImageData* rgb = new ImageData(imageMsg, camInfoMsg);
}

void CoreApp::depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImageConstPtr& depthMsg) {
  ImageData* rgb = new ImageData(imageMsg, camInfoMsg);
  ImageData* depth = new ImageData(depthMsg);
}
