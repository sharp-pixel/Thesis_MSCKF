/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/


#include "CoreApp.h"

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
    depthApproxSync_ = new message_filters::Synchronizer<MyDepthApproxSyncPolicy>(MyDepthApproxSyncPolicy(10),
      image_sub_, image_info_sub_, depth_sub_);
    depthApproxSync_->registerCallback(boost::bind(&CoreApp::depthCallback, this, _1, _2, _3));
  }
  else {
    imageExactSync_ = new message_filters::Synchronizer<MyImageExactSyncPolicy>(MyImageExactSyncPolicy(10),
      image_sub_, image_info_sub_);
    imageExactSync_->registerCallback(boost::bind(&CoreApp::cameraCallback, this, _1, _2));
  }
}

// TODO: develop mono rgb vo
void CoreApp::cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg) {
  SensorData* rgb = new SensorData(imageMsg, camInfoMsg);
  std::cout << "..." << std::endl;
}

void CoreApp::depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImageConstPtr& depthMsg) {
  std::cout << "..." << std::endl;
  // std::cout << imageMsg->header.stamp << std::endl;
  // std::cout << camInfoMsg->header.stamp << std::endl;
  // std::cout << depthMsg->header.stamp << std::endl;
  previousFrame = currentFrame;
  currentFrame = new SensorData(imageMsg, camInfoMsg, depthMsg);
  std::cout << currentFrame->id << std::endl;
  if (currentFrame->id > 0) {
    CameraModel cam(camInfoMsg);
    PNP_RESULT r = currentFrame->estimateMotion(previousFrame, cam);
    std::cout << r << std::endl;
  }
}
