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

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("basic_odom", 50);
}

// TODO: develop mono rgb vo
void CoreApp::cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg) {
  SensorData* rgb = new SensorData(imageMsg, camInfoMsg);
  std::cout << "..." << std::endl;
}

void CoreApp::depthCallback(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImageConstPtr& depthMsg) {
  int minInliers = 5;
  double maxNorm = 0.3;
  // std::cout << "..." << std::endl;
  // std::cout << imageMsg->header.stamp << std::endl;
  // std::cout << camInfoMsg->header.stamp << std::endl;
  // std::cout << depthMsg->header.stamp << std::endl;
  currentFrame = new SensorData(imageMsg, camInfoMsg, depthMsg);
  // Drop first 3 frame
  if (currentFrame->id == 3) {
    odoms = new Odometry(new OdometryNode(currentFrame));
  }
  else if (currentFrame->id > 4) {
    CameraModel cam(camInfoMsg);
    PNP_RESULT r = currentFrame->estimateMotion(odoms->last()->sensorData, cam);
    // std::cout << r << std::endl;

    if (r.inliers < minInliers) return;
    double norm = r.normofTransform();
    if (norm >= maxNorm) return;

    OdometryNode* newOdomNode = new OdometryNode(odoms->last(), r.tvec, r.rvec, currentFrame);
    odoms->addOdomNode(newOdomNode);
    std::cout << "odoms size: " << odoms->size() << std::endl;

    nav_msgs::Odometry odomMsg;
    geometry_msgs::TransformStamped odomTf;
    getOdomMsg(newOdomNode, odomMsg, odomTf);
    odom_broadcaster_.sendTransform(odomTf);
    odom_pub_.publish(odomMsg);
  }
}

void CoreApp::getOdomMsg(OdometryNode* odomNode, nav_msgs::Odometry& odomMsg, geometry_msgs::TransformStamped& odomTf) {
  geometry_msgs::Quaternion odomQuat;
  geometry_msgs::TransformStamped odom_trans;

  vector<double> t = odomNode->getT();
  Eigen::Quaterniond q = odomNode->getQ();
  odomQuat.x = q.x(); odomQuat.y = q.y(); odomQuat.z = q.z(); odomQuat.w = q.w();

  odomTf.header.stamp = odomNode->sensorData->timeStamp.toRosTime();
  odomTf.header.frame_id = "odom";
  odomTf.child_frame_id = "basic_odom";

  odomTf.transform.translation.x = t[0];
  odomTf.transform.translation.y = t[1];
  odomTf.transform.translation.z = t[2];
  odomTf.transform.rotation = odomQuat;

  odomMsg.header.stamp = odomNode->sensorData->timeStamp.toRosTime();
  odomMsg.header.frame_id = "odom";
  odomMsg.child_frame_id = "basic_odom";

  odomMsg.pose.pose.position.x = t[0];
  odomMsg.pose.pose.position.y = t[1];
  odomMsg.pose.pose.position.z = t[2];
  odomMsg.pose.pose.orientation = odomQuat;
}
