/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#include "OdometryNode.h"

OdometryNode::OdometryNode(SensorData* sensorData)
  : sensorData(sensorData) {
  T = Eigen::Isometry3d::Identity();
  accumT = Eigen::Isometry3d::Identity();
  parent = NULL;
}

OdometryNode::OdometryNode(OdometryNode* parent, cv::Mat tvec, cv::Mat rvec, SensorData* sensorData)
  : parent(parent), tvec(tvec), rvec(rvec), sensorData(sensorData) {
  this->T = cvMat2Eigen(rvec, tvec);
  this->accumT = parent->accumT * this->T;
}

Eigen::Isometry3d OdometryNode::cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d r;
  cv::cv2eigen(R, r);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd angle(r);
  Eigen::Translation<double, 3> trans(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2));
  T = angle;
  T(0, 3) = tvec.at<double>(0, 0);
  T(1, 3) = tvec.at<double>(0, 1);
  T(2, 3) = tvec.at<double>(0, 2);
  return T;
}
