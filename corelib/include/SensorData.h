/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef SensorData_H_
#define SensorData_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>

#include "CameraModel.h"

struct PNP_RESULT {
  cv::Mat rvec;
  cv::Mat tvec;
  int inliers;
  PNP_RESULT(cv::Mat r, cv::Mat t, int i): rvec(r), tvec(t), inliers(i){}
  PNP_RESULT() {}
  friend std::ostream& operator<<(std::ostream &strm, const PNP_RESULT &a) {
    return strm << "Translation: " << a.tvec << ", Rotation: " << a.rvec << ", inlier number: " << a.inliers;
  }
};

class SensorData {
public:
  // SensorData() {}
  SensorData(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
  SensorData(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImageConstPtr& depthMsg);
  std::vector<cv::DMatch> match(SensorData* other);
  PNP_RESULT estimateMotion(SensorData* other, CameraModel& camera);
  inline cv::Point3f point2dTo3d(cv::Point3f& point, CameraModel& camera);

  sensor_msgs::ImageConstPtr rosImageMsg;
  cv::Mat cvRgbImage;
  cv::Mat cvDepthImage;
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  int64 id;

private:
  void convertCV(const sensor_msgs::ImageConstPtr& imageMsg, cv::Mat& to);
  void genFeatures();
  static int64 ID_SUCC;
};

#endif
