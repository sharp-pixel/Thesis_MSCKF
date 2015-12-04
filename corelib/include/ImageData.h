/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef IMAGEDATA_H_
#define IMAGEDATA_H_

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

#include <vector>

class ImageData {
public:
  ImageData() {}
  ImageData(const sensor_msgs::ImageConstPtr& imageMsg);
  ImageData(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
  void match(ImageData* other);

  sensor_msgs::ImageConstPtr rosImageMsg;
  cv::Mat cvImage;
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  cv::vector<cv::DMatch> matches;

private:
  void convertCV(const sensor_msgs::ImageConstPtr& imageMsg);
  void genFeatures();
};

#endif
