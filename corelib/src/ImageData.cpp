/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#include "ImageData.h"

ImageData::ImageData(const sensor_msgs::ImageConstPtr& imageMsg) {
  rosImageMsg = imageMsg;
  convertCV(imageMsg);
  // imshow("Display window", cvImage);
  genFeatures();
}


ImageData::ImageData(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
  : ImageData(imageMsg) {
  // TODO: keep camera info
}

void ImageData::convertCV(const sensor_msgs::ImageConstPtr& imageMsg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(imageMsg, imageMsg->encoding);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  this->cvImage = cv_ptr->image;
}

void ImageData::genFeatures() {
  cv::SURF surf(400);
  cv::Mat mask = cv::Mat::zeros(cvImage.size(), CV_8U);
  surf(cvImage, mask, keyPoints, descriptors);
}

void ImageData::match(ImageData* other) {
  cv::Ptr< cv::DescriptorMatcher > matcher = new cv::FlannBasedMatcher();
  matcher->match(this->descriptors, other->descriptors, matches);
}
