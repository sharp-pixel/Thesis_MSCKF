/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#include "SensorData.h"

int64 SensorData::ID_SUCC = 0;

SensorData::SensorData(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
  : id(SensorData::ID_SUCC) {
  SensorData::ID_SUCC++;
  rosImageMsg = imageMsg;
  convertCV(imageMsg, this->cvRgbImage);
  genFeatures();

  // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
  // cv::imshow( "Display window", cvRgbImage );
  // cv::waitKey(0);
}


SensorData::SensorData(const sensor_msgs::ImageConstPtr& imageMsg,
  const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
  const sensor_msgs::ImageConstPtr& depthMsg)
  : SensorData(imageMsg, camInfoMsg) {
  convertCV(depthMsg, this->cvDepthImage);
}

void SensorData::convertCV(const sensor_msgs::ImageConstPtr& imageMsg, cv::Mat& to) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(imageMsg, imageMsg->encoding);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  to = cv_ptr->image;
}

void SensorData::genFeatures() {
  cv::SURF surf(400);
  cv::Mat mask = cv::Mat::ones(cvRgbImage.size(), CV_8U);
  surf(cvRgbImage, mask, keyPoints, descriptors);
}

std::vector<cv::DMatch> SensorData::match(SensorData* other) {
  std::vector<cv::DMatch> matches;
  cv::Ptr< cv::DescriptorMatcher > matcher = new cv::FlannBasedMatcher();
  matcher->match(this->descriptors, other->descriptors, matches);
  return matches;
}

PNP_RESULT SensorData::estimateMotion(SensorData* other, CameraModel& camera) {
  std::vector<cv::DMatch> matches = this->match(other);
  // std::cout << "keyPoints number: " << keyPoints.size() << ", matches number:" << matches.size() << std::endl;
  if (matches.size() == 0) return PNP_RESULT(cv::Mat(), cv::Mat(), 0);
  std::vector<cv::DMatch> goodMatches;
  double minDis = 99999;
  double good_match_threshold = 4;
  for ( size_t i=0; i < matches.size(); i++ ) {
    if ( matches[i].distance < minDis )
      minDis = matches[i].distance;
  }

  for ( size_t i=0; i < matches.size(); i++ ) {
    if (matches[i].distance < good_match_threshold*minDis)
      goodMatches.push_back(matches[i]);
  }

  // std::cout << "goodMatches number: " << goodMatches.size() << std::endl;

  std::vector<cv::Point3f> pts_obj;
  std::vector<cv::Point2f> pts_img;

  for (size_t i=0; i < goodMatches.size(); i++) {
    cv::Point2f p = this->keyPoints[goodMatches[i].queryIdx].pt;
    ushort d = this->cvDepthImage.ptr<ushort>(int(p.y))[int(p.x)];
    if (d == 0)
      continue;
    // std::cout << "Point2f: " << cv::Point2f(other->keyPoints[goodMatches[i].trainIdx].pt) << std::endl;
    pts_img.push_back(cv::Point2f(other->keyPoints[goodMatches[i].trainIdx].pt));

    cv::Point3f pt(p.x, p.y, d);
    cv::Point3f pd = point2dTo3d(pt, camera);
    // std::cout << "p: " << p << std::endl;
    // std::cout << "Point3f: " << pd << std::endl;
    pts_obj.push_back(pd);
  }

  double camera_matrix_data[3][3] = {
    {camera.fx, 0, camera.cx},
    {0, camera.fy, camera.cy},
    {0, 0, 1}
  };
  cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
  cv::Mat rvec, tvec, inliers;
  // std::cout << "cameraMatrix: " << cameraMatrix << std::endl;
  cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

  PNP_RESULT result;
  result.rvec = rvec;
  result.tvec = tvec;
  result.inliers = inliers.rows;

  return result;
}

cv::Point3f SensorData::point2dTo3d(cv::Point3f& point, CameraModel& camera) {
  cv::Point3f p;
  p.z = double(point.z) / camera.scale;
  p.x = (point.x - camera.cx) * p.z / camera.fx;
  p.y = (point.y - camera.cy) * p.z / camera.fy;
  return p;
}
