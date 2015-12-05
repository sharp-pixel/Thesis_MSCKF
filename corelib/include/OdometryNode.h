/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef ODOETRYNODE_H_
#define ODOETRYNODE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>

#include "SensorData.h"

class OdometryNode {
public:
  OdometryNode(SensorData* sensorData);
  OdometryNode(OdometryNode* parent, cv::Mat tvec, cv::Mat rvec, SensorData* sensorData);
  std::vector<double> getT()
    {std::vector<double> t; t.push_back(accumT(0,3)); t.push_back(accumT(1,3)); t.push_back(accumT(2,3)); return t;}
  Eigen::Quaterniond getQ() {return Eigen::Quaterniond(this->accumT.rotation()).normalized();}
  SensorData *sensorData;
  OdometryNode *parent;
  Eigen::Isometry3d T;
  Eigen::Isometry3d accumT;


private:
  cv::Mat tvec;
  cv::Mat rvec;

  Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);

};

#endif
