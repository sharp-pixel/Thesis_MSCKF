/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef ODOETRYNODE_H_
#define ODOETRYNODE_H_


#include <opencv2/core/core.hpp>

class OdometryNode {
public:
  OdometryNode();
  OdometryNode(OdometryNode* father, cv::Mat tvec, cv::Mat rvec);

private:
  cv::Mat position;
  cv::Mat orientation;
};

#endif
