/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <sensor_msgs/CameraInfo.h>

class CameraModel {
public:
  double fx;
  double fy;
  double cx;
  double cy;
  double scale;
  double D[4];
  double K[3][3];
  double R[3][3];
  double P[3][4];
  CameraModel(double _fx, double _fy, double _cx, double _cy) : fx(_fx), fy(_fy), cx(_cx), cy(_cy), scale(1000.0) {}
  CameraModel(double _fx, double _fy, double _cx, double _cy, double _scale) : fx(_fx), fy(_fy), cx(_cx), cy(_cy), scale(_scale) {}
  CameraModel(const double D[], const double K[], const double R[], const double P[]) {
    // this->D = D;
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 4; j++) {
        this->P[i][j] = P[i * 4 + j];
        if (j == 3) continue;
        this->K[i][j] = K[i * 3 + j];
        this->R[i][j] = R[i * 3 + j];
      }
    }
    fx = this->K[0][0]; fy = this->K[1][1]; cx = this->K[0][2]; cy = this->K[1][2];
    scale = 1000;
  }
  CameraModel(const sensor_msgs::CameraInfoConstPtr& camInfoMsg) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 4; j++) {
        this->P[i][j] = camInfoMsg->P[i * 4 + j];
        if (j == 3) continue;
        this->K[i][j] = camInfoMsg->K[i * 3 + j];
        this->R[i][j] = camInfoMsg->R[i * 3 + j];
      }
    }
    fx = this->K[0][0]; fy = this->K[1][1]; cx = this->K[0][2]; cy = this->K[1][2];
    scale = 1000;
  }
};

#endif
