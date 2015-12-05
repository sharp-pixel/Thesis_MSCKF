/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <vector>
#include <cassert>

#include "OdometryNode.h"

class Odometry {
public:
  explicit Odometry(OdometryNode* odomNode);
  void addOdomNode(OdometryNode* odomNode);
  inline OdometryNode* last() {return odoms.back();}
  int size() {return odoms.size();};

private:
  std::vector<OdometryNode*> odoms;
};

#endif
