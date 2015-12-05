/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#include "Odometry.h"

Odometry::Odometry(OdometryNode* odomNode) {
  odoms.push_back(odomNode);
}

void Odometry::addOdomNode(OdometryNode* odomNode) {
  assert(odomNode->parent == odoms.back());
  odoms.push_back(odomNode);
}
