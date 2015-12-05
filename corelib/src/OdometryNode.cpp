/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

OdometryNode::OdometryNode() {

}

OdometryNode::OdometryNode(OdometryNode* father, cv::Mat tvec, cv::Mat rvec) {
  this->position = father->position - tvec;
  
}
