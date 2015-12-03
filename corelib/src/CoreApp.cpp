/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/


#include "CoreApp.h"

CoreApp::CoreApp() {
  ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

  ros::NodeHandle mono_nh(nh, "mono");
	ros::NodeHandle mono_pnh(pnh, "mono");
  image_transport::ImageTransport mono_it(mono_nh);
	image_transport::TransportHints hintsMono("raw", ros::TransportHints(), mono_pnh);
  image_sub_.subscribe(mono_it, mono_nh.resolveName("image_rect"), 1, hintsMono);
  monoExactSync_ = new message_filters::Synchronizer<MyMonoExactSyncPolicy>(MyMonoExactSyncPolicy(10), image_sub_, image_info_sub_);
}
