/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/


#include "CoreWrapper.h"

CoreApp::CoreApp() {
  ros::NodeHandle mono_nh(nh, "left");
  image_transport::ImageTransport mono_it(mono_nh);
	image_transport::TransportHints hintsMono("raw", ros::TransportHints(), mono_pnh);
  image_sub_ = subscribe(mono_it, mono_nh.resolveName("image_rect"), 1, hintsMono);
  monoExactSync_ = new message_filters::Synchronizer<MyStereoApproxSyncPolicy>(MyMonoExactSyncPolicy(10), image_sub_, image_info_sub_);
}
