/*
Copyright - Jialei Jin, jjlfolk@gmail.com
*/

#ifndef COREAPP_H_
#define COREAPP_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


class CoreApp {
public:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> image_info_sub_;
  typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyMonoExactSyncPolicy;
	message_filters::Synchronizer<MyMonoExactSyncPolicy> * monoExactSync_;
};


#endif
