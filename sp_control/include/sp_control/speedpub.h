/**********res.h声明全局变量************/
#pragma once
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

 
const int publish_rate = 15; 
 
extern char g_szBuffer[]; // 环形缓冲区
extern geometry_msgs::PoseStamped lastPose,currentPose;
extern geometry_msgs::TwistStamped vel;
extern ros::Publisher* pub_vel;
float linearx = 40;
float lineary = 50;
float linearz = 40;
float angularx = 20;
float angulary = 20;
float angularz = 20;
