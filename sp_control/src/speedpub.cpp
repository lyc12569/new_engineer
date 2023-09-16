#include <eigen3/Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include "speedpub.h"

geometry_msgs::PoseStamped lastPose,currentPose;
geometry_msgs::TwistStamped vel;
geometry_msgs::PoseStamped emptyPose;
double dx,dy,dz,lyaw,lpitch,lroll,cyaw,cpitch,croll;
ros::Publisher* pub_vel;

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg_p)
{
    if(lastPose.pose == emptyPose.pose){
        lastPose.pose = msg_p->pose;
    }
    else{
        lastPose.pose = currentPose.pose;
        currentPose.pose =  msg_p->pose;
    
        // ROS_INFO_STREAM(lastPose.pose);
        // ROS_INFO_STREAM(currentPose.pose);
        tf::Quaternion quatl,quatc;
        tf::quaternionMsgToTF(lastPose.pose.orientation, quatl);
        tf::quaternionMsgToTF(currentPose.pose.orientation, quatc);
        tf::Matrix3x3(quatl).getRPY(lroll, lpitch, lyaw); //进行转换
        tf::Matrix3x3(quatc).getRPY(croll, cpitch, cyaw); //进行转换
        dx = currentPose.pose.position.x - lastPose.pose.position.x;
        dy = currentPose.pose.position.y - lastPose.pose.position.y;
        dz = currentPose.pose.position.z - lastPose.pose.position.z;
        vel.twist.linear.y = -linearx*dx*publish_rate;
        vel.twist.linear.x = -lineary*dy*publish_rate;
        vel.twist.linear.z = -linearz*dz*publish_rate;
        vel.twist.angular.z = angularx*(cyaw - lyaw)*publish_rate;
        vel.twist.angular.y = angulary*(cpitch - lpitch)*publish_rate;
        vel.twist.angular.x = angularz*(croll - lroll)*publish_rate;
        vel.header.frame_id = "arm_base_link";
        vel.header.stamp = ros::Time::now();
        // if(vel.twist.linear.x != 0 && vel.twist.linear.y != 0 && vel.twist.linear.z != 0 
        // && vel.twist.angular.x != 0 && vel.twist.angular.y != 0 && vel.twist.angular.z != 0) 
        pub_vel->publish(vel);

        //angular.x -> joint5
        //angular.y -> joint6
        //angular.z -> joint7
     }
}



int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"speedpub");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds",10);
    pub_vel = &(pub);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("ORB_SLAM3/pose",10,&pose_callback);



    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号


        //暂无应用
        ros::spinOnce();
    }


    return 0;
}