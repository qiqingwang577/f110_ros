#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>


ros::Publisher command_pub;
ros::Publisher driving_mode_pub;

ros::Subscriber scan_sub;

float speed = 1.0;
float angle = 0.0;

int distmax_index, distmin_index;
float distmax, distmin, distmax_angle, distmin_angle;

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::vector<float> range = std::vector<float>(scan->ranges.begin(), scan->ranges.end());
  float angle_max = scan->angle_max;
  float angle_min = scan->angle_min;
  float angle_increment = scan->angle_increment;

  int num_increment = range.size();

  distmax_index = 0;
  distmin_index = 0;
  distmax = 0.0;
  distmin = 100000;
  distmax_angle = 0.0;
  distmin_angle = 0.0;

  //find maximum distance and minimum diatance with corresponding indexes
  //if current distmax is bigger/smaller than the previous, overwrite it
  for (int i = 270; i < (range.size()-270); i++){
    if(distmax < range.at(i)){
       distmax = range.at(i);
       distmax_index = i;
    }
    if(distmin > range.at(i)){
       distmin = range.at(i);
       distmin_index = i;
    }
  }

  distmax_angle = (distmax_index - 540) * angle_increment;
  distmin_angle = (distmin_index - 540) * angle_increment;  // angle_increment = 0.005823
/*  ROS_INFO ("distmax_index %i", distmax_index);
  ROS_INFO ("distmin_index %i", distmin_index);
  ROS_INFO ("distmax_angle %f", distmax_angle);
  ROS_INFO ("distmin_angle %f", distmin_angle);
  ROS_INFO ("distmax %f", distmax);
  ROS_INFO ("distmin %f", distmin);*/
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qiqing_follow");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  scan_sub = n.subscribe("/scan", 1, callback);
//  command_ = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_ftg", 1);

  while(ros::ok()) {
    //FAILED!!
    /*if(distmax_angle <= 0.0){
        angle = -1 * distmax_angle * distmax_angle;
      }
    else{
        angle = 1 * distmax_angle * distmax_angle;
      }
    if(distmin < 0.4){
        angle = - 0.5 * distmin_angle * distmin_angle;
      }
    if(abs(distmin_index - 540) <= 220){
       angle = - 1 * (distmin_index - 540)/abs(distmin_index - 540);
    }*/

    /*modify angle because of the worst case: small distance and small angle,
    that means a wall in front of car */
    angle = - 0.5 * (1/distmin) * (1/distmin_angle);

    //steering angle limit
    if(angle >= 1){
      angle = 1;
    }
    if(angle < -1){
      angle = -1;
    }

    //Dynamic speed, depending on distance to wall
    if(distmin > 1){
      speed = 1.5;
    } else if(0 <= distmin <= 1){
      speed = 0.8;
    } else{
      speed = 1.0;
    }
    //ROS_INFO ("distmin %f", distmin);
    //ROS_INFO ("speed %f", speed);

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = angle;

    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
