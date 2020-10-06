#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

ackermann_msgs::AckermannDriveStamped drive_st_msg;
ackermann_msgs::AckermannDrive drive_msg;

ros::Publisher command_pub;

char selected_mode = 'k';

void callback_mode(const std_msgs::Char::ConstPtr& msg) {
  selected_mode = msg->data;
}

void callback_keyboard(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  if(selected_mode == 'k'){
    //ROS_INFO("Now Mode: %s", "Keyboard");
    command_pub.publish(*msg);
  }
}

void callback_random(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  if(selected_mode == 'r'){
    //ROS_INFO("Now Mode: %s", "Random");
    command_pub.publish(*msg);
  }
}

void callback_ftg(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  if(selected_mode == 'f'){
    //ROS_INFO("Now Mode: %s", "Follow the gap");
    command_pub.publish(*msg);
  }
}

void callback_pid(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  if(selected_mode == 'p'){
    //ROS_INFO("Now Mode: %s", "PID Controller");
    command_pub.publish(*msg);
  }
}

void callback_drive_emergency(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
  selected_mode = 'k';
  ROS_INFO("Now: %s", "Emergency!!! now car will backward, and wait for your controll");
  command_pub.publish(*msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qiqing_mux");
  ros::NodeHandle n;

  //Subscribers
  ros::Subscriber mode_sub = n.subscribe("/mode", 1, callback_mode);
  ros::Subscriber keyboard_sub = n.subscribe("/drive_keyboard", 1, callback_keyboard);
  ros::Subscriber random_sub = n.subscribe("/drive_random", 1, callback_random);
  ros::Subscriber ftg_sub = n.subscribe("/drive_ftg", 1, callback_ftg);
  ros::Subscriber emergency_sub = n.subscribe("/drive_emergency", 1, callback_drive_emergency);
  ros::Subscriber pid_sub = n.subscribe("/drive_pid", 1, callback_pid);

  //Publisher
  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

  sleep(1);
  ROS_INFO("\n----------------------------------------\nWelcome to race!\n----------------------------------------\nIntroduction: Please choose your driving mode.\n1. Driving via PID controller [p]\n2. Driving via Scanner [f]\n3. Driving randomly [r]\n4. Driving via Keyboard [k]:\nw: Forward; a: Left; d: Right; s: Backwards; Space: Break");

  //loop frequency
  ros::spin();

  return 0;
}
