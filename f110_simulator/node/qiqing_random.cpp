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

ros::Publisher command_pub;
ros::Publisher driving_mode_pub;

//                       0:       1:       2:      3:
float mapping[5][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 0.0}, {0.0, 0.0}};

float speed_limit = 1.8;
float angle_limit = 0.3;

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

//get the random speed and its direction
unsigned generateRandomIndex(unsigned array_length){
  unsigned idx = rand() % array_length;
  return idx;
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qiqing_random");

  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_random", 1);

  float speed = 0.0;
  float angle = 0.0;

  while(ros::ok()) {
    unsigned index = generateRandomIndex(4);

    speed = mapping[index][0];
    angle = mapping[index][1];

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = angle * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    //command_pub.publish(drive_st_msg);
    command_pub.publish(drive_st_msg);

    loop_rate.sleep();
  }
  return 0;
}
