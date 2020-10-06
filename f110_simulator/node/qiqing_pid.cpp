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
#include <cmath>


ros::Publisher command_pub;
ros::Publisher driving_mode_pub;

ros::Subscriber scan_sub;

float speed = 1.0;
float target_distance = 1.5;
float alpha, distmin_angle, num_increment, dt, u_t;
float angle = 0.0;
int frequency = 20;

float Kp = 1.7;   //1.0
float Kd = 0.08;  //K_correct = 0.002
float Ki = 0.008; //0.008
float error_previous = 0.0;
float error_sum = 0.0;

void callback_pid_drive(const ackermann_msgs::AckermannDriveStamped:: ConstPtr& msg){
  //get values from drive
  ackermann_msgs::AckermannDrive drive_msg;
  drive_msg = msg->drive;
  angle = msg->drive.steering_angle;
}

void callback_pid_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
  //get values from scan
  std::vector<float> range = std::vector<float>(scan->ranges.begin(), scan->ranges.end());
  int num_increment = range.size();
  float angle_min = scan->angle_min;
  float angle_increment = scan->angle_increment;

  float error_current = 0.0;

  //find minimum distance and its index, if current distmin is smaller than the previous, overwrite it
  //focus on the wall on the right side
  int distmin_index = 0;
  float distmin = 100000;
  for (int i = 0; i < range.size()/2; i++){
    if(range.at(i) < distmin){
       distmin = range.at(i);
       distmin_index = i;
    }
  }
  //calculate angle towards to wall
  //in rad
  float alpha_in_rad = angle_increment * (270 - distmin_index);
  //caculate in Grad
  float alpha = alpha_in_rad / M_PI * 180;

  error_current = target_distance - (distmin + sin(alpha)*speed*1/frequency);
  error_previous = error_current;
  error_sum += error_current;

  //caculate timestep
  dt = 1 / frequency;

  //float factor = 0.0025;
  //u_t = Kp * error_current + Kd * factor * ((error_previous - error_current) / dt) + Ki * error_sum;

  //u_t = Kp_correct * error_current + Kd * ((error_previous - error_current) / dt) + Ki * error_sum;

  u_t = Kp * error_current + Kd * ((error_previous - error_current) * dt) + Ki * error_sum;

  //steering angle limit
  if(u_t >= 1){
    angle = 1;
  }
  else if( u_t <= -1){
    angle = -1;
  }
  else{
    angle = u_t;
  }
  //ROS_INFO ("alpha %f", alpha);
  //ROS_INFO ("angle %f", angle);
  //ROS_INFO ("Kd * (error_previous - error_current) %d", f);
  //ROS_INFO ("error_sum %f", error_sum);
  //ROS_INFO ("error_current %f", error_current);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qiqing_pid");
  ros::NodeHandle n;
  ros::Rate loop_rate(frequency);

  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_pid", 1);
  ros::Subscriber pid_scan_sub = n.subscribe("/scan", 1, callback_pid_scan);
  ros::Subscriber pid_drive_sub = n.subscribe("/drive", 1, callback_pid_drive);

  while(ros::ok()) {

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
