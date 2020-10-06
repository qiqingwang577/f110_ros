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
#include <math.h>

ros::Publisher command_pub;
ros::Publisher emergency_pub;

//
ros::Subscriber scan_sub;
ros::Subscriber speed_sub;

float time_threshold = 0.4;
float speed;

//get the current speed
void callback_speed(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  speed = (*msg).drive.speed;
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::vector<float> range = std::vector<float>(scan->ranges.begin(), scan->ranges.end());
  // in rad
  float angle_min = scan->angle_min;
  float angle_increment = scan->angle_increment;

  float distance;
  float alpha;
  float projected_speed;
  float ttc;

  //calculate TTC for all directions
  for (int i = 0; i < range.size(); i++){
    distance = range.at(i);
    alpha = angle_min + i * angle_increment;
    projected_speed = speed * cos(alpha);

    //let TTC be a large nummer when the car is close to wall, but car go backwards to wall//
    if(projected_speed <= 0){
       projected_speed = 0.001;
    }

    ttc = distance / projected_speed;

    if (ttc <= time_threshold){
      std_msgs::Header header;
      header.stamp = ros::Time::now();

      //if emergency, car will break and go backwards
      //Ackermann
      ackermann_msgs::AckermannDrive drive_msg;
      drive_msg.speed = -0.5;

      //AckermannStamped
      ackermann_msgs::AckermannDriveStamped drive_st_msg;
      drive_st_msg.header = header;
      drive_st_msg.drive = drive_msg;
      command_pub.publish(drive_st_msg);
      sleep(2);

      //stop the car
      drive_msg.speed = 0.0;
      drive_st_msg.drive = drive_msg;
      // publish AckermannDriveStamped message to drive topic
      command_pub.publish(drive_st_msg);

      break;
    }
  }

}

int main(int argc, char *argv[]) {
   ros::init(argc, argv, "qiqing_aeb");
   ros::NodeHandle n;

   command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_emergency", 1);

   speed_sub = n.subscribe("/drive", 1,callback_speed);
   scan_sub = n.subscribe("/scan", 1, callback_scan);

   ros::spin();
   return 0;
}
