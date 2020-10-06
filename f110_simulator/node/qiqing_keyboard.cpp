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
#include <std_msgs/Char.h>

ros::Publisher command_pub;
ros::Publisher driving_mode;

//                       0:       1:       2:      3:
float mapping[5][2] = {{1.0, 0.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 0.0},{0.0, 0.0}};

float speed_limit = 1.5;
float angle_limit = 0.2;

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

unsigned keyToIndex(char message) {
  unsigned res;
  if (message == 'w') {       //forward
    res = 0;
  }
  else if (message == 'd') {  //right
    res = 1;
  }
  else if (message == 'a') {  //left
    res = 2;
  }
  else if (message == 's') {  //backwards
    res = 3;
  }
  else if (message == ' ') {  //break
    res = 4;
  }
  else {
    res = 4;
  }
  return res;
}

char choosemode(char input){
  switch (input) {
    case 'r':                //mode: go random
      return 'r';
    case 'f':                //mode: follow the gap
      return 'f';
    case 'p':                //mode: pid Controller
        return 'p';
    default:                 //mode: keyboard kontroll
      return 'k';
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "qiqing_keyboard");
  ros::NodeHandle n;

  driving_mode = n.advertise<std_msgs::Char>("/mode", 1);
  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive_keyboard", 1);

  float speed = 0.0;
  float angle = 0.0;

  while(ros::ok()) {
    char input  = getch();
    unsigned index = keyToIndex(input);
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

    command_pub.publish(drive_st_msg);

    //publish message for driving mode
    std_msgs::Char mode_msg;
    mode_msg.data = choosemode(input);
    driving_mode.publish(mode_msg);
  }
  return 0;
}
