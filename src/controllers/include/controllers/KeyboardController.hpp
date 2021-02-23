// Taken from:
// https://github.com/ros/ros_tutorials/blob/noetic-devel/turtlesim/tutorials/teleop_turtle_key.cpp

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "signal.h"
#include "stdio.h"
#include "termios.h"
#include "unistd.h"

class KeyboardReader {
public:
  KeyboardReader();

  void readOne(char *c);
  void shutdown();

private:
  int kfd;
  struct termios cooked;
};

class KeyboardController {
public:
  KeyboardController();
  ~KeyboardController();
  void keyLoop();

private:
  // ROS Handler
  ros::NodeHandle nh_;

  // Scale values
  double l_scale_;
  double a_scale_;

  // Control values
  double linear_;
  double angular_;

  // ROS Publisher
  ros::Publisher cmd_pub_;

  // Instantiate Keyboard Reader
  KeyboardReader input;
};