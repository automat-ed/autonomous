// Taken from:
// https://github.com/ros/ros_tutorials/blob/noetic-devel/turtlesim/tutorials/teleop_turtle_key.cpp

#include "controllers/KeyboardController.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "signal.h"
#include "stdio.h"
#include "termios.h"
#include "unistd.h"

#define KEY_UP 0x41
#define KEY_DOWN 0x42
#define KEY_RIGHT 0x43
#define KEY_LEFT 0x44
#define KEY_Q 0x71
#define KEY_S 0x73

///////////////////////
//  Keyboard Reader  //
///////////////////////

KeyboardReader::KeyboardReader() : kfd(0) {
  // Get console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char *c) {
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown() { tcsetattr(kfd, TCSANOW, &cooked); }

///////////////////////////
//  Keyboard Controller  //
///////////////////////////

KeyboardController::KeyboardController(ros::NodeHandle *nh)
    : linear_(0), angular_(0) {
  ROS_INFO("Starting C++ Keyboard Controller...");

  // ROS Node Handle
  nh_ = nh;

  // ROS Parameters
  nh_->param("avel_scale", a_scale_, 0.3);
  nh_->param("lvel_scale", l_scale_, 0.3);

  cmd_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

KeyboardController::~KeyboardController() { input.shutdown(); }

void KeyboardController::keyLoop() {
  char c;
  bool dirty = false;

  puts("Reading from keyboard");
  puts("------------------------------");
  puts("Use arrow keys to move. Press `q` to quit.");

  while (true) {
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return;
    }

    // Reset control values
    linear_ = 0;
    angular_ = 0;

    ROS_DEBUG("Keyboard value: 0x%02X\n", c);

    switch (c) {
    case KEY_UP:
      ROS_DEBUG("Up key pressed");
      linear_ = 1.0;
      dirty = true;
      break;
    case KEY_DOWN:
      ROS_DEBUG("Down key pressed");
      linear_ = -1.0;
      dirty = true;
      break;
    case KEY_RIGHT:
      ROS_DEBUG("Right key pressed");
      angular_ = -1.0;
      dirty = true;
      break;
    case KEY_LEFT:
      ROS_DEBUG("Left key pressed");
      angular_ = 1.0;
      dirty = true;
      break;
    case KEY_S:
      ROS_DEBUG("Stop");
      dirty = true;
      break;
    case KEY_Q:
      ROS_DEBUG("Quit");
      return;
    }

    if (dirty == true) {
      // Create control message
      geometry_msgs::Twist msg;
      msg.angular.z = a_scale_ * angular_;
      msg.linear.x = l_scale_ * linear_;
      cmd_pub_.publish(msg);
      dirty = false;
    }
  }

  return;
}

////////////
//  main  //
////////////

void quit(int sig) {
  (void)sig;
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_controller");
  ros::NodeHandle nh("~");

  KeyboardController controller(&nh);

  // Handle the SIGINT signal
  signal(SIGINT, quit);

  controller.keyLoop();

  quit(0);
  return 0;
}