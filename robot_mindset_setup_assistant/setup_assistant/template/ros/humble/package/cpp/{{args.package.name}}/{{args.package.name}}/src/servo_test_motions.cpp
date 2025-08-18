/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_test_motions.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_COMMA 0x2C
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "tcp";
const std::string BASE_FRAME_ID = "base_link";

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char *c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();
  void command_circ(double r_x,
                    double r_y,
                    rclcpp::Rate &rate);

  rclcpp::Node::SharedPtr nh_;
  // Member variable to store the callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;
  rcl_interfaces::msg::SetParametersResult parameter_change_callback(const std::vector<rclcpp::Parameter> &params);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  std::string frame_to_publish_;
  double joint_vel_cmd_;
  double linear_twist_vel_;
};

KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID)
{
  nh_ = rclcpp::Node::make_shared("servo_test_motions");

  nh_->declare_parameter("vel", 0.1);
  linear_twist_vel_ = nh_->get_parameter("vel").as_double();

  parameter_callback_handle = nh_->add_on_set_parameters_callback(
      std::bind(&KeyboardServo::parameter_change_callback, this, std::placeholders::_1));

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
}

rcl_interfaces::msg::SetParametersResult KeyboardServo::parameter_change_callback(const std::vector<rclcpp::Parameter> &params)
{
  // Create a result object to report the outcome of parameter changes.
  auto result = rcl_interfaces::msg::SetParametersResult();

  // Assume success unless an unsupported parameter is encountered.
  result.successful = true;

  // Iterate through each parameter in the change request.
  for (const auto &param : params)
  {
    // Check if the changed parameter is 'velocity_limit' and of type double.
    if (param.get_name() == "vel" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      RCLCPP_INFO(nh_->get_logger(), "Parameter vel has changed. The new value is: %f", param.as_double());
      this->linear_twist_vel_ = param.as_double();
    }
    else
    {
      // Mark the result as unsuccessful and provide a reason.
      result.successful = false;
      result.reason = "Unsupported parameter";
    }
  }
  // Return the result object, indicating whether the parameter change(s) were successful or not.
  return result;
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

/// @brief Create a circular path
/// @param rx Radius in the x direction
/// @param ry Radius in the y direction
/// @param step Time variable (angle)
/// @param x Output x coordinate
/// @param y Output y coordinate
void create_circ(double rx, double ry, double step, double &x, double &y)
{
  // Create a circular path
  x = rx * cos(step);
  y = ry * sin(step);
}

void KeyboardServo::command_circ(double r_x,
                                 double r_y,
                                 rclcpp::Rate &rate)
{
  RCLCPP_WARN(nh_->get_logger(), "Creating circular path");
  for (double step = 0; step < 2 * M_PI; step += 0.1)
  {
    double x, y;
    create_circ(r_x, r_y, step, x, y);
    RCLCPP_INFO(nh_->get_logger(), "Circle point: (%f, %f)", x, y);
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->twist.linear.x = x;
    twist_msg->twist.linear.z = y;

    twist_msg->header.stamp = nh_->now();
    twist_msg->header.frame_id = frame_to_publish_;
    twist_pub_->publish(std::move(twist_msg));
    rate.sleep();
  }
  RCLCPP_WARN(nh_->get_logger(), "Circular path completed");
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();
  rclcpp::Rate sleepRate_50(50);  // 50 Hz
  rclcpp::Rate sleepRate_25(25);  // 25 Hz
  rclcpp::Rate sleepRate_20(20);  // 20 Hz
  rclcpp::Rate sleepRate_10(10);  // 10 Hz
  rclcpp::Rate sleepRate_5(5);    // 5 Hz
  rclcpp::Rate sleepRate_2(2);    // 2 Hz
  rclcpp::Rate sleepRate_1(1);    // 1 Hz
  rclcpp::Rate sleepRate_05(0.5); // 0.5 Hz

  // double r_x = 0.15;              // Radius in the x direction
  // double r_y = 0.15;              // Radius in the y direction
  // double linear_twist_vel = 0.15; // Linear velocity for Cartesian jog

  
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("'Q' to quit.");
  
  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return -1;
    }
    
    // Use the same radius for both x and y to create a circular path
    double linear_twist_vel = linear_twist_vel_; // Linear velocity for Cartesian jog
  
    double r_x = linear_twist_vel_;              // Radius in the x direction
    double r_y = linear_twist_vel_;              // Radius in the y direction
    
    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    {
    case KEYCODE_LEFT:
      RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
      twist_msg->twist.linear.y = -linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_RIGHT:
      RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
      twist_msg->twist.linear.y = linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_UP:
      RCLCPP_DEBUG(nh_->get_logger(), "UP");
      twist_msg->twist.linear.x = linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_DOWN:
      RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
      twist_msg->twist.linear.x = -linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_PERIOD:
      RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
      twist_msg->twist.linear.z = -linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_COMMA:
      RCLCPP_DEBUG(nh_->get_logger(), "COMMA");
      twist_msg->twist.linear.z = linear_twist_vel;
      publish_twist = true;
      break;
    case KEYCODE_E:
      RCLCPP_DEBUG(nh_->get_logger(), "E");
      frame_to_publish_ = EEF_FRAME_ID;
      break;
    case KEYCODE_W:
      RCLCPP_DEBUG(nh_->get_logger(), "W");
      frame_to_publish_ = BASE_FRAME_ID;
      break;
    case KEYCODE_1:
      command_circ(r_x, r_y, sleepRate_10);
      break;
    case KEYCODE_2:
      command_circ(r_x, r_y, sleepRate_5);
      break;
    case KEYCODE_3:
      command_circ(r_x, r_y, sleepRate_2);
      break;
    case KEYCODE_4:
      command_circ(r_x, r_y, sleepRate_1);
      break;
    case KEYCODE_5:
      command_circ(r_x, r_y, sleepRate_05);
      break;
    case KEYCODE_6:
      command_circ(r_x, r_y, sleepRate_25);
      break;
    case KEYCODE_7:
      command_circ(r_x, r_y, sleepRate_20);
      break;
    case KEYCODE_R:
      RCLCPP_DEBUG(nh_->get_logger(), "R");
      joint_vel_cmd_ *= -1;
      break;
    case KEYCODE_Q:
      RCLCPP_DEBUG(nh_->get_logger(), "quit");
      return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
      std::cout << "\033[1;32m[KeyboardServo]: Published Twist command to " << frame_to_publish_ << " frame.\033[0m" << std::endl
                << std::flush;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
      std::cout << "\033[1;32m[KeyboardServo]: Published Joint Jog command.\033[0m" << std::endl
                << std::flush;
    }
  }

  return 0;
}
