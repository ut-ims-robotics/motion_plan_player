// Include ROS
#include <ros/ros.h>
#include <ros/package.h>
#include "motion_planning/motion_plan_player.h"
#include "sensor_msgs/JointState.h"

// Include standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

MotionPlanPlayer::MotionPlanPlayer(): node_("~")
{

  // Read parameters
  node_.param<double>("rate", rate_, 20);

  load();

  std::string topic;
  node_.param<std::string>("topic", topic, "joint_states");
  pub_ = node_.advertise<sensor_msgs::JointState>(topic, 1);
}

// Read the datapoints from a file
bool MotionPlanPlayer::load()
{
  std::string pkg_path = ros::package::getPath(ROS_PACKAGE_NAME); 
  std::string file_path;
  node_.param<std::string>("file", file_path, pkg_path + "/sample_data/dataset.csv");

  ROS_INFO("Loading joint states from: %s", file_path.c_str());

  // reset the plan
  plan_.clear();

  // open csv file for reading
  std::fstream file(file_path.c_str(), std::ios::in);

  std::string::size_type sz;  // alias of size_t
  std::string line = "";
  while (getline(file, line))
  {
    std::stringstream linestream(line);
    std::string value;
    std::vector<double> joint_states;

    // parse the line (comma separated joint states)
    while (getline(linestream, value, ','))
    {
      double val = std::strtod(value.c_str(), NULL);
      joint_states.push_back(val);
    }

    // add joint states to the plan
    if(joint_states.size())
    {
      plan_.emplace_back(joint_states);
    }
  }

  plan_it_ = plan_.begin();

  return true;
}

bool MotionPlanPlayer::publish()
{
  if (plan_it_ == plan_.end())
  {
    ROS_INFO("Restarting...");
    plan_it_ = plan_.begin();
    return false;
  }

  ROS_INFO("Publishing joint states");
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  std::string base_name("panda_joint");
  for (int i = 1; i < 8; i++)
  {
    msg.name.push_back(base_name + std::to_string(i));
  }
  msg.position = *plan_it_;

  // publish the message
  pub_.publish(msg);

  plan_it_++;  // next timestep
  return true;
}


bool MotionPlanPlayer::spin()
{
  std::string file_path = "/home/veix/catkin_ws/src/motion_plan_player/sample_data/dataset.csv";
  ros::Rate loop_rate(this->rate_);
  while (node_.ok())
  {
    if(!publish())
    {
      load();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}
