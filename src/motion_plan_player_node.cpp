#include <ros/ros.h>
#include "motion_plan_player/motion_plan_player.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan_player");
  MotionPlanPlayer player;
  player.spin();
  return EXIT_SUCCESS;
}
