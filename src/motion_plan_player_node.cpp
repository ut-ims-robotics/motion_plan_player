#include <ros/ros.h>
#include "motion_planning/motion_plan_player.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_plan_player");
  MotionPlanPlayer player;
  player.spin();
  return EXIT_SUCCESS;
}
