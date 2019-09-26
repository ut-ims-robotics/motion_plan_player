#ifndef MOTION_PLAN_PLAYER_H
#define MOTION_PLAN_PLAYER_H

#include <vector>
#include "string.h"

class MotionPlanPlayer
{
  public:
  MotionPlanPlayer();
  bool load();
  bool spin();
  bool publish();

  private:
    ros::NodeHandle node_;
    ros::Publisher pub_;
    double rate_;
    std::vector<std::vector<double> > plan_;
    std::vector<std::vector<double> >::const_iterator plan_it_;
};

#endif //MOTION_PLAN_PLAYER_H
