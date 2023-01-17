#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "barbot_move/barbot_move.hpp"
#include "barbot_move/defs.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ROS_INFO("Start moving");
  barbot::BarbotActions barbot_actions;
  barbot_actions.move_to_point(barbot::table1_loc);

  return 0;
}