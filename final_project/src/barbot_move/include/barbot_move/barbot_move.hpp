#ifndef BARBOT_ACTIONS
#define BARBOT_ACTIONS

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_broadcaster.h>
#include "barbot_move/defs.hpp"
#include <aruco_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <map>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;

typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;
typedef boost::shared_ptr< torso_control_client>  torso_control_client_Ptr;
typedef boost::shared_ptr< gripper_control_client>  gripper_control_client_Ptr;


namespace barbot
{
class BarbotActions
{
public:
    BarbotActions();
    bool move_to_point(double loc[]);
    bool move_acturator_to_cart(double x, double y, double z, double roll, double pitch, double yaw);
    // bool move_arm_joints(double j1=0.2, double j2=-1.34, double j3=-0.2, double j4=1.94, double j5=-1.57, double j6=1.37, double j7=0.00);
    bool move_arm_joints(jointParams params);
    bool move_head_joints(double head_1_joint, double head_2_joint);
    bool move_torso_joints(double z);
    bool move_gripper_joints(double gripper_left, double gripper_right);

    bool make_drink(std::vector<int> ingredient_list, const aruco_msgs::MarkerArray& msg);

    void standy();
    void putdown();
    void deliver_to_table(int table_number);
    void d2sconvert(double data[], jointParams& params);
    jointParams jointParams_;
    headParams headParams_;

private:
    void createArmClient(arm_control_client_Ptr& actionClient);
    void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, jointParams params);

    void createHeadClient(head_control_client_Ptr& actionClient);
    void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal, double head_1_joint, double head_2_joint);

    void createTorsoClient(torso_control_client_Ptr& actionClient);
    void waypoints_torso_goal(control_msgs::FollowJointTrajectoryGoal& goal, double z);

    void createGripperClient(gripper_control_client_Ptr& actionClient);
    void waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal, double gripper_left, double gripper_right);

};
}

#endif


