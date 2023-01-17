#include "barbot_move/barbot_move.hpp"

namespace barbot
{
    BarbotActions::BarbotActions()
    {
        ROS_INFO("Action inits");
    }

    void BarbotActions::d2sconvert(double data[], jointParams& params)
    {
        params.arm_joint_1 = data[0];
        params.arm_joint_2 = data[1];
        params.arm_joint_3 = data[2];
        params.arm_joint_4 = data[3];
        params.arm_joint_5 = data[4];
        params.arm_joint_6 = data[5];
        params.arm_joint_7 = data[6];
    }
    bool BarbotActions::move_to_point(double loc[]) // double x, double y, double qx, double qy, double qz, double qw
    {
        MoveBaseClient ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;    

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = loc[0];// 1.97;
        goal.target_pose.pose.position.y = loc[1];// -0.56;
        goal.target_pose.pose.orientation.x = loc[2];// 0.0;
        goal.target_pose.pose.orientation.y = loc[3];// 0.0;
        goal.target_pose.pose.orientation.z = loc[4];// 0.68;
        goal.target_pose.pose.orientation.w =  loc[5];// 0.72;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, Tiago has reached the destination");
            return EXIT_SUCCESS;
        }    
        else
        {
            ROS_INFO("The base failed to move to the destination");
            return EXIT_FAILURE;
        }    
    }

    bool BarbotActions::move_acturator_to_cart(double x, double y, double z, double roll, double pitch, double yaw) 
    {
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "base_footprint";
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.position.z = z;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        std::vector<std::string> torso_arm_joint_names;
        // select group of joints
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        // choose your preferred planner
        group_arm_torso.setPlannerId("SBLkConfigDefault");
        group_arm_torso.setPoseReferenceFrame("base_footprint");
        group_arm_torso.setPoseTarget(goal_pose);
        
        ROS_INFO_STREAM("Planning to move " <<
                    group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());

        group_arm_torso.setStartStateToCurrentState();
        group_arm_torso.setMaxVelocityScalingFactor(1.0);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // set the maximum time to find a plan
        group_arm_torso.setPlanningTime(5.0);
        bool success = bool(group_arm_torso.plan(my_plan));

        if( !success)
            throw std::runtime_error("No plan found");
        ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

        // excute the plan
        ros::Time start = ros::Time::now();
        moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
        if (!bool(e))
            throw std::runtime_error("Error executing plan");
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        spinner.stop();

        return EXIT_SUCCESS;
    }

    bool BarbotActions::move_arm_joints(jointParams params)
    {
        ROS_INFO("Starting run_traj_control application ...");
        if(!ros::Time::waitForValid(ros::WallDuration(10.0)))
        {
            ROS_FATAL("Timed-out waiting for valid time.");
            return EXIT_FAILURE;
        }
        arm_control_client_Ptr ArmClient;
        createArmClient(ArmClient);

        control_msgs::FollowJointTrajectoryGoal arm_goal;
        waypoints_arm_goal(arm_goal, params);

        // Sends the command to start the given trajectory 1s from now
        arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        ArmClient->sendGoal(arm_goal);

        // Wait for trajectory execution
        while(!(ArmClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds
        }
    }
    bool BarbotActions::move_head_joints(double head_1_joint, double head_2_joint)
    {
        ROS_INFO("Starting run_traj_control application ...");
        if(!ros::Time::waitForValid(ros::WallDuration(10.0)))
        {
            ROS_FATAL("Timed-out waiting for valid time.");
            return EXIT_FAILURE;
        }
        head_control_client_Ptr HeadClient;
        createHeadClient(HeadClient);

        control_msgs::FollowJointTrajectoryGoal head_goal;
        waypoints_head_goal(head_goal, head_1_joint, head_2_joint);

        head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        HeadClient->sendGoal(head_goal);

        while(!(HeadClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds
        }
    }
    bool BarbotActions::move_torso_joints(double z)
    {
        std::string s1 = "rosrun play_motion move_joint torso_lift_joint ";
        std::string s2 = " 2.0";
        std::string cmd = s1 + std::to_string(z) + s2;
        system(cmd.c_str());

        /*
        ROS_INFO("Starting run_traj_control application ...");
        if(!ros::Time::waitForValid(ros::WallDuration(10.0)))
        {
            ROS_FATAL("Timed-out waiting for valid time.");
            return EXIT_FAILURE;
        }
        torso_control_client_Ptr TorsoClient;
        createTorsoClient(TorsoClient);

        control_msgs::FollowJointTrajectoryGoal torso_goal;
        waypoints_torso_goal(torso_goal, z);

        torso_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        TorsoClient->sendGoal(torso_goal);

        while(!(TorsoClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds
        }
        */
    }
    bool BarbotActions::move_gripper_joints(double gripper_left, double gripper_right)
    {
        std::string s1 = "rosrun play_motion move_joint gripper_left_finger_joint ";
        std::string s2 = " 1.0";
        std::string cmd = s1 + std::to_string(gripper_left) + s2;
        system(cmd.c_str());

        s1 = "rosrun play_motion move_joint gripper_right_finger_joint ";
        cmd = s1 + std::to_string(gripper_right) + s2;
        system(cmd.c_str());
        /*
        ROS_INFO("Starting run_traj_control application ...");
        if(!ros::Time::waitForValid(ros::WallDuration(10.0)))
        {
            ROS_FATAL("Timed-out waiting for valid time.");
            return EXIT_FAILURE;
        }
        gripper_control_client_Ptr GripperClient;
        createGripperClient(GripperClient);

        control_msgs::FollowJointTrajectoryGoal gripper_goal;
        waypoints_gripper_goal(gripper_goal, gripper_left, gripper_right);

        gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        GripperClient->sendGoal(gripper_goal);

        while(!(GripperClient->getState().isDone()) && ros::ok())
        {
            ros::Duration(4).sleep(); // sleep for four seconds
        }
        */
    }
    bool BarbotActions::make_drink(std::vector<int> ingredient_list, const aruco_msgs::MarkerArray& msg)
    {
        std::vector<int> markerID_list;
        for(int i; i<msg.markers.size();i++)
        {
            markerID_list.push_back(msg.markers[i].id);
        }

        for(int idx=0; idx<ingredient_list.size(); idx++)
        {
            auto it = find(markerID_list.begin(), markerID_list.end(), ingredient_list[idx]);
            if(it != markerID_list.end())
            {
                int markerIndex = it - markerID_list.begin();
                ROS_INFO_STREAM("Start fetching ingredient:" << ingredient_list[idx]);
                ROS_INFO_STREAM("The marker index is: " << markerIndex);
                ROS_INFO("Move to the object");
                move_acturator_to_cart(msg.markers[markerIndex].pose.pose.position.x-0.3, msg.markers[markerIndex].pose.pose.position.y, msg.markers[markerIndex].pose.pose.position.z, 1.57,0.0, 0.0);

                move_acturator_to_cart(msg.markers[markerIndex].pose.pose.position.x-0.2, msg.markers[markerIndex].pose.pose.position.y, msg.markers[markerIndex].pose.pose.position.z, 1.57,0.0, 0.0);
                move_gripper_joints(0.025, 0.020);
                if(idx==0)
                    move_gripper_joints(0.04, 0.04);
                
                ROS_INFO("Move to ready position");
                move_acturator_to_cart(msg.markers[markerIndex].pose.pose.position.x-0.3, msg.markers[markerIndex].pose.pose.position.y, msg.markers[markerIndex].pose.pose.position.z, 1.57,0.0, 0.0);

                // ros::Duration(2.0).sleep();

                // move_acturator_to_cart(msg.markers[markerIndex].pose.pose.position.x- 0.3, msg.markers[markerIndex].pose.pose.position.y, msg.markers[markerIndex].pose.pose.position.z, 1.57,0.0, 0.0);
                
                // ros::Duration(2.0).sleep();
            }
        }

        ROS_INFO("Drink is made");
    }
    void BarbotActions::standy()
    {
        d2sconvert(jointParams_init, jointParams_);
        move_arm_joints(jointParams_);
        move_torso_joints(0.15);
        // move_head_joints(0.0, 0.0);
        move_gripper_joints(0.04, 0.04);
    }
    void BarbotActions::putdown()
    {
        move_acturator_to_cart(0.33, 0.2, 0.75, 1.57, 0.0, 0.0); // place onto the dinner table
        move_gripper_joints(0.04, 0.04);
        move_acturator_to_cart(0.30, 0.2, 0.75, 1.57, 0.0, 0.0); // place onto the dinner table
    }
    void BarbotActions::deliver_to_table(int table_number)
    {
        switch (table_number)
        {
        case /* constant-expression */1:
            /* code */
            move_to_point(table1_loc);
            break;
        case /* constant-expression */2:
            /* code */
            move_to_point(table2_loc);
            break;
        case /* constant-expression */3:
            /* code */
            move_to_point(table3_loc);
            break;
        case /* constant-expression */4:
            /* code */
            move_to_point(table4_loc);
            break;
        }
        
    }
    /********************/
    /**Private functions*/
    /********************/
    void BarbotActions::createArmClient(arm_control_client_Ptr& actionClient)
    {
        ROS_INFO("Creating action client to arm controller ...");

        actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory")); // setup the topic name

        int iterations = 0, max_iterations = 3;
        
        // wait for the arm controller action server to come up
        while(!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
        {
            ROS_DEBUG("Waiting for the arm_controller_action server to come up");
            ++iterations;
        }

        if( iterations == max_iterations)
            throw std::runtime_error("Error in createArmClient: arm controller action server not availalbe");   
    }

    void BarbotActions::waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, jointParams params)
    {
        goal.trajectory.joint_names.push_back("arm_1_joint");
        goal.trajectory.joint_names.push_back("arm_2_joint");
        goal.trajectory.joint_names.push_back("arm_3_joint");
        goal.trajectory.joint_names.push_back("arm_4_joint");
        goal.trajectory.joint_names.push_back("arm_5_joint");
        goal.trajectory.joint_names.push_back("arm_6_joint");
        goal.trajectory.joint_names.push_back("arm_7_joint");

        goal.trajectory.points.resize(1);

        int index = 0;
        goal.trajectory.points[index].positions.resize(7);
        goal.trajectory.points[index].positions[0] = params.arm_joint_1;
        goal.trajectory.points[index].positions[1] = params.arm_joint_2;
        goal.trajectory.points[index].positions[2] = params.arm_joint_3;
        goal.trajectory.points[index].positions[3] = params.arm_joint_4;
        goal.trajectory.points[index].positions[4] = params.arm_joint_5;
        goal.trajectory.points[index].positions[5] = params.arm_joint_6;
        goal.trajectory.points[index].positions[6] = params.arm_joint_7;
        // Velocities
        goal.trajectory.points[index].velocities.resize(7);
        for (int j = 0; j < 7; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 1.0;
        }
        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
    }

    void BarbotActions::createHeadClient(head_control_client_Ptr& actionClient)
    {
        ROS_INFO("Creating action client to head controller ...");

        actionClient.reset( new head_control_client("/head_controller/follow_joint_trajectory") );

        int iterations = 0, max_iterations = 3;
        // Wait for head controller action server to come up
        while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
        {
            ROS_DEBUG("Waiting for the head_controller_action server to come up");
            ++iterations;
        }

        if ( iterations == max_iterations )
            throw std::runtime_error("Error in createheadClient: head controller action server not available");
    }

    void BarbotActions::waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal, double head_1_joint, double head_2_joint)
    {
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");

        goal.trajectory.points.resize(1);

        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = head_1_joint;
        goal.trajectory.points[index].positions[1] = head_2_joint;

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 1.0;
        }
        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
    }

    void BarbotActions::createTorsoClient(torso_control_client_Ptr& actionClient)
    {
        ROS_INFO("Creating action client to torso controller ...");

        actionClient.reset( new torso_control_client("/torso_controller/follow_joint_trajectory") );

        int iterations = 0, max_iterations = 3;
        // Wait for head controller action server to come up
        while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
        {
            ROS_DEBUG("Waiting for the torso_controller_action server to come up");
            ++iterations;
        }

        if ( iterations == max_iterations )
            throw std::runtime_error("Error in createheadClient: torso controller action server not available");
    }

    void BarbotActions::waypoints_torso_goal(control_msgs::FollowJointTrajectoryGoal& goal, double z)
    {
        goal.trajectory.joint_names.push_back("torso_lift_joint");

        goal.trajectory.points.resize(1);

        int index = 0;
        goal.trajectory.points[index].positions.resize(1);
        goal.trajectory.points[index].positions[0] = z;

        // Velocities
        goal.trajectory.points[index].velocities.resize(1);
        for (int j = 0; j < 1; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 1.0;
        }
        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
    }

    void BarbotActions::createGripperClient(gripper_control_client_Ptr& actionClient)
    {
        ROS_INFO("Creating action client to gripper controller ...");

        actionClient.reset( new head_control_client("/gripper_controller/follow_joint_trajectory") );

        int iterations = 0, max_iterations = 3;
        // Wait for head controller action server to come up
        while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
        {
            ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
            ++iterations;
        }

        if ( iterations == max_iterations )
            throw std::runtime_error("Error in createheadClient: gripper controller action server not available");
    }

    void BarbotActions::waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal, double gripper_left, double gripper_right)
    {
        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

        goal.trajectory.points.resize(1);

        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = gripper_left;
        goal.trajectory.points[index].positions[1] = gripper_right;

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 1.0;
        }
        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
    }

}