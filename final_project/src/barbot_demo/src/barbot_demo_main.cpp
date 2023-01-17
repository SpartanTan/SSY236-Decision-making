#include "ros/ros.h"
#include "barbot_brain/BarbotBrain.hpp"
#include "barbot_brain/getIngredients.h"
#include "barbot_brain/drink.h"
#include "barbot_move/barbot_move.hpp"
#include <aruco_msgs/MarkerArray.h>
// #include <string>
// #include <vector>

namespace barbot
{
class BarbotService
{
public:
    BarbotService() : brain(false)
    { 
        timer_ = nh_.createTimer(ros::Duration(0.5), &BarbotService::timerCallback, this);
        service_ = nh_.advertiseService("Barbot_service", &BarbotService::serve, this);
        markersub_ = nh_.subscribe("/aruco_marker_publisher/markers", 10, &BarbotService::markerCallback, this);

        actions.standy();
        actions.move_to_point(bar_loc);

        ROS_INFO("Init success");
    }
    
private:
    void timerCallback(const ros::TimerEvent& event);
    bool serve(barbot_brain::drink::Request &req, barbot_brain::drink::Response &res);
    void markerCallback(const aruco_msgs::MarkerArray& msg);

    ros::NodeHandle nh_;
    barbot::BarbotBrain brain;
    barbot::BarbotActions actions;
    ros::ServiceServer service_;
    ros::Subscriber markersub_;
    ros::Timer timer_;
    int servicetimeout_=0;
    
    jointParams jointParams_ = jointParams();
    headParams headParams_ = headParams();

    std::vector<int> ingredient_list;
    bool marker_pos_setting = false;
    aruco_msgs::MarkerArray markerArray;
    // markerPos marker_
};
}

bool barbot::BarbotService::serve(barbot_brain::drink::Request &req,
                        barbot_brain::drink::Response &res)
{
    std::string type = req.type.c_str();
    int table_number = req.table_number;
    //**********************************//
    //***** Stage 1: Brain process *****//
    //**********************************//
    // - Get ingredient list
    ROS_INFO("Stage-1: Get ingredients");
    
    ingredient_list = brain.get_ingredient_ids(type);
    for(auto i:ingredient_list)
    {
        ROS_INFO_STREAM("I have to pick ingredients with ID: " << i);
    }
    if(ingredient_list.size() != 2)
        ROS_ERROR("No ingredients");
    actions.move_head_joints(0.0, -0.7);
    // For simple testing 
    // ingredient_list.push_back(0);
    // ingredient_list.push_back(582);
    marker_pos_setting = false;
    while(!marker_pos_setting);
    ROS_INFO("marker_pos is set");
    // wait for the aruco
    // ros::Duration(2.0).sleep();
    // Test zone//
    // actions.move_gripper_joints(0.03, 0.03);
    // actions.move_gripper_joints(0.0, 0.0);
    // ROS_INFO("Gripper done");

    // actions.d2sconvert(jointParams_ready, jointParams_);
    // actions.move_arm_joints(jointParams_);
    // actions.move_torso_joints(0.20);

    // for(int i=0; i < markerArray.markers.size(); i++)
    // {
    //     ROS_INFO_STREAM("Object " << i+1 << " ID: " << markerArray.markers[i].id);
    // }    //***** Stage 1  Finished *****//

    ROS_INFO("Stage1 is done");
    // if(type == "Cola" | type == "cola")
    //     actions.move_acturator_to_cart(0.35,0.2,0.5,0.0,1.57,0.0); // pointing down position
    // ROS_INFO("joint_5_is %lf", jointParams_.arm_joint_5);

    //////////////////////////////////////

    //**********************************//
    //***** Stage 2: Standby *****//
    //**********************************//
    // - Preparation for aruco and fetching
    // actions.standy();

    //**********************************//
    //***** Stage 3: Make drink *****//
    //**********************************//
    ROS_INFO("Stage-3: Make drink");
    if(markerArray.markers.size()!=6)
        ROS_ERROR("markerArray is empty");
    actions.make_drink(ingredient_list, markerArray);
    actions.move_head_joints(0.0, 0.0);
    //**********************************//
    //***** Stage 4: Move to table *****//
    //**********************************//

    ROS_INFO("Stage-4: delivering");
    // actions.move_to_point(table_1);
    actions.deliver_to_table(table_number);
    ROS_INFO("Devliver done");

    //**********************************//
    //***** Stage 5: Put down the drink *****//
    //**********************************//
    ROS_INFO("Stage-5: Put down the drink");
    actions.putdown();
    actions.standy();

    //**********************************//
    //***** Stage : Return to starting position *****//
    //**********************************//
    ROS_INFO("Stage-: Return to starting pos");
    actions.move_to_point(bar_loc); 
    

    // clear the flags
    ingredient_list.clear();
    ROS_INFO("Service is done");
    return true;
}

void barbot::BarbotService::timerCallback(const ros::TimerEvent& event)
{
    servicetimeout_ += 1;
    // ROS_INFO("timerout: %d", servicetimeout_);
}

void barbot::BarbotService::markerCallback(const aruco_msgs::MarkerArray& msg)
{
    
    if(ingredient_list.size()!=0 && !marker_pos_setting)
    {
        if(msg.markers.size()==6)
        {
            markerArray = msg;  
            marker_pos_setting = true;
            ROS_INFO("Get marker positions!!");    
        }
        else
        {
            ROS_ERROR("Can't see all the markers!!");
        }
    } 
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "barbot_service");
    ros::NodeHandle nl;
    ros::MultiThreadedSpinner spinner(4);
    // barbot_brain::getIngredients::Request req;
    // req.drink_name = "tset";
    // ROS_INFO("Demo started %s", req.drink_name.c_str());
    
    barbot::BarbotService barbot_srv;
    // ros::spin();
    // ros::waitForShutdown();
    spinner.spin();

    return 0;
}