#include "ros/ros.h"
#include "myservice/drink.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <string>
#include <aruco_msgs/MarkerArray.h>


class Tiago_service
{
public:
    Tiago_service()
    {
        service_ = nh_.advertiseService("Tiago_fetch_drink", &Tiago_service::drink_service, this);
        // marksersub_t.create<aruco_msgs::MarkerArray>("/aruco_marker_publisher/markers", 100, boost::bind(&Tiago_service::markerCallback, this, _1));
        markersub_ = nh_.subscribe("/aruco_marker_publisher/markers", 1000, &Tiago_service::Callback, this);
        // ROS_INFO("Timer create");
        // timer_ = nh_.createTimer(ros::Duration(0.5), &Tiago_service::timerCallback, this);
    }

private:
    bool drink_service(myservice::drink::Request &req,
                        myservice::drink::Response &res);
    void Callback(const aruco_msgs::MarkerArray& msg);

    ros::ServiceServer service_;
    ros::SubscribeOptions marksersub_t;
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber markersub_;


};

bool Tiago_service::drink_service(myservice::drink::Request &req,
                        myservice::drink::Response &res)
{
    res.id = "1";
    res.drink_class = "cola";
    for(int trial = 0; trial < 20; ++trial)
    {
        ros::Duration(2.0).sleep();
        ROS_INFO("is sleeping");
    }
}
void Tiago_service::Callback(const aruco_msgs::MarkerArray& msg)
{
    int itr = 0;
    if((msg.markers.size() == 2))// && !marker_pos_setting)// && ready_to_fetch_flg)
    {
        if(!(itr%1000))
        {
            ROS_INFO_STREAM("length: " << msg.markers.size());
            itr = 0;
        }
        itr++;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Tiago_service_node");
    ros::MultiThreadedSpinner spinner(4);
    ROS_INFO("Ready to get drink ID and class, please input the type.");
    Tiago_service tiago_service;
    ros::NodeHandle n;
    // ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
    // ros::Timer timer = nh.createTimer(ros::Duration(1), callback);

    // ros::spin();
    spinner.spin();
    
    return 0;
}