#include "ros/ros.h"
#include "myservice/drink.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <string>
#include <aruco_msgs/MarkerArray.h>


// test queries
// owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)
// owl_individual_of(I, drinkOntology:'Acoholic')
// owl_has(drinkOntology:'Corona', rdf:type, Class)

struct Pos
{
    struct position
    {
        double x;
        double y;
        double z;
    }position;
    
    struct orientation
    {
        double x;
        double y;
        double z;
        double w;
    }orientation; 
};


class Tiago_service
{
public:
    Tiago_service()
    {
        Tiago_standby();
        ROS_INFO("Tiago standby");
        struct Pos p;
        service_ = nh_.advertiseService("Tiago_fetch_drink", &Tiago_service::fetch_drink_service, this);
        markersub_ = nh_.subscribe("/aruco_marker_publisher/markers", 1000, &Tiago_service::markerCallback, this);
        // ROS_INFO("Timer create");
        // timer_ = nh_.createTimer(ros::Duration(0.5), &Tiago_service::timerCallback, this);
        testnum_ = 1;
    }

private:
    bool fetch_drink_service(myservice::drink::Request &req,
                        myservice::drink::Response &res);
    void timerCallback(const ros::TimerEvent& event);
    void markerCallback(const aruco_msgs::MarkerArray& msg);
    bool move_to_destination();
    bool Tiago_standby();
    bool lookdown();
    bool start_aruco();


    ros::ServiceServer service_;
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber markersub_;
    
    int timeout_ = 0;
    int testnum_ = 0;
    bool both_marker_detected = false;
    bool marker_pos_setting = false;
    int desired_id = 0;

    Pos marker_pos;
    Pos actuator_pos;

};

bool Tiago_service::Tiago_standby()
{
    system("rosrun tiago_service move_head 0.0 0.0");
}

bool Tiago_service::lookdown()
{
    system("rosrun tiago_service move_head 0.0 -0.7");   
}

bool Tiago_service::start_aruco()
{
    // This is not the correct way. The function will block the node
    // Try launch the detector in launch file.
    // system("roslaunch tiago_service markerSize:=");
}

void Tiago_service::timerCallback(const ros::TimerEvent& event)
{
    timeout_ += 1;
    // ROS_INFO("timerout: %d", timeout_);
    ROS_INFO("timerout");
}

void Tiago_service::markerCallback(const aruco_msgs::MarkerArray& msg)
{
    int selected_marker_index = 0;
    if(msg.markers.size() == 2 && !marker_pos_setting)
    {
        if(desired_id == msg.markers[0].id)
            selected_marker_index = 0;
        else if (desired_id == msg.markers[1].id)
            selected_marker_index = 1;
        marker_pos.position.x = msg.markers[selected_marker_index].pose.pose.position.x;
        marker_pos.position.y = msg.markers[selected_marker_index].pose.pose.position.x;
        marker_pos.position.z = msg.markers[selected_marker_index].pose.pose.position.x;

        marker_pos_setting = true;   
    }  
}
bool Tiago_service::fetch_drink_service(myservice::drink::Request &req,
                        myservice::drink::Response &res)
{
    res.id = "1";
    res.drink_class = "cola";
    std::string type = req.type.c_str();
    ROS_INFO("type: %s", type.c_str()); 
    move_to_destination();
    lookdown();
    // start_aruco();
    if(both_marker_detected)
        ;


    // ROS_INFO("Class initiate success: %d", testnum_);
}

// rosrun tiago_service move_to_destination
bool Tiago_service::move_to_destination()
{
    system("rosrun tiago_service move_to_destination");
}

bool fetch_id_n_class(myservice::drink::Request &req,
                        myservice::drink::Response &res)
{
    res.id = "1";
    res.drink_class = "cola";
    std::string type = req.type.c_str(); 

    PrologClient pl = PrologClient("/rosprolog", true);
    std::string que = "getID('" + type + "',ID)";

    // PrologQuery bdgs = pl.query("owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)");
    PrologQuery bdgs1 = pl.query(que.c_str());
    for(PrologQuery::iterator it=bdgs1.begin(); it != bdgs1.end(); it++)
    {
        PrologBindings bdg = *it;
        // std::cout << bdg["ID"] << std::endl;
        res.id = bdg["ID"].toString();
    }

    que = "getDirectClass('" + type + "', DirectClass)";
    ROS_INFO("que: %s", que.c_str());
    PrologQuery bdgs2 = pl.query(que.c_str());
    
    for(PrologQuery::iterator it=bdgs2.begin(); it != bdgs2.end(); it++)
    {
        PrologBindings bdg = *it;
        // std::cout << bdg["Class"] << std::endl;
        std::string drink_class_str = bdg["DirectClass"].toString();
        // ROS_INFO("drink_class is: %s", drink_class_str);
        std::size_t found = drink_class_str.find_last_of("#");
        
        res.drink_class = drink_class_str.substr(found+1);
    }
    ROS_INFO("Request: %s", req.type.c_str());
    ROS_INFO("ID is: %s", res.id.c_str());
    ROS_INFO("drink_class is: %s", res.drink_class.c_str());

    return true;
}

void callback(const ros::TimerEvent&)
{
    ROS_INFO("timer");
}

void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Tiago_service_node");
    ROS_INFO("Ready to get drink ID and class, please input the type.");
    // Tiago_service tiago_service;
    ros::NodeHandle n;
    ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
    // ros::Timer timer = nh.createTimer(ros::Duration(1), callback);

    ros::spin();
    
    return 0;
}