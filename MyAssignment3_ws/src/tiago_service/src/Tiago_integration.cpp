#include "ros/ros.h"
#include "myservice/drink.h"
#include "tiago_service/fetch_drink.h"
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
        service_ = nh_.advertiseService("Tiago_fetch_drink", &Tiago_service::fetch_drink_service, this);
        // marksersub_t.create<aruco_msgs::MarkerArray>("/aruco_marker_publisher/markers", 100, boost::bind(&Tiago_service::markerCallback, this, _1));
        markersub_ = nh_.subscribe("/aruco_marker_publisher/markers", 10, &Tiago_service::markerCallback, this);
        // ROS_INFO("Timer create");
        // timer_ = nh_.createTimer(ros::Duration(0.5), &Tiago_service::timerCallback, this);
        testnum_ = 1;
    }

private:
    bool fetch_drink_service(tiago_service::fetch_drink::Request &req,
                        tiago_service::fetch_drink::Response &res);
    void timerCallback(const ros::TimerEvent& event);
    void markerCallback(const aruco_msgs::MarkerArray& msg);
    bool move_to_destination();
    bool Tiago_standby();
    bool lookdown();
    bool start_aruco();
    void ready_to_fetch();
    void fetch(double x, double y, double z);

    bool fetch_id(std::string type);


    ros::ServiceServer service_;
    ros::SubscribeOptions marksersub_t;
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Subscriber markersub_;
    
    int timeout_ = 0;
    int testnum_ = 0;
    int desired_id = 0;

    // flags
    bool both_marker_detected = false;
    bool marker_pos_setting = false;
    bool ready_to_fetch_flg = false;

    

    Pos marker_pos;
    Pos actuator_pos;

};

bool Tiago_service::Tiago_standby()
{
    // system("rosrun tiago_service Tiago_move_cartesian 0.35 0.2 0.5 0.0 1.57 0.0"); // move the arm to standby position
    lookdown();
    ROS_INFO("Tiago standby");
}

bool Tiago_service::lookdown()
{
    system("rosrun tiago_service move_head 0.2 -1.0");   
}

bool Tiago_service::start_aruco()
{
    // This is not the correct way. The function will block the node
    // Try launch the detector in launch file.
    // system("roslaunch tiago_service markerSize:=");
}

void Tiago_service::ready_to_fetch()
{
    system("rosrun tiago_service Tiago_move_cartesian 0.35 0.2 0.5 1.57 0.0 0.0");
    ROS_INFO("Ready to fetch");
}

void Tiago_service::fetch(double x, double y, double z)
{
    // fetch the drink by the given coordinate
    double x_rel = x - 0.3;
    std::string s1 = "rosrun tiago_service Tiago_move_cartesian ";
    std::string s2 = " 1.57 0.0 0.0";
    std::string cmd = s1 + std::to_string(x_rel) + ' ' + std::to_string(y) + ' ' + std::to_string(z) + s2;
    ROS_INFO("command is: %s", cmd.c_str());
    // system(cmd.c_str());
    system("rosrun tiago_service Tiago_move_cartesian 0.4 -0.05 0.5 1.57 0.0 0.0");
    ROS_INFO("Fetch complete");
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
    int itr = 0;
    if((msg.markers.size() == 2) && !marker_pos_setting && ready_to_fetch_flg)
    {
        if(itr%100)
        {
            ROS_INFO_STREAM("length: " << msg.markers.size());
            itr = 0;
        }
        itr++;
        if(desired_id == msg.markers[0].id)
        {
            selected_marker_index = 0;
            marker_pos.position.x = msg.markers[0].pose.pose.position.x;
            marker_pos.position.y = msg.markers[0].pose.pose.position.y;
            marker_pos.position.z = msg.markers[0].pose.pose.position.z;
            marker_pos_setting = true; 
        }       
        else if (desired_id == msg.markers[1].id)
        {
            selected_marker_index = 1;
            marker_pos.position.x = msg.markers[1].pose.pose.position.x;
            marker_pos.position.y = msg.markers[1].pose.pose.position.y;
            marker_pos.position.z = msg.markers[1].pose.pose.position.z;
            marker_pos_setting = true; 
        } 
        else
        {
            marker_pos_setting = false;
            ROS_ERROR("Markers are not 0 or 1");
        }
                   
          
    }  
}
bool Tiago_service::fetch_drink_service(tiago_service::fetch_drink::Request &req,
                        tiago_service::fetch_drink::Response &res)
{
    
    std::string type = req.type.c_str();
    ROS_INFO("type: %s", type.c_str()); 
    fetch_id(type);
    res.id = std::to_string(desired_id);

    ros::Duration(5.0).sleep();
    move_to_destination();
    
    ready_to_fetch_flg = true;
    ready_to_fetch();
    
    
    // sleep for a while for aruco to fetch the marker position
    // for(int trial = 0; trial < 5; ++trial)
    // {
    //     if(marker_pos_setting)
    //     {
    //         ROS_INFO_STREAM("Sending the marker position x: " << marker_pos.position.x << ", y: " << marker_pos.position.y << ", z: " << marker_pos.position.z);
    //         break;
    //     }    
    //     else
    //         ROS_INFO("Marker not found, service is finished");
    //     ROS_INFO("Is trying to find the markers");
    //     ros::Duration(2.0).sleep();
    // }
    if(marker_pos_setting)
    {   
        // ROS_INFO_STREAM("hello");
        // ROS_INFO_STREAM("Sending the marker position x: " << marker_pos.position.x << ", y: " << marker_pos.position.y << ", z: " << marker_pos.position.z);
        res.drink_pose.position.x = marker_pos.position.x;
        res.drink_pose.position.y = marker_pos.position.y;
        res.drink_pose.position.z = marker_pos.position.z;

        res.drink_pose.orientation.x = marker_pos.orientation.x;
        res.drink_pose.orientation.y = marker_pos.orientation.y;
        res.drink_pose.orientation.z = marker_pos.orientation.z;
        res.drink_pose.orientation.w = marker_pos.orientation.w;

    }   
    ROS_INFO("Start fetching");
    fetch(res.drink_pose.position.x, res.drink_pose.position.y, res.drink_pose.position.z);


    ROS_INFO("ROS service done");
}

// rosrun tiago_service move_to_destination
bool Tiago_service::move_to_destination()
{
    system("rosrun tiago_service move_to_destination");
}

bool Tiago_service::fetch_id(std::string type)
{
    std::string id;
    PrologClient pl = PrologClient("/rosprolog", true);
    std::string que = "getID('" + type + "',ID)";

    // PrologQuery bdgs = pl.query("owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)");
    PrologQuery bdgs1 = pl.query(que.c_str());
    for(PrologQuery::iterator it=bdgs1.begin(); it != bdgs1.end(); it++)
    {
        PrologBindings bdg = *it;
        // std::cout << bdg["ID"] << std::endl;
        id = bdg["ID"].toString();
    }
    desired_id = stoi(id);

    ROS_INFO_STREAM("desired_id: " << desired_id);
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