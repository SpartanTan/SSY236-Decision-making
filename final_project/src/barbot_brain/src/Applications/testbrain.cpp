#include "ros/ros.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nl;

    ROS_INFO("Start node");
    PrologClient pl = PrologClient("/rosprolog", true);

    PrologQuery bdgs = pl.query("owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)");
    for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;
        std::cout << bdg["ID"] << std::endl;
    }
    ros::spin();

    return 0;
}