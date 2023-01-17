#include "ros/ros.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include "barbot_brain/BarbotBrain.hpp"
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "barbot_brain");
    ros::NodeHandle nl;
    
    barbot::BarbotBrain brain(false);

    std::string str = "Lemon_beer_M";
    std::vector<int> ingrelist;

    ingrelist = brain.get_ingredient_ids(str);
    for(auto i:ingrelist)
        std::cout << i << std::endl;
    ros::spin();

    return 0;
}