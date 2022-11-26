#include "ros/ros.h"
#include "myservice/drink.h"
#include <rosprolog/rosprolog_client/PrologClient.h>



// test queries
// owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)
// owl_individual_of(I, drinkOntology:'Acoholic')

bool fetch_id_n_class(myservice::drink::Request &req,
                        myservice::drink::Response &res)
{
    res.id = "1";
    res.drink_class = "cola";
    ROS_INFO("Request: %s", req.type.c_str());
    ROS_INFO("ID is: %s", res.id.c_str());

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drink_server");
    ros::NodeHandle n;

    // ros::ServiceServer service = n.advertiseService("drink_id_n_class", fetch_id_n_class);
    PrologClient pl = PrologClient("/rosprolog", true);

    PrologQuery bdgs = pl.query("owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)");
    
    for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
      PrologBindings bdg = *it;
    //   std::cout << "Found solution: " << (bool)(it != bdgs.end()) << std::endl;
      std::cout << "I = "<< bdg["ID"] << std::endl;
    }

    // ROS_INFO("Ready to get drink ID and class, please input the type");
    // ros::spin();
    
    return 0;
}