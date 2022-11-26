#include "ros/ros.h"
#include "myservice/drink.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <string>


// test queries
// owl_has(drinkOntology:'Corona', drinkOntology:'hasId', ID)
// owl_individual_of(I, drinkOntology:'Acoholic')
// owl_has(drinkOntology:'Corona', rdf:type, Class)

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drink_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("drink_id_n_class", fetch_id_n_class);
    // PrologClient pl = PrologClient("/rosprolog", true);

    // PrologQuery bdgs = pl.query("rdf_has(drinkOntology:'Corona', rdf:type, Class)");
    
    // for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    // {
    //   PrologBindings bdg = *it;
    // //   std::cout << "Found solution: " << (bool)(it != bdgs.end()) << std::endl;
    // //   std::cout << "I = "<< bdg["ID"] << std::endl;
    // std::cout << bdg["Class"].toString() << std::endl;
    // }   

    ROS_INFO("Ready to get drink ID and class, please input the type");
    ros::spin();
    
    return 0;
}