#include "barbot_brain/BarbotBrain.hpp"


namespace barbot
{
    BarbotBrain::BarbotBrain(bool as_service)
    {   if(as_service)
        {
            brainService_ = nh_.advertiseService("getIngredients", &BarbotBrain::fetch_id_n_class, this);
            ROS_INFO("service start");
        }
        else
        {
            ROS_INFO("Brain starts not as a service");
        }      
    }

    bool BarbotBrain::callback(barbot_brain::getIngredients::Request &req,barbot_brain::getIngredients::Response &res)
    {
        ROS_INFO_STREAM("drink name is: " << req.drink_name.c_str());
        res.Ingredient_list.push_back(3);
        res.Ingredient_list.push_back(2);
        ROS_INFO_STREAM("Ingredient #1 is:" << std::to_string(res.Ingredient_list.at(0)));
        ROS_INFO_STREAM("Ingredient #2 is:" << std::to_string(res.Ingredient_list.at(1)));
    }

    bool BarbotBrain::fetch_id_n_class(barbot_brain::drink::Request &req,
                        barbot_brain::drink::Response &res)
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

    std::vector<int> BarbotBrain::get_ingredient_ids(std::string type)
    {
        std::vector<int> ingredient_list;
        PrologClient pl = PrologClient("/rosprolog", true);
        std::string que = "getIngredientIDs('" + type + "',ID)";

        PrologQuery bdgs1 = pl.query(que.c_str());
        for(PrologQuery::iterator it=bdgs1.begin(); it != bdgs1.end(); it++)
        {
            PrologBindings bdg = *it;
            // std::cout << stoi(bdg["ID"].toString()) << std::endl;
            ingredient_list.push_back(stoi(bdg["ID"].toString())); // conversion forom prologvalue to string to int
        }

        return ingredient_list;
    }

    void BarbotBrain::test_brain(std::string str)
    {
        ROS_INFO("I received %s", str.c_str());
    }

}