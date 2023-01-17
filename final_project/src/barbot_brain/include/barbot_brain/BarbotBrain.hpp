#ifndef BARBOT_BRAIN
#define BARBOT_BRAIN

#include "ros/ros.h"
#include <rosprolog/rosprolog_client/PrologClient.h>
#include "barbot_brain/getIngredients.h"
#include "barbot_brain/drink.h"
#include <string>
#include <vector>

namespace barbot
{
class BarbotBrain
{
public:
    // constructor
    BarbotBrain(bool as_service);

    // test function
    bool testquery(const std::string& type);

    bool callback(barbot_brain::getIngredients::Request &req,
                barbot_brain::getIngredients::Response &res);

    bool fetch_id_n_class(barbot_brain::drink::Request &req,
                        barbot_brain::drink::Response &res);
    void test_brain(std::string str);
    std::vector<int> get_ingredient_ids(std::string type);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer brainService_;
};
}

#endif