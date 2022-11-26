#include <iostream>
#include <string>
#include <rosprolog/rosprolog_client/PrologClient.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mytest_ros_prolog");
    PrologClient pl = PrologClient("/rosprolog", true);

    PrologQuery bdgs = pl.query("rdf_individual_of(I, ssy236Ontology:'Orange')");

    // // PrologQuery bdgs = pl.query("owl_individual_of(ssy236Ontology:'orange_2', ssy236Ontology:'Orange')");
    // bool res = false;
    // for(auto &it : bdgs){
    //     cout << "I1 = " << *it.begin() << endl;
    //     cout << "I2 = " << *it.end() << endl;
    // }
    // std::cout << res << std::endl;


    // PrologQuery bdgs = pl.query("member(A, [1, 2, 3, 4]), B = ['x', A], C = foo(bar, A, B)");

    for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
      PrologBindings bdg = *it;
      cout << "Found solution: " << (bool)(it != bdgs.end()) << endl;
      cout << "I = "<< bdg["I"] << endl;
    }

    return 0;
}