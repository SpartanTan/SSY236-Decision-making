# ssy236_g02


## Group Information
Group nr: 02

Students: Zhicun Tan, VDV

Full Name : zhicun@student.chalmers.se

## User guide
Git clone this workspace in parallel with /tiago_public_ws and /knowrob_melodic

Source everthing in every terminal. I suggest you open more than 4 terminals together.

    source /opt/ros/melodic/setup.bash
    source exchange/tiago_docker/tiago_public_ws/devel/setup.bash
    source exchange/tiago_docker/knowrob_melodic/devel/setup.bash
    source exchange/tiago_docker/ssy236_g02/devel/setup.bash

I expect you have compile the knowrob and tiago_public packages.

Change directory to ssy236_g02 workspace, compile this workspace 

    catkin_make


In the terminal where the demo(Gazebo, Rviz, etc) starts, type:

    export GAZEBO_MODEL_PATH=/home/user/exchange/tiago_docker/ssy236_g02/src/barbot_demo/models/:/home/user/exchange/tiago_docker/tiago_public_ws/src/tiago_simulation/tiago_gazebo/models

    export GAZEBO_RESOURCE_PATH=/home/user/exchange/tiago_docker/ssy236_g02/src/barbot_demo

Launch the simulator with two Rviz interfaces. 

    roslaunch barbot_demo barbot_demo.launch

After the robot sets its arms to default position(the very default position every time when launching the Gazebo), run the following code in another terminal.(you may need to source the packages first).

    rosrun barbot_demo barbot_demo

Then you will see the robot starts moving towards the bar and going into standby state. Wait for the terminal prints

    Init success

before next step.

After seeing "Init success", open another terminal and then type

    rosservice call /Barbot_service "Vanilla_coke_M" 1

Vanilla_coke_M is the name of the drink you want, 1 is the nunmber of the table(There are four tables in total, so any integer between 1 to 4 is acceptable).

All the names of the drinks will be listed below.
- Lime_beer_M
- Vanilla_coke_M
- Lime_fanta_M
- Lime_Vodka_M
- Vanilla_beer_M
- Vanilla_coke_M
- Vanilla_fanta_M
- Vanilla_vodka_M

Tiago will try to reach two ingredients then grab the last one towards the given table.

Enjoy the drink, if there's nothing going wrong :)

Before the service ends, Tiago will go back to bar and wait for the next service call. Simply retype the rosservice command 
