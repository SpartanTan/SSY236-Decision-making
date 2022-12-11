# start docker and login
pal_docker -it -d --name=tiago registry.git.chalmers.se/phric/general_info/lab_equipment/tiago_docs/chalmers_tiago101:latest
docker exec -it -w /home/user/ -u user tiago bash

# To use knowrob
source tiago_public_ws/devel/setup.bash
sudo apt-get install swi-prolog
sudo apt-get install libjson-glib-dev
sudo apt-get install libmongoc-1.0-0
sudo  apt-get install libmongoc-dev
sudo apt install python-pip
pip install rdflib
##################
now it is replaced with 
./install.sh
##################


source /opt/ros/melodic/setup.bash
source exchange/tiago_docker/tiago_public_ws/devel/setup.bash
source exchange/tiago_docker/knowrob_melodic/devel/setup.bash
source exchange/tiago_docker/SSY236-Decision-making/assignment03_ws/devel/setup.bash
# in knowrob package
# compile the exact pkg
catkin_make --only-pkg-with-deps genowl
#compile the remaning pkgs
catkin_make -DCATKIN_WHITELIST_PACKAGES=""


export GAZEBO_MODEL_PATH=/home/user/exchange/tiago_docker/assignment03_ws/src/tiago_service/models/:/home/user/exchange/tiago_docker/tiago_public_ws/src/tiago_simulation/tiago_gazebo/models

export GAZEBO_RESOURCE_PATH=/home/user/exchange/tiago_docker/assignment03_ws/src/tiago_service

# Assignment3 after source test
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel

# After setting environment variables test
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true robot:=steel world:=A03 map:=/home/user/exchange/tiago_docker/assignment03_ws/src/tiago_service/living_room_map

# detect the aruco
roslaunch tiago_aruco_demo detector.launch markerId:=<marker_ID> markerSize:=0.045

# For final project
export GAZEBO_MODEL_PATH=/home/user/exchange/tiago_docker/ssy236_g02/src/barbot_demo/models/:/home/user/exchange/tiago_docker/tiago_public_ws/src/tiago_simulation/tiago_gazebo/models
export GAZEBO_RESOURCE_PATH=/home/user/exchange/tiago_docker/ssy236_g02/src/barbot_demo


