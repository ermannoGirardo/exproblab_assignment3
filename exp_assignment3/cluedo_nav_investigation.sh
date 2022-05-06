#!/bin/bash

gnome-terminal --tab --title="sherlock_pkg" -- bash -c "roslaunch sherlock_assignment3 demo_gazebo.launch 2</dev/null"
gnome-terminal --tab --title="services" -- bash -c "sleep 5; roslaunch exp_assignment3 services.launch 2</dev/null"
gnome-terminal --tab --title="cluedo_FSM" -- bash -c "sleep 10; rosrun exp_assignment3 cluedo_investigation_FSM.py"
