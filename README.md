# # Experimental Robotics Lab Assignment 3:
## I. Cluedo investigation

![Cluedo_Map](https://user-images.githubusercontent.com/48509825/161590960-7892a0c4-97c4-4b8c-9630-f4886926a472.jpg)

Figure 1: Cluedo map of the original game

## II. Description
In this repository you can find a solution of a Cluedo like investigation game played by Sherlock robot.
Sherlock has to collect hints moving in the environment in a completely autonomous way.
An hypothesis has the following structure: [ID, killer, killer_place, killer_weapon].
Killer is the name of the murder, killer_place is the name of the room in wich took place the murder
and finally killer_weapon is the name of the weapon used to complete the murder.
An hypothesis is complete if all the fields are filled, it is consistent if it is complete and only one name for each field is present.
In figure 2 and 3 you can see the simulation enviroment of Rviz and Gazebo respectively.
In the simulation environment are present 30 ArUco markers scattered in 6 different rooms, 5 in each room:
* room1 : (-4, -3)
* room2 : (-4, -2)
* room3 : (-4,  7)
* room4 : ( 5, -7)
* room5 : ( 5, -3)
* room6 : ( 5,  1)

ArUco markers are placed at different heigths: placed on the walls (height 1 m ca.), as you can see.
Once a marker is seen by Sherlock's cam, it provides its ID that will be managed by the oracle in order to provide an hint.
Once an hypothesis is consistent Sherlock has to go in the centre of the arena (x=0.0, y= 1.0, which should be also the starting position of the robot), in order to ask to the oracle if the collected hypothesisis the true one. If the hypothesis is wrong Sherlock has to collect other hints until the true hypothesis is found.
The game ends when the oracle validates Sherlock solution.
All the investigation behaviour is menaged via a smach FSM as used in the first assignment.
Sherlock is able to autonomously navigate in the sourrounding environment, thanks to its differential drive mobile base and opportunely tuned move_base & SLAM GMAPPING nodes.

![MapAss3](https://user-images.githubusercontent.com/48509825/167033061-827f5d9b-28bf-4cd9-ac57-6d41274e20f4.png)

Figure 1: Sherlock3 in the map


## III. SW Architecture
The assignment is composed by three main packages:

* **sherlock_assignment3**: is the pkg generated by moveit_setup_assistant starting from sherlock.xacro in the exp_assignment3/urdf.
* **exp_assignment3**: contains the oracle with its services, the urdf of the robot (xacro file, materials and gazebo plugins), the Gazebo map, a custom service and the cluedo investigation FSM.
* **aruco_ros** : it contains all the functionalities to detecting squared markers and also to estimate its position with respect to the camera (if opportunely calibrated).

### COMPONENT DIAGRAM



![Architecture](https://user-images.githubusercontent.com/48509825/167031589-22283940-e1c2-41ea-9fa4-18a376f002a2.png)

Figure 2: Component Diagram of the Architecture




## IV. All you need
### SERVICES
* **move_arm.py** is the node demanded to move Sherlock arm. Thanks to MoveIt! this node simply take the joints configuration wanted and then 
            planning the trajectory (in joint space) thanks to Sherlock MoveIt! pkg ad hoc developed. Service on /move_arm_service
* **final_oracle.cpp** The Oracle node (already implemented), provides an hint on the basis of the acquired ArUco marker ID. The Oracle knows also the final solution ID, that Sherlock has to collect during the game
*  **marker_publisher.cpp** is the node demanded to manage the input video acquired by Sherlock cam, in order to detect Markers and thanks to ArUco dictionary ( ARUCO_MIP_36h12) is possible to trace its ID. A publisher was added to shared the ID.

### FSM
* **cluedo_investigation_FSM.py** is the core of the architecture. This node menages all the investigation behaviour, initialization of the investigation, menagement of all the acquired hypotheses (ontology), Sherlock navigation and manipulation to scan the environment

### SMACH VIEWER FSM



Figure 3: Smach Viewer FSM of the cluedo investigation

### Launch Files
* sherlock_assignment3/launch/**demo_gazebo.launch** is demanded to execute all the moveit plugins and controllers for the arm and spawns Sherlock into the predefined Gazebo scene. It also executes the **simulation.launch** file already developed to execute the **simulation.cpp** file to source the Oracle node.
* exp_assignment3/launch/**navigation.launch** is demanded to source the parameter files for move_base pkg, implementing an autonomous navigation thanks to its planner. This launch file also executes the gmapping pkg, that provides laser-based SLAM (Simultaneous Localization and Mapping).
* exp_assignment3/launch/**services.launch** is demanded to execute the above lauch file, smach viewer, move_arm and the marker_publisher nodes. 

### Documentation
It is also present Doxygen documentation for exp_assignemnt3. In particular you can find docs folder with **index.html** file
 
### SEQUENCE DIAGRAM 



![Sequence](https://user-images.githubusercontent.com/48509825/167031691-3420e141-315c-4abc-abaa-834f3ae33295.png)

Figure 4: Sequence Diagram of the investigation

## V. How to run the simulation
* If you are not familiar with ROSPlan, I suggest you to see its GitHub page and download it --> [ROSPlan](https://github.com/KCL-Planning/ROSPlan).
* You have to clone this repository into your ros workspace:
```
git clone https://github.com/ermannoGirardo/exproblab_assignment3
```
* You have to compile and build the workspace
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
* If you want to execute one by one the launch files:
```
roslaunch sherlock_assignment3 demo_gazebo.launch
```
```
roslaunch exp_assignment3 services.launch
```

### Into your ws, go to the py file and mark it as exe
```
roscd exp_assignment3/scripts
```

```
chmod +x cluedo_investigation_FSM.py
```

### Wait a moment and then start the investigation
```
rosrun exp_assignment3 cluedo_investigation_FSM.py
```


_!!! Launch the files in this specific sequence!!!_
If you don't want to see all the warning messages you can add **2</dev/null** after each command.

* If you want there is also a bash file that execute all for you.
So, first you have to install gnome terminal, if you haven't yet:
  ```
  apt install gnome-terminal
  ```
  Then you have to modify the permit of the file, in order to make it an executable file:
  ```
  roscd exp_assignment3
  ```
  ```
  chmode +x cluedo_nav_investigation.sh
  ```
  And now you can execute the file:
  ```
  ./cluedo_nav_investigation
  ```
  
 ## VII Drawbacks and possible Adjustments
 * **slowness**: since the low performance of the robot (really basic), when an high linear acceleration along x axis is imposed, Sherlock 
                  rears up and LIDAR sensor detects inexistent obstacles. In order to avoid this unwanted behaviour (as much as possible) the acceleration and speed parameter for move base was setted very low, consequently Sherlock moves very slowly.
                  Also the manipulation is really low. The joint velocities for the arm are set as high as possible, under the breaking level of the motors. Maybe also the Docker image used to test the simulation has low performance and the speed of the simulation is slowed down.
* **decrease the scanning time**: in order to decrease the time for the scanning state (hint acquisition) a possible solution could be to create a room class in order to track the number of already acquired hint in the room. Once 5 differents markers are detected in the same room exit from the acquired hint state. Another possible solution could be to do not equip Sherlock with an arm but using two different cameras in order to see both the markers on the walls and on the floor.

## VIII About the Author
**Robotics Engineer** @Università degli Studi di Genova via Opera Pia. 
**Phone number:** 3451552733
**Email:** girardoermanno@gmail.com
**GitHub Page:** [Click Here](https://github.com/ermannoGirardo)



