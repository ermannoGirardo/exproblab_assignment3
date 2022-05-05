#!/usr/bin/env python

## @package exp_assignment3
#   \file cluedo_investigation_FSM.py
#   \brief This node provides the finite state machine
#   \author Ermanno Girardo
#   \version 1.0
#   \date 5/5/2022
#
#   \details
#
#   Clients : <BR>
#        /move_arm_service
#
#        /oracle_hint
#
#        /oracle_solution
# 
#   Subscribers : <BR>
#        /odom
#		  /acquired_hint
#        
#        
#   Publishers : <BR>
#        /cmd_vel
#
#   Actions : <BR>
#        /MoveBaseAction
#
#
# Description: 
#   
# This node is the core of the architecture. It is based on a smach FSM that menage all behaviour of the cluedo investigation.
# In particular it first initializes the investigation, extending Sherlock's arm thanks to move arm service.
# Thanks to move base action and SLAM_GMAPPING it menages Sherlock navigation between rooms.
# Ad Hoc class for acquiring hints and manage the hypotheses is used.
# This node also takes the ID of detected marker and send it to the oracle that will respond with an hint message.
# Once an hypothesis is completed and consistent Sherlock has to go in the centre of the arena in order to tell the
# acquired hypothesis. If the hypothesis is the true one the investigation is finished otherwise Sherlock come back to travel
# between rooms. 
#
#  


import roslib
import rospy
import smach
import smach_ros
import time
import random
from geometry_msgs.msg import *
import math
from exp_assignment3.srv import *
from std_msgs.msg import *
from nav_msgs.msg import *
from move_base_msgs.msg import *
import actionlib
from tf import transformations
from erl2.srv import *

##GLOBAL VARIABLES

move_arm_client = None
sherlock_pos = Point()
room1 = Point()
room1.x = -4
room1.y = -3
room2 = Point()
room2.x = -4
room2.y = 2
room3 = Point()
room3.x = -4 
room3.y = 7
room4 = Point()
room4.x = 5
room4.y = -7
room5 = Point()
room5.x = 5
room5.y = -3
room6 = Point()
room6.x = 5
room6.y = 1
rooms = [room1,room2,room3,room4,room5,room6]
room_counter = 0
centre_arena = Point()
centre_arena.x = 0
centre_arena.y = -1
cmd_vel_publisher = None
all_room_visited = False
hint_gen_client = None
oracle_solution_client = None
number_acquired_hint = 0
previous_marker = None
first_hint = True
hypotheses = []
already_acquired_hp= [False,False,False,False,False,False]
complete_consistent_hp =     [False,False,False,False,False,False]

########Hypotheses class##########################
																									   
class Hypotheses:																		   
	""" 																							   
	This class is basically the ontology for the investigation.
	In particular this class contains all the hypotheses collected
	by Sherlock during the game. 
	"""
	def __init__(self):
		#Class initialization      
		self.killer = []
		self.killer_weapon = []
		self.killer_location = []
		self.ID=-1

##############################################

######################FUNCTIONS#################

def euler_dist(point_1,point_2):
	"""
	This function compute the euler distance between two points
	"""
	distance = math.sqrt((point_2.x - point_1.x)**2 + (point_2.y - point_1.y) **2)
	return distance
	


def move_sherlock_arm(j1,j2,j3,j4,j5):
	"""
	This function is used to move Sherlock arm thanks to move_arm_service
	In particular it instanciate the request of the service and simply fill
	all the values passed as params
	"""
	arm_goal= MoveArmRequest()
	arm_goal.joint0 = j1
	arm_goal.joint1 = j2
	arm_goal.joint2 = j3
	arm_goal.joint3 = j4
	arm_goal.joint4 = j5

	res = move_arm_client(arm_goal)
	if res:
		print("Arm motion request accepted")

def odom_clbk(odom_msg):
	"""
	The odom callback is used to update Sherlock position.
	In particular the x and y position of Sherlock are updated 
	into local variables visible to all modules
	"""
	global sherlock_pos
	sherlock_pos = odom_msg.pose.pose.position
	
def acquired_hint_clbk(markerId):
	"""
	This function manages the marker id, once it is received.
	In particular the markerID is passed to the '/oracle_hint' service,
	that reply with the hint structure filled (ID, key and value).
	Once the data of the hint are available update the ontology.
	"""
	global hypotheses, hint_gen_client, previous_marker, first_hint
	#Call the oracle passing the ID of the marker
	#In order to obtain the hint associated
	response = hint_gen_client(markerId.data)
	acquired_hint = response.oracle_hint
	is_double=False
	j=0
	#Check if the hint value is valid
	if(acquired_hint.value!='-1'):
		#Check the type of the hint
		if(acquired_hint.key=='who'):
			#Run on all the hypotheses
			while ((j<len(hypotheses[acquired_hint.ID].killer)) and (is_double==False)):
				#Check if the hint is already perceived
				if(acquired_hint.value==hypotheses[acquired_hint.ID].killer[j]):
					#If the hint is already perceived remove it
					hypotheses[acquired_hint.ID].killer.remove(acquired_hint.value)
					#The hint is already perceived
					is_double=True
				#update the counter
				j = j + 1
			#Append the new hint
			hypotheses[acquired_hint.ID].killer.append(acquired_hint.value)

		is_double=False
		j=0   
		#Check the type of the hint
		if(acquired_hint.key=='where'):
			#Run on all the hypothesis
			while ((j<len(hypotheses[acquired_hint.ID].killer_location))and (is_double==False)):
				#Check if the hint is already perceived
				if(acquired_hint.value==hypotheses[acquired_hint.ID].killer_location[j]):
					hypotheses[acquired_hint.ID].killer_location.remove(acquired_hint.value)
					#The hint is already perceived
					is_double=True
				j = j + 1
			hypotheses[acquired_hint.ID].killer_location.append(acquired_hint.value)

		is_double=False
		j=0     
		#Check the type of the hint
		if(acquired_hint.key=='what'):
			#Run on all the hypothesis
			while ((j<len(hypotheses[acquired_hint.ID].killer_weapon))and (is_double==False)):
				#Check if the hint is already perceived
				if(acquired_hint.value==hypotheses[acquired_hint.ID].killer_weapon[j]):
					hypotheses[acquired_hint.ID].killer_weapon.remove(acquired_hint.value)
					#The hint is already perceived
					is_double=True
				j = j + 1
			hypotheses[acquired_hint.ID].killer_weapon.append(acquired_hint.value)

		# If is the fist marker acquired
		if first_hint:
			first_hint = False
			print("First hint acquired from Marker:" + str(markerId.data))
			print("ID:" + str(acquired_hint.ID) + " Key:" + str(acquired_hint.key) + " Value:" + str(acquired_hint.value))
			previous_marker = markerId.data
			
		#From the second hint forward check that the actual hint ID is different from the previous one
		if ((markerId.data != previous_marker) and (not first_hint)):
			print("New hint acquired from Marker:" + str(markerId.data))
			print("ID:" + str(acquired_hint.ID) + " Key:" + str(acquired_hint.key) + " Value:" + str(acquired_hint.value))
			previous_marker = markerId.data

	else:
		print("Marker Detected but I cannot see very well!")
##################################################

#############SMACH FINITE STATE MACHINE#############

####INITIALIZATION STATE####

class Init(smach.State):
	"""
	Initialize the investigation
	In particular this state extend Sherlock Arm
	"""
	def __init__(self):
		#Initialize smach state declaring outcomes
		smach.State.__init__(self, outcomes=['initialization'])
		
	def execute(self,userdata):
		print("The investigation is starting!")
		print("[MANIPULATION] Extend the arm to move to the first room...")
		move_sherlock_arm(-math.pi/2,0,0,0,0)
		return 'initialization'



#GO TO ROOM STATE
class Go_To_Room(smach.State):
	"""
	State in wich Sherlock navigate in the environment
	In particular first Sherlock visits all the six rooms in order
	Then after this phase the one room is selected randomly
	checking of course that the selected one is not the room 
	in wich Sherlock is actually.
	"""
	def __init__(self):
		#Initialize smach state declaring outcomes
		smach.State.__init__(self, outcomes=['search_hypotheses'])
		
	def execute(self,userdata):
		global sherlock_pos, cmd_vel_publisher, all_room_visited, room_counter
		room_number = room_counter + 1
		print('[NAVIGATION] I am going to room:' + str(room_number) + "...")
		move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

		room_pos=move_base_msgs.msg.MoveBaseActionGoal()
		room_pos.goal.target_pose.header.frame_id = "odom"
		sel_room = rooms[room_counter]
		room_pos.goal.target_pose.pose.position.x = sel_room.x
		room_pos.goal.target_pose.pose.position.y = sel_room.y
		room_pos.goal.target_pose.pose.orientation.w = 1
        
		move_base_client.wait_for_server()
		move_base_client.send_goal(room_pos.goal)
		reached = False
		while reached == False:
			distance = euler_dist(sel_room,sherlock_pos)
			if distance <= .15:
				reached = True
				print("I have reached room: " + str(room_counter	+ 1))
				#since probably sherlock has no  perfectly reached the room
				#cancel the goal
				move_base_client.cancel_all_goals()
				#Stop Sherlock
				velocity = Twist()
				velocity.linear.x = 0
				velocity.angular.z = 0
				cmd_vel_publisher.publish(velocity)
				time.sleep(.5)
			time.sleep(.1)
		
		
		######UPDATE THE ROOM COUNTER########
		
		#check if all rooms have been visited
		#if no update counter
		if (room_counter < 5) and (all_room_visited == False):
			room_counter = room_counter + 1
			all_room_visited = False
		#if all the rooms have been visited
		#we have to remember the room where Sherlock is
		#in order to randomly not selected this one
		else:
			all_room_visited = True
			are_in_room = room_counter
			while(room_counter == are_in_room):
				#randomly choose a room_counter 
				room_counter = random.randint(0,5)
		########################################
		
				
		return 'search_hypotheses'
			
##################################################### 
		
		
		
		
##################ACQUIRE HINTS STATE#################

class Acquire_Hints(smach.State):
	"""
	Once arrived into a room Sherlock starts moving its arm
	in order to acquire hints, thanks to markers into the scene.
	In order to move the arm, valid poses are declared and calling
	the service 'move_arm_service' and thanks to MoveIt! it is possible
	to solve manipulation task planning in Joints Space.
	In order to acquire 
	Once an hypothesis is acquired it is added into the custom ontology, 
	thanks to the Hypotheses class.
	The validity and completeness of the acquired hypotheses will be 
	"""
	def __init__(self):
		#Initialize smach state declaring outcomes
		smach.State.__init__(self, outcomes=['check_hp'])
	def execute(self,userdata):
		global hypotheses
		print("[MANIPULATION] Acquiring Hints ...")
		
		###### A Series of poses now is specified to scan the environment #####
		###FIRST SCAN THE DOWN AREA######
		move_sherlock_arm(0,0,-3,3,0)
		print("[SCANNING] First Pose Completed")
		move_sherlock_arm(-math.pi,0,-3,3,0)
		print("[SCANNING] Second Pose Completed")
		###THEN EXTEND THE ARM AND SCAN THE TOP AREA###
		move_sherlock_arm(-math.pi,0,0,0,-math.pi/4)
		print("[SCANNING] Third Pose Completed")
		move_sherlock_arm(0,0,0,0,-math.pi/4)
		print("[SCANNING] Fourth Pose Completed")
		move_sherlock_arm(math.pi/2,0,0,0,-math.pi/4)
		print("[SCANNING] Fiveth Pose Completed")
		return 'check_hp'
		
#CHECK HYPOTHESIS STATE
class Check_Hypothesis(smach.State):
	"""
	Finished the acquistion state it is now time to understand if Sherlock has collect
	new consistent and complete hypothesis.
	If yes in the next state Sherlock has to go in the centre of the arena to ask to the
	oracle if the hypothesis collected is the solution.
	If no consistent and complete hypotheses has been collected then go to another room
	and acquire other hints
	"""
	def __init__(self):
		#Initialize smach state declaring outcomes
		smach.State.__init__(self, outcomes=['try_hp','continue_to_acquire'])
	def execute(self,userdata):
		global already_acquired_hp, complete_consistent_hp, hypotheses
		new_complete_consistent_hp = False
		print("This is the list of all acquired hypotheses: \n")
		for i in range (6):
			hp = "ID {0}: The killer is {1} using {2} in {3}".format(i,hypotheses[i].killer,hypotheses[i].killer_weapon,hypotheses[i].killer_location)
			print(hp)
			#Check if hypotheses are consistent simply checking the number of instance present in the hypotheses fields
			if(len(hypotheses[i].killer)==1 and len(hypotheses[i].killer_location)==1 and len(hypotheses[i].killer_weapon)==1):
				complete_consistent_hp[i]  = True
				if already_acquired_hp[i] == False:
					already_acquired_hp[i] = True
					new_complete_consistent_hp = True
			else:
				complete_consistent_hp[i]  = False
				
		if new_complete_consistent_hp == True:
			print('New complete and consistent hypothesis acquired')
			print('Go to the Oracle to test it!...')
			return 'try_hp'
		else:
			print('No new complete and consistent hypothesis acquired')
			return 'continue_to_acquire'
		
		
#TELL HYPOTHESIS STATE
class Tell_Hypothesis(smach.State):
	"""
	If this state is executed means that Sherlock has collected a new complete and consistent hypothesis.
	Then lets go in the centre of the arena to ask to the Oracle the solution ID and compare with this new hp ID.
	If the new hypothesis is the true one the investigation is finished.
	Otherwise go to another room to acquire other hints.
	"""
	def __init__(self):
		#Initialization of the smach state declaring the outcomes
		smach.State.__init__(self, outcomes=['wrong_hp','correct_hp'])
	def execute(self,userdata):
		global oracle_solution_client,centre_arena, cmd_vel_publisher, complete_consistent_hp
		is_correct_hp = False
		solution_ID = None
		
		#Go in the centre of the Arena
		print('[NAVIGATION] I am going in the centre of the Arena')
		move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
		go_to=move_base_msgs.msg.MoveBaseActionGoal()
		go_to.goal.target_pose.header.frame_id = "odom"
		go_to.goal.target_pose.pose.position.x = centre_arena.x
		go_to.goal.target_pose.pose.position.y = centre_arena.y
		go_to.goal.target_pose.pose.orientation.w = 1
        
		move_base_client.wait_for_server()
		move_base_client.send_goal(go_to.goal)
		reached = False
		while reached == False:
			distance = euler_dist(centre_arena,sherlock_pos)
			if distance <= .15:
				reached = True
				print("I have reached the centre of the arena")
				#since probably sherlock has no  perfectly reached the room
				#cancel the goal
				move_base_client.cancel_all_goals()
				#Stop Sherlock
				velocity = Twist()
				velocity.linear.x = 0
				velocity.angular.z = 0
				cmd_vel_publisher.publish(velocity)
				time.sleep(.5)
			time.sleep(.1)
		
		#Ask the oracle solution and compare it with the new complete and consistent hp 
		oracle_solution_response = oracle_solution_client()
		for i in range (6):
			if ((complete_consistent_hp[i] == True) and (i==oracle_solution_response.ID)):
				is_correct_hp = True
				solution_ID = i
		if is_correct_hp:
			print("Solution found")
			print("The true hypothesis is:" + str(solution_ID))
			print("The investigation is finished")
			return 'correct_hp'
		else:
			print("No the solution is wrong!!")
			print("You have to acquire other hints and come back to me")
			return 'wrong_hp'
		

def main():
	"""
	Body of the main function
	Initialize the node as cluedo_FSM
	Wait for the services
	Create a client for ask to the oracle the solution
	Create a client for receive an hint
	Create a client for move the arm's joints
	Create the SMACH state machine and define the outcomes of FSM as CASE_SOLVED
	"""
	global move_arm_client, cmd_vel_publisher, hypotheses, hint_gen_client, oracle_solution_client
	rospy.init_node('cluedo_investigation_FSM')
	
	##Check the availability of services
	print("Wait for services")
	rospy.wait_for_service("oracle_solution")
	print("Oracle solution service online")
	rospy.wait_for_service("oracle_hint")
	print("Oracle Hint Online")
	rospy.wait_for_service("move_arm_service")
	print("Move Arm Service Online")
	
	print("Services Ready")
	
	
	##Create services client
	print("Declare all clients")
	oracle_solution_client = rospy.ServiceProxy("oracle_solution", Oracle)
	hint_gen_client = rospy.ServiceProxy("oracle_hint",Marker)
	move_arm_client = rospy.ServiceProxy("move_arm_service",MoveArm)
	
	##Create subscribers
	print("Declare all subscribers")
	odom_sub = rospy.Subscriber('/odom',Odometry,odom_clbk)
	hint_sub = rospy.Subscriber('/acquired_hint',Int32, acquired_hint_clbk)
	
	
	
	##Publisher
	print("Declare all publishers")
	cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	

	#Initialize all the hypotheses of the game
	#In particular the number of possible hp is 6,
	#differentiated by the ID
	for i in range(6):
		hypotheses_constructor = Hypotheses()
		hypotheses_constructor.ID = i
		hypotheses.append(hypotheses_constructor)

	
	##Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['CASE_SOLVED'])
	
	##Open the container
	with sm:
		"""
		Function used to define the links between FSM' s states
		"""
		##Add states to the container
		
		smach.StateMachine.add('INITIALIZE', Init(), 
								transitions={'initialization':'GO_TO_ROOM'})
		
		smach.StateMachine.add('GO_TO_ROOM', Go_To_Room(), 
								transitions={'search_hypotheses':'ACQUIRE_HINTS'}) 

		smach.StateMachine.add('ACQUIRE_HINTS', Acquire_Hints(), 
								transitions={'check_hp':'CHECK_HYPOTHESIS'})

		smach.StateMachine.add('CHECK_HYPOTHESIS', Check_Hypothesis(), 
								transitions={'try_hp':'TELL_HYPOTHESIS', 
											'continue_to_acquire':'GO_TO_ROOM'})
		
		smach.StateMachine.add('TELL_HYPOTHESIS', Tell_Hypothesis(), 
								transitions={'wrong_hp':'GO_TO_ROOM', 
											'correct_hp':'CASE_SOLVED'})
                                            
	##Create and start the introspection server for visualization
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	##Execute the state machine
	outcome = sm.execute()

	##Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()            




