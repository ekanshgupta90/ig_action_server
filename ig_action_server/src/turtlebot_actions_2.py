import rospy
import time
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import publisher
from math import radians
import dynamic_reconfigure.client

def moveAbs(x,y,v):
	move (x,y,v,"Absolute");


def moveRel(x,y,v):
	move (x,y,v,"Relative");

def turnAbs(d,r):
	pass

def turnRel(a,r):
	turn(a,r)

def move(x,y,v,action):
	setVelocity(v, 'LINEAR');
	if action == "Absolute":
		frameType = 'map'
	else:
		frameType = 'base_link'

	move_base = publisher.move_base_action_client ()
    
	print "mvoeing"
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = frameType
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x 
	if frameType == 'map':
		goal.target_pose.pose.position.y = y #3 meters
	goal.target_pose.pose.orientation.w = 1.0 #go forward

	move_base.send_goal(goal)
	success = move_base.wait_for_result()
	if not success:
		move_base.cancel_goal()
		rospy.logerr("The base failed to move forward")
	else:
		rospy.loginfo("Successfully executed the vertex")
	
	publisher.close_move_base_action_client()

def turn(angle, rotation):
	setVelocity(rotation, 'ANGULAR');
	twist = Twist()
	cmd_vel = getCmdVel()
	cycles = int(angle/45)
	twist.angular.z = radians(45)*cycles #0.785398*2*rotation    # 90 deg/s
	for i in range(0,int(cycles)):
		cmd_vel.publish(twist)
		rospy.sleep(0.5)

def getCmdVel():
	cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
	rospy.sleep(1)
	return cmd_vel

def setVelocity(velocity, type):
	client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
	if type == 'LINEAR':
		params = {'min_vel_x' : velocity, 'max_vel_x' : velocity, 'max_trans_vel': velocity, 'min_trans_vel':velocity}
	else:
		params = {'max_rot_vel': velocity, 'min_rot_vel':velocity}
	config = client.update_configuration(params)
	rospy.sleep(0.2)
	rospy.loginfo("Velocity set to " + str(velocity))