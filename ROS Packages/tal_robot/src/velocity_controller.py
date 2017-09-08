#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


PI = 3.14
DEG2RAD = PI/180
RAD2DEG = 180/PI

KPX = 5*(0.0001)
KDX = 2*(0.0001)
KIX = 0*(0.00001)
KPY = 1*(0.00001)
KDY = 5*(0.000001)
KIY = 0*(0.00001)
KPZ = 1*(0.00001)
KDZ = 5*(0.000001)
KIZ = 0*(0.00001)


global end_eff_vel
global joint_command
global pre_point

def ballCenterCallback(msg):
	global end_eff_vel
	global pre_point
	if(msg.x==-1000):
		end_eff_vel[0] = 0.000
		end_eff_vel[1] = 0.000
		end_eff_vel[2] = 0.000
		end_eff_vel[3] = 0.00
		end_eff_vel[4] = 0.00
		end_eff_vel[5] = 0.00
	else:
		end_eff_vel[0] = msg.z*KPX + (msg.z-pre_point.z)*KDX + msg.z*KIX
		end_eff_vel[1] = msg.x*KPY + (msg.x-pre_point.x)*KDY + msg.x*KIY
		end_eff_vel[2] = msg.y*KPZ + (msg.y-pre_point.y)*KDZ + msg.y*KIZ
		pre_point.x = msg.x
		pre_point.y = msg.y
		pre_point.z = msg.z

def jointStateCallback(joint_state):
	global end_eff_vel
	global joint_command

	#Updating jointAngles from actual(or simulation) to based upon DHparameter as Jacobian is analytically calculated from DHparameters
	theta1 =  joint_state.position[0] + 90*DEG2RAD
	theta2 =  joint_state.position[1] - 90*DEG2RAD
	theta3 = -joint_state.position[2] 
	theta4 = -joint_state.position[3] + 90*DEG2RAD
	theta5 =  joint_state.position[4] + 0*DEG2RAD

	#Calculating jacobian for given joint angles
	jacobian = np.array([[ 0.022750000000000001393035734030114*np.cos(theta1 + theta2 + theta3 + theta4) - 0.18749999999999998851893625799356*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.18750000000000001148106374200644*np.sin(theta1 + theta2 + theta3) - 0.18750000000000001148106374200644*np.sin(theta1 + theta2) - 0.0000000000000000027860714680602285463203131670179*np.cos(theta1) - 0.18749999999999998851893625799356*np.sin(theta1 - 1.0*theta2) - 0.022749999999999998606964265969886*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.18749999999999998851893625799356*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.18750000000000001148106374200644*np.sin(theta1 + theta2 + theta3) + 0.022750000000000001393035734030114*np.cos(theta1 + theta2 + theta3 + theta4) - 0.18750000000000001148106374200644*np.sin(theta1 + theta2) + 0.18749999999999998851893625799356*np.sin(theta1 - 1.0*theta2) + 0.022749999999999998606964265969886*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.18749999999999998851893625799356*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.18750000000000001148106374200644*np.sin(theta1 + theta2 + theta3) + 0.022750000000000001393035734030114*np.cos(theta1 + theta2 + theta3 + theta4) + 0.022749999999999998606964265969886*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.022750000000000001393035734030114*np.cos(theta1 + theta2 + theta3 + theta4) + 0.022749999999999998606964265969886*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4),                                                                                                                                                                                                                                 0],
[ 0.18750000000000001148106374200644*np.cos(theta1 + theta2 + theta3) - 0.022749999999999998606964265969886*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.18749999999999998851893625799356*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.022750000000000001393035734030114*np.sin(theta1 + theta2 + theta3 + theta4) + 0.18750000000000001148106374200644*np.cos(theta1 + theta2) + 0.18749999999999998851893625799356*np.cos(theta1 - 1.0*theta2) - 0.0000000000000000027860714680602285463203131670179*np.sin(theta1), 0.022749999999999998606964265969886*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.18750000000000001148106374200644*np.cos(theta1 + theta2 + theta3) - 0.18749999999999998851893625799356*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.022750000000000001393035734030114*np.sin(theta1 + theta2 + theta3 + theta4) + 0.18750000000000001148106374200644*np.cos(theta1 + theta2) - 0.18749999999999998851893625799356*np.cos(theta1 - 1.0*theta2), 0.022749999999999998606964265969886*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.18750000000000001148106374200644*np.cos(theta1 + theta2 + theta3) - 0.18749999999999998851893625799356*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.022750000000000001393035734030114*np.sin(theta1 + theta2 + theta3 + theta4), 0.022749999999999998606964265969886*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.022750000000000001393035734030114*np.sin(theta1 + theta2 + theta3 + theta4),                                                                                                                                                                                                                                 0],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                   - 0.0455*np.sin(theta2 + theta3 + theta4) - 0.375*np.cos(theta2 + theta3) - 0.375*np.cos(theta2),                                                                                                                                                                                                                                               - 0.0455*np.sin(theta2 + theta3 + theta4) - 0.375*np.cos(theta2 + theta3),                                                                                                                               -0.0455*np.sin(theta2 + theta3 + theta4),                                                                                                                                                                                                                                 0],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                          -1.0*np.sin(theta1),                                                                                                                                                                                                                                                                                                  -1.0*np.sin(theta1),                                                                                                                                                    -1.0*np.sin(theta1), 0.50000000000000003061616997868383*np.sin(theta1 + theta2 + theta3 + theta4) - 0.49999999999999996938383002131617*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) - 0.00000000000000006123233995736766035868820147292*np.sin(theta1)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                               np.cos(theta1),                                                                                                                                                                                                                                                                                                       np.cos(theta1),                                                                                                                                                         np.cos(theta1), 0.00000000000000006123233995736766035868820147292*np.cos(theta1) - 0.50000000000000003061616997868383*np.cos(theta1 + theta2 + theta3 + theta4) + 0.49999999999999996938383002131617*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4)],
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         1.0,                                                                                                                                                                                                                                                                                                                                                                                         0.00000000000000006123233995736766035868820147292,                                                                                                                                                                                                                                                                 0.00000000000000006123233995736766035868820147292,                                                                                                                   0.00000000000000006123233995736766035868820147292,                                                                                                                                                             np.cos(theta2 + theta3 + theta4) + 3.7493994566546441699929521964488e-33]])
 
	#Calculating joint_velocities using jacobian pseudo inverse
	q_dot = np.linalg.pinv(jacobian).dot(end_eff_vel)
	rospy.loginfo(q_dot)
	#Converting joint_velocities back to actual robot
	joint_command.position[0] = joint_state.position[0] + q_dot[0]
	joint_command.position[1] = joint_state.position[1] + q_dot[1]
	joint_command.position[2] = joint_state.position[2] - q_dot[2]
	joint_command.position[3] = joint_state.position[3] - q_dot[3]
	joint_command.position[4] = joint_state.position[4] + q_dot[4]


def sender():
	#Defining endEffectorVelocity as global variable to be calculated based on center of ball
	global end_eff_vel
	end_eff_vel = np.array([0.0,0.0,0.0,0.0,0.0,0.0])		
	global joint_command
	global pre_point
	pre_point = Point(0,0,0)

	#Creating node, publisher and subscribers
	rospy.init_node('velocity_controler', anonymous=True)
	jointCommandPublisher = rospy.Publisher('/joint_command', JointState, queue_size=10)
	rospy.Subscriber('/ball_center', Point, ballCenterCallback)
	rospy.Subscriber('joint_states', JointState, jointStateCallback)

	#Publishing Rate
	rate = rospy.Rate(100) # in hz

	#Defining joint_command
	joint_command = JointState()
	joint_command.header = std_msgs.msg.Header()
	joint_command.header.seq = 0
	joint_command.header.stamp = rospy.Time.now()
	joint_command.header.frame_id = ''
	joint_command.name = ['link_1_joint', 'link_2_joint', 'link_3_joint', 'link_4_joint', 'link_5_joint']
	joint_command.position = [0.0, 0.0, 0.0, 0.0, 0.0]
	
	while not rospy.is_shutdown():
		#Updating header for joint_command
		joint_command.header.seq = joint_command.header.seq + 1
		joint_command.header.stamp = rospy.Time.now()
		#rospy.loginfo(joint_command)
		#Publishing Joint Commands
		jointCommandPublisher.publish(joint_command)
		rate.sleep()

if __name__ == '__main__':
	try:
		sender()
	except rospy.ROSInterruptException:
		pass


