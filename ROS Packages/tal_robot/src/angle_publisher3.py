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

KP = 1*(0.00001)
KD = 6*(0.000001)
KI = 0*(0.00001)

global end_eff_vel

def ballCenterCallback(msg):
	global end_eff_vel
	if(msg.x==-1000):
		end_eff_vel[0] = 0
		end_eff_vel[1] = 0
		end_eff_vel[2] = 0
	else:
		end_eff_vel[0] = msg.x*KP + msg.x*KD + msg.x*KI
		end_eff_vel[1] = 0
		end_eff_vel[2] = msg.y*KP + msg.y*KD + msg.y*KI

def jointStateCallback(joint_state):
	global end_eff_vel

	#Updating jointAngles from actual(or simulation) to based upon DHparameter as Jacobian is analytically calculated from DHparameters
	theta1 =  joint_state.position[0] + 90*DEG2RAD
	theta2 =  joint_state.position[0] - 90*DEG2RAD
	theta3 = -joint_state.position[0] 
	theta4 = -joint_state.position[0] - 90*DEG2RAD
	theta5 =  joint_state.position[0] + 90*DEG2RAD

	#Calculating jacobian for given joint angles
	jacobian = np.array([[ 0.0112253128654403692094269480304*np.cos(theta1 + theta2 + theta3 + theta4) - 0.27151380302421940921009380076612*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.10348619697578059078990619923388*np.sin(theta1 + theta2 + theta3) - 0.10348619697578059078990619923388*np.sin(theta1 + theta2) + 0.018226222462944643775046101612924*np.cos(theta1) - 0.27151380302421940921009380076612*np.sin(theta1 - 1.0*theta2) - 0.029451535328385012984473049643325*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.27151380302421940082132627859807*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.10348619697578058240113867706584*np.sin(theta1 + theta2 + theta3) + 0.011225312865440368299483953277008*np.cos(theta1 + theta2 + theta3 + theta4) - 0.10348619697578058240113867706584*np.sin(theta1 + theta2) + 0.27151380302421940082132627859807*np.sin(theta1 - 1.0*theta2) + 0.029451535328385012074530054889932*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.27151380302421940082132627859807*np.sin(theta1 - 1.0*theta2 - 1.0*theta3) - 0.10348619697578058240113867706584*np.sin(theta1 + theta2 + theta3) + 0.011225312865440368299483953277008*np.cos(theta1 + theta2 + theta3 + theta4) + 0.029451535328385012074530054889932*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4), 0.011225312865440368299483953277008*np.cos(theta1 + theta2 + theta3 + theta4) + 0.029451535328385012074530054889932*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4),                                                                                                                                                                                                                  0],
    [ 0.10348619697578059078990619923388*np.cos(theta1 + theta2 + theta3) - 0.029451535328385012984473049643325*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.27151380302421940921009380076612*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.0112253128654403692094269480304*np.sin(theta1 + theta2 + theta3 + theta4) + 0.10348619697578059078990619923388*np.cos(theta1 + theta2) + 0.27151380302421940921009380076612*np.cos(theta1 - 1.0*theta2) + 0.018226222462944643775046101612924*np.sin(theta1), 0.029451535328385012074530054889932*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.10348619697578058240113867706584*np.cos(theta1 + theta2 + theta3) - 0.27151380302421940082132627859807*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.011225312865440368299483953277008*np.sin(theta1 + theta2 + theta3 + theta4) + 0.10348619697578058240113867706584*np.cos(theta1 + theta2) - 0.27151380302421940082132627859807*np.cos(theta1 - 1.0*theta2), 0.029451535328385012074530054889932*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.10348619697578058240113867706584*np.cos(theta1 + theta2 + theta3) - 0.27151380302421940082132627859807*np.cos(theta1 - 1.0*theta2 - 1.0*theta3) + 0.011225312865440368299483953277008*np.sin(theta1 + theta2 + theta3 + theta4), 0.029451535328385012074530054889932*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.011225312865440368299483953277008*np.sin(theta1 + theta2 + theta3 + theta4),                                                                                                                                                                                                                  0],
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                            - 0.036364966571066269399002906544183*np.sin(theta2 + theta3 + theta4) - 0.33524874885020919390576921159663*np.cos(theta2 + theta3) - 0.33524874885020919390576921159663*np.cos(theta2),                                                                                                                                                                                     - 0.036364966571066269399002906544183*np.sin(theta2 + theta3 + theta4) - 0.33524874885020919390576921159663*np.cos(theta2 + theta3),                                                                                                  -0.036364966571066269399002906544183*np.sin(theta2 + theta3 + theta4),                                                                                                                                                                                                                  0],
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                           -0.89399666360055785041538456425769*np.sin(theta1),                                                                                                                                                                                                                                                                   -0.89399666360055785041538456425769*np.sin(theta1),                                                                                                                     -0.89399666360055785041538456425769*np.sin(theta1), 0.24671017286682130130608676989891*np.sin(theta1 + theta2 + theta3 + theta4) - 0.64728649073373654910929779435878*np.sin(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) + 0.40057631786691524780321102445988*np.sin(theta1)],
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                            0.89399666360055785041538456425769*np.cos(theta1),                                                                                                                                                                                                                                                                    0.89399666360055785041538456425769*np.cos(theta1),                                                                                                                      0.89399666360055785041538456425769*np.cos(theta1), 0.64728649073373654910929779435878*np.cos(theta1 - 1.0*theta2 - 1.0*theta3 - 1.0*theta4) - 0.40057631786691524780321102445988*np.cos(theta1) - 0.24671017286682130130608676989891*np.cos(theta1 + theta2 + theta3 + theta4)],
    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       1.0,                                                                                                                                                                                                                                                                                                                                                                                                       -0.44807361612917018245383360408596,                                                                                                                                                                                                                                                                               -0.44807361612917018245383360408596,                                                                                                                                 -0.44807361612917018245383360408596,                                                                                                              0.79923003452892899778028366031172*np.cos(theta2 + theta3 + theta4) + 0.20076996547107095747962288812537]])
	
	#Calculating joint_velocities using jacobian pseudo inverse
	q_dot = np.linalg.pinv(jacobian).dot(end_eff_vel)
	
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

	#Creating node, publisher and subscribers
	rospy.init_node('velocity_controler', anonymous=True)
	jointCommandPublisher = rospy.Publisher('/joint_command', JointState, queue_size=10)
	rospy.Subscriber('/ball_center', Point, ballCenterCallback)
	rospy.Subscriber('/joint_state', JointState, jointStateCallback)

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
		
		#Publishing Joint Commands
		jointCommandPublisher.publish(joint_command)
		rate.sleep()

if __name__ == '__main__':
	try:
		sender()
	except rospy.ROSInterruptException:
		pass


