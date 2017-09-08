#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import struct
import math
from sensor_msgs.msg import JointState
import std_msgs.msg
import time

#ROBOT PARAMETERS
GEAR_RATIO = (13.5,27,13.5,14.4,14.4)
MAX_PULSES = 100
HOMING_PULSE = 10
STREAMING_RATE = 100
PI = 3.14159
PPR = ((4000.0/(2*PI)),(4000.0/(2*PI)),(4000.0/(2*PI)),(1600.0/(2*PI)),(1600.0/(2*PI))) #pulse per radian
RAD2DEG = 180/PI
DEG2RAD = PI/180

#Defining functions for sending data to arduino    
def write_Int16(data,ser):
    ser.write(struct.pack('>B',(data>>8)&0xFF))
    ser.write(struct.pack('>B',data&0xFF))
def write_Byte(data,ser):
    ser.write(struct.pack('>B',(data)&0xFF))

#Declaring global variables
global pulses
global dir_ena
global enable
global joint_state

def jointCommandCallback(joint_command):
	
	global pulses
	global dir_ena
	global enable
	global joint_state

		#Calculations of pulses for each joint as well as direction of joint motion
	dir_ena = (enable<<5)
	for i in range(5):
		pulses[i] = (int)(abs(joint_command.position[i]-joint_state.position[i])*PPR[i]*GEAR_RATIO[i])
		if(joint_command.position[i]>joint_state.position[i]):
			dir_ena = dir_ena|(1<<i)
		if(pulses[i]>MAX_PULSES):
			pulses[i] = MAX_PULSES
			rospy.logwarn('Speed limit exceeded for motor %d',(i+1))

		#Updating Joint_State based on commanded angles as no explicit encoder is available
		del_ang = math.copysign(pulses[i]/(PPR[i]*GEAR_RATIO[i]),joint_command.position[i]-joint_state.position[i])
		joint_state.position[i] = joint_state.position[i] + del_ang

	#Setting pulses and direction for joint4 and joint5 (therefore joint5_velocity = joint5_velocity - joint4_velocity)
	temp_pul_3 = -pulses[3]
	temp_pul_4 = -pulses[4]
	if(dir_ena&0b00001000==0b00001000):
		temp_pul_3 = pulses[3]
	if(dir_ena&0b00010000==0b00010000):
		temp_pul_4 = pulses[4]
	temp_pul_4 = temp_pul_4 + temp_pul_3
	if(temp_pul_4>0):
		dir_ena = dir_ena|(0b00010000)
		pulses[4] = temp_pul_4
	else:
		dir_ena = dir_ena&(0b11101111)
		pulses[4] = -temp_pul_4

	#Setting directions of some joints to match simulation
	dir_ena = dir_ena^(0b00010011)

def listener():

	#Initializing global varibales
	global pulses
	global dir_ena
	global enable	
	global joint_state
	pulses = [0,0,0,0,0]
	dir_ena = 0
	enable = 1

	#Getting Initial Location of JOINTS from user
	joint1_init_pos = -1
	join2_init_pos = -1
	joint3_init_pos = -1
	while(True):
		joint1_init_pos = input("Select position of joint1 (Black zone = 0, White zone=1) (Enter 1 or 0): ")
		if(joint1_init_pos==0):
			print "Black zone"
			break
		elif(joint1_init_pos==1):
			print "White Zone"
			break
		else:
			print "Please provide input in form of 0 or 1"

	while(True):
		joint2_init_pos = input("Select position of joint2 (Black zone = 0, White zone=1) (Enter 1 or 0): ")
		if(joint2_init_pos==0):
			print "Black zone"
			break
		elif(joint2_init_pos==1):
			print "White Zone"
			break
		else:
			print "Please provide input in form of 0 or 1"

	while(True):
		joint3_init_pos = input("Select position of joint3 (Black zone = 0, White zone=1) (Enter 1 or 0): ")
		if(joint3_init_pos==0):
			print "Black zone"
			break
		elif(joint3_init_pos==1):
			print "White Zone"
			break
		else:
			print "Please provide input in form of 0 or 1"
	
	#Starting Serial Interface
	print ('Starting connection to arduino')	
	ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.005)
	rospy.sleep(3.0)
	print('connected to arduino')	
	
	#Starting Homing of Robot (make robot go to inital position) 
	print ('DO NOT DISTURB --- HOMING OF JOINTS STARTED --- DO NOT DISTURB')
	#Homing of JOINT 1---------------------------------------------1111111111111111111111----------------------------
	pre_sensor_val = -1
	sensor_val = -1
	if(joint1_init_pos==0):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00010000==0b00000000 and pre_sensor_val&0b00010000==0b00000000):
						dir_ena=0b00100001
						pulses[0] = HOMING_PULSE
					elif(sensor_val&0b00010000==0b00010000 and pre_sensor_val&0b00010000==0b00010000):
						dir_ena=0b00100000
						pulses[0] = HOMING_PULSE
					elif(sensor_val&0b00010000==0b00010000 and pre_sensor_val&0b00010000==0b00000000):
						dir_ena = 0
						pulses[0] = 0
						break
					elif(sensor_val&0b00010000==0b00000000 and pre_sensor_val&0b00010000==0b00010000):
						dir_ena = 0
						pulses[0] = 0
						break
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)
	elif(joint1_init_pos==1):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00010000==0b00000000 and pre_sensor_val&0b00010000==0b00010000):
						dir_ena = 0
						pulses[0] = 0
						break
					else:
						dir_ena = 0b00100000
						pulses[0] = HOMING_PULSE
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)

	#Homing of JOINT 2-----------------------------------------------22222222222222222222222222222222-------------------------------
	pre_sensor_val = -1
	sensor_val = -1
	if(joint2_init_pos==0):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00001000==0b00000000 and pre_sensor_val&0b00001000==0b00000000):
						dir_ena=0b00100010
						pulses[1] = HOMING_PULSE
					elif(sensor_val&0b00001000==0b00001000 and pre_sensor_val&0b00001000==0b00001000):
						dir_ena=0b00100000
						pulses[1] = HOMING_PULSE
					elif(sensor_val&0b00001000==0b00001000 and pre_sensor_val&0b00001000==0b00000000):
						dir_ena = 0
						pulses[1] = 0
						break
					elif(sensor_val&0b00001000==0b00000000 and pre_sensor_val&0b00001000==0b00001000):
						dir_ena = 0
						pulses[1] = 0
						break
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)
	elif(joint2_init_pos==1):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00001000==0b00001000 and pre_sensor_val&0b00001000==0b00000000):
						dir_ena = 0
						pulses[1] = 0
						break
					else:
						dir_ena = 0b00100010
						pulses[1] = HOMING_PULSE
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)
	
	
#Homing of JOINT 3-----------------------------------------------33333333333333333333333333333333333-------------------------------
	pre_sensor_val = -1
	sensor_val = -1
	if(joint3_init_pos==0):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00000100==0b00000000 and pre_sensor_val&0b00000100==0b00000000):
						dir_ena=0b00100100
						pulses[2] = HOMING_PULSE
					elif(sensor_val&0b00000100==0b00000100 and pre_sensor_val&0b00000100==0b00010000):
						dir_ena=0b00100000
						pulses[2] = HOMING_PULSE
					elif(sensor_val&0b00000100==0b00000100 and pre_sensor_val&0b00000100==0b00000000):
						dir_ena = 0
						pulses[2] = 0
						break
					elif(sensor_val&0b00000100==0b00000000 and pre_sensor_val&0b00000100==0b00000100):
						dir_ena = 0
						pulses[2] = 0
						break
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)
	elif(joint3_init_pos==1):
		while(True):
			tm = time.clock()
			#Sending commands to arduino
			write_Int16(pulses[0],ser)
			write_Int16(pulses[1],ser)
			write_Int16(pulses[2],ser)
			write_Int16(pulses[3],ser)
			write_Int16(pulses[4],ser)
			write_Byte(dir_ena,ser)
			#Reading Sensor values from arduino
			n = ser.inWaiting()
			if(n>0):
				if(n==1):
					sensor_val = ord(ser.read(n))
				else:
					#Clearing buffer in case of wrong data from arduino
					ser.read(n)
			if(not (sensor_val==-1)):
				if(pre_sensor_val==-1):
					pre_sensor_val = sensor_val
				else:
					if(sensor_val&0b00000100==0b00000100 and pre_sensor_val&0b00000100==0b00000000):
						dir_ena = 0
						pulses[2] = 0
						break
					else:
						dir_ena = 0b00100100
						pulses[2] = HOMING_PULSE
			pre_sensor_val = sensor_val
			tm = time.clock()-tm
			time.sleep(0.01-tm)


#Homing of JOINT 4-----------------------------------------------44444444444444444444444444444444444444-------------------------------
	pre_sensor_val = -1
	sensor_val = -1
	while(True):
		tm = time.clock()
		#Sending commands to arduino
		write_Int16(pulses[0],ser)
		write_Int16(pulses[1],ser)
		write_Int16(pulses[2],ser)
		write_Int16(pulses[3],ser)
		write_Int16(pulses[4],ser)
		write_Byte(dir_ena,ser)
		#Reading Sensor values from arduino
		n = ser.inWaiting()
		if(n>0):
			if(n==1):
				sensor_val = ord(ser.read(n))
			else:
				#Clearing buffer in case of wrong data from arduino
				ser.read(n)
		if(not (sensor_val==-1)):
			if(pre_sensor_val==-1):
				pre_sensor_val = sensor_val
			else:
				if(sensor_val&0b00000010==0b00000010 and pre_sensor_val&0b00000010==0b00000000):
					dir_ena=0
					pulses[3] = 0
					break
				else:
					dir_ena=0b00101000
					pulses[3] = HOMING_PULSE*5
		pre_sensor_val = sensor_val
		tm = time.clock()-tm
		time.sleep(0.01-tm)

#Homing of JOINT 5-----------------------------------------------55555555555555555555555555555555555-------------------------------
	pre_sensor_val = -1
	sensor_val = -1
	while(True):
		tm = time.clock()
		#Sending commands to arduino
		write_Int16(pulses[0],ser)
		write_Int16(pulses[1],ser)
		write_Int16(pulses[2],ser)
		write_Int16(pulses[3],ser)
		write_Int16(pulses[4],ser)
		write_Byte(dir_ena,ser)
		#Reading Sensor values from arduino
		n = ser.inWaiting()
		if(n>0):
			if(n==1):
				sensor_val = ord(ser.read(n))
			else:
				#Clearing buffer in case of wrong data from arduino
				ser.read(n)
		if(not (sensor_val==-1)):
			if(pre_sensor_val==-1):
				pre_sensor_val = sensor_val
			else:
				if(sensor_val&0b00000001==0b00000001 and pre_sensor_val&0b00000001==0b00000000):
					dir_ena=0
					pulses[4] = 0
					break
				else:
					dir_ena=0b00101000
					pulses[4] = HOMING_PULSE*5
		pre_sensor_val = sensor_val
		tm = time.clock()-tm
		time.sleep(0.01-tm)

	print ('Going to initial angles')
	#-------------------------------------------------HOMING DONE-----------------------------------------------------------------------------
	initial_pulse_req = (35*PPR[0]*GEAR_RATIO[0]*DEG2RAD,28.5*PPR[1]*GEAR_RATIO[1]*DEG2RAD,(38.5+90)*PPR[2]*GEAR_RATIO[2]*DEG2RAD,(4+90)*PPR[3]*GEAR_RATIO[3]*DEG2RAD,0*PPR[4]*GEAR_RATIO[4]*DEG2RAD)
	pulses = [0,0,0,0,0]
	cur_pul = [0,0,0,0,0]
	num_pul = 0	
	while(True):
		tm = time.clock()
		#Sending commands to arduino
		write_Int16(pulses[0],ser)
		write_Int16(pulses[1],ser)
		write_Int16(pulses[2],ser)
		write_Int16(pulses[3],ser)
		write_Int16(pulses[4],ser)
		write_Byte(dir_ena,ser)
		pulses = [0,0,0,0,0]
		#Reading Sensor values from arduino
		n = ser.inWaiting()
		if(n>0):
			if(n==1):
				sensor_val = ord(ser.read(n))
			else:
				#Clearing buffer in case of wrong data from arduino
				ser.read(n)
		if(cur_pul[0]<initial_pulse_req[0]):
			pulses[0] = 10
			cur_pul[0] = cur_pul[0]+10
		if(cur_pul[1]<initial_pulse_req[1]):
			pulses[1] = 10		
			cur_pul[1] = cur_pul[1]+10
		if(cur_pul[2]<initial_pulse_req[2]):
			pulses[2] = 10
			cur_pul[2] = cur_pul[2]+10
		if(cur_pul[3]<initial_pulse_req[3]):
			pulses[3] = 10
			cur_pul[3] = cur_pul[3]+10
		if(cur_pul[4]<initial_pulse_req[4]):
			pulses[4] = 10
			cur_pul[4] = cur_pul[4]+10
		dir_ena = 0b00101001
		if(pulses[0]==0  and pulses[1]==0  and pulses[2]==0  and pulses[3]==0  and pulses[4]==0 ):
			break
		tm = time.clock()-tm
		time.sleep(0.01-tm)
	
	print ('Homing Done, Creating Node and setting up Driver')
	pulses = [0,0,0,0,0]
	dir_ena = 0
	tm = 0
	
	#------------------------------------------------------------------------------------------------------------------------------------------
	#------------------------------------------------------------------------------------------------------------------------------------------
	#------------------------------------------------------------------------------------------------------------------------------------------
	#------------------------------------------------------------------------------------------------------------------------------------------
	#------------------------------------------------------------------------------------------------------------------------------------------
		
	#Creating jointStatePublisher and Subscriber
	rospy.init_node('tal_brabo_driver', anonymous=True)
	rospy.Subscriber('/joint_command', JointState, jointCommandCallback)
	jointStatePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)

	#Defining joint_state (current state of robot)
	joint_state = JointState()
	joint_state.header = std_msgs.msg.Header()
	joint_state.header.seq = 0
	joint_state.header.stamp = rospy.Time.now()
	joint_state.header.frame_id = ''
	joint_state.name = ['link_1_joint', 'link_2_joint', 'link_3_joint', 'link_4_joint', 'link_5_joint']
	joint_state.position = [0.0, 0.0, -PI/2, PI/2, 0.0]

	#Command streaming rate
	rate = rospy.Rate(STREAMING_RATE) # in hz
	while not rospy.is_shutdown():
		
		#Sending commands to arduino
		write_Int16(pulses[0],ser)
		write_Int16(pulses[1],ser)
		write_Int16(pulses[2],ser)
		write_Int16(pulses[3],ser)
		write_Int16(pulses[4],ser)
		write_Byte(dir_ena,ser)
		
		#Reading Sensor values from arduino
		n = ser.inWaiting()
		if(n>0):
			if(n==1):
				sensor_val = ord(ser.read(n))
				#rospy.loginfo(format(sensor_val,'08b'))
			else:
				#Clearing buffer in case of wrong data from arduino
				ser.read(n)
	
		#Updating header of join_state
		joint_state.header.seq = joint_state.header.seq+1
		joint_state.header.stamp = rospy.Time.now()
		rospy.loginfo(pulses)
		#Publishing joint_state 
		jointStatePublisher.publish(joint_state)
		
		#resetting pulses
		#pulses = [0,0,0,0,0]

		rate.sleep()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

