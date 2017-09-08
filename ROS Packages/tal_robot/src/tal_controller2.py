#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import struct
import math
from sensor_msgs.msg import JointState


GEAR_RATIO = (13.5,27,13.5,14.4,14.4)
PI = 3.14
PPR = ((4000.0/(2*PI)),(4000.0/(2*PI)),(4000.0/(2*PI)),(1600.0/(2*PI)),(1600.0/(2*PI))) #pulse per radian
RAD2DEG = 180/PI
DEG2RAD = PI/180
MAX_PULSES = 60
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.005)

#states

enable = 1
global angles
global tnp
global min_n
global max_n
global min_nn
    
def write_Int16(data):
    ser.write(struct.pack('>B',(data>>8)&0xFF))
    ser.write(struct.pack('>B',data&0xFF))

def write_Byte(data):
    ser.write(struct.pack('>B',(data)&0xFF))


def callback(joint_state):
	global angles
	global tnp
	global min_n
	global max_n
	global min_nn
	dir_ena = (enable<<5)
	pulses = [0]*5
	for i in range(5):
		pulses[i] = (int)(abs(joint_state.position[i]-angles[i])*PPR[i]*GEAR_RATIO[i])
		if(joint_state.position[i]>angles[i]):
			dir_ena = dir_ena|(1<<i)
		if(pulses[i]>MAX_PULSES):
			pulses[i] = MAX_PULSES
			rospy.logwarn('Speed limit exceeded for motor %d',(i+1))
			del_ang = math.copysign(MAX_PULSES/(PPR[i]*GEAR_RATIO[i]),joint_state.position[i]-angles[i])
			angles[i] = angles[i] + del_ang
		else:
			angles[i] = joint_state.position[i]

	temp_pul_3 = -pulses[3]
	temp_pul_4 = -pulses[4]
	#rospy.loginfo(dir_ena)
	if(dir_ena&0b00001000==0b00001000):
		temp_pul_3 = pulses[3]

	if(dir_ena&0b00010000==0b00010000):
		temp_pul_4 = pulses[4]
	
	
	temp_pul_4 = temp_pul_4 - temp_pul_3
	if(temp_pul_4>0):
		dir_ena = dir_ena|(0b00010000)
		pulses[4] = temp_pul_4
	else:
		dir_ena = dir_ena&(0b11101111)
		pulses[4] = -temp_pul_4
	#rospy.loginfo(temp_pul_3)
	dir_ena = dir_ena^(0b00000011)
	write_Int16(pulses[0])
	write_Int16(pulses[1])
	write_Int16(pulses[2])
	write_Int16(pulses[3])
	write_Int16(pulses[4])
	write_Byte(dir_ena)

	tnp = tnp + pulses[0]

	n = 0
	#n = ser.inWaiting()
	if(n>0):
		s = ser.read(n)
		#rospy.loginfo(n)
		if(n==8):
			s1 = ord(s[0])
			s2 = ord(s[1])
			s3 = ord(s[2])
			s4 = ord(s[3])
			s5 = ord(s[4])
			s6 = ord(s[5])
			s7 = ord(s[6])
			s8 = ord(s[7])
			data1 = s4<<24|s3<<16|s2<<8|s1
			data2 = s8<<24|s7<<16|s6<<8|s5
			if(data1>2147483647):
				data1 = (data1-1) - 4294967295
			if(data2>2147483647):
				data2 = (data2-1) - 4294967295
			if(data1>max_n):
				max_n = data1
			if(data1<min_n):
				min_n = data1
			if(data2<min_nn):
				min_nn = data2
			rospy.loginfo("%d\t%d\t%d\t%d\t%d\t%d\t%d",data1,data2,tnp,min_n,max_n,n,pulses[2])
		
	#rospy.loginfo(n)
	#rospy.loginfo(joint_state.position)
	rospy.loginfo(pulses)
	#rospy.loginfo(tnp)
	#rospy.loginfo(angles)
	#rospy.loginfo(dir_ena)

def listener():
	global tnp
	global min_n
	global max_n
	global min_nn
	global angles
	#angles = [0,0,0,0,0]
	angles = [-1.5704006431838355, -0.00017951764015866623, -1.5692848026691197, 1.5708947149710393, 0]
	#[0*DEG2RAD, -90*DEG2RAD, 90*DEG2RAD, 0*DEG2RAD, 0*DEG2RAD]
	min_n = 8
	max_n = 0
	tnp = 0
	min_nn = 0
	rospy.init_node('tal_controller', anonymous=True)
	rospy.loginfo('connected to arduino')
	rospy.Subscriber('joint_states', JointState, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
