#!/usr/bin/env python

import rospy, time ,math, message_filters ################# imports all required libraries and functions #############
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance
from sensor_msgs.msg import Imu
from math import atan, asin,cos, sin, atan2,pi
from std_msgs.msg import String, Float32



############################### global variables #######################
imu_data = Imu()
acc_f_p = np.zeros(3)
acc_i_p = np.zeros(3)
acc = np.zeros(3)
rb_data = PoseStamped()
battery_percent = Float32()
velocity_x = Float32()
mocap_p = np.zeros(3)
mocap_i = np.zeros(3)
yaw_mocap_p = 0
yaw_mocap_i = 0
yaw_rate = 0
yaw_rate_f_p = 0
yaw_rate_i_p = 0
u_p = 0
v_p = 0
w_p = 0
P_c = np.identity(3)
P_p = np.identity(3)
velocity_f_p = np.transpose(np.zeros(3))
velocity_k = np.transpose(np.zeros(3))
velocity_k_p = np.transpose(np.zeros(3))
velocity_i_p = np.transpose(np.zeros(3))
velocity = np.transpose(np.zeros(3))
velocity_acc = np.transpose(np.zeros(3))
velocity_f_p[0] = 0
velocity_f_p[1] = 0
velocity_f_p[2] = 0

velocity_k[0] = 0
velocity_k[1] = 0
velocity_k[2] = 0

velocity_k_p[0] = 0
velocity_k_p[1] = 0
velocity_k_p[2] = 0

velocity_i_p[0] = 0
velocity_i_p[1] = 0
velocity_i_p[2] = 0

velocity[0] = 0
velocity[1] = 0
velocity[2] = 0

velocity_acc[0] = 0
velocity_acc[1] = 0
velocity_acc[2] = 0
Q = np.array([[0.0123,0,0],[0,0.0403,0],[0,0,0.0266]])
R = np.array([[9.1e-04,0,0],[0,1.105e-3,0],[0,0,2.47e-3]])

################### These are functions to assign the data from subscribed topics to global variables #####################
def euler_from_quaternion(x,y,z,w):

	roll = atan2(2*(w*x-y*z),-(x**2+y**2-z**2-w**2))
	yaw  = atan2(-2*(y*x-z*w),(x**2-y**2-z**2+w**2))
	try:
		pitch   = asin(2*(y*w+z*x))
	except:
		pitch = asin(1)
	orientationAngle = np.array([roll,pitch,yaw])
	return orientationAngle

def call_battery(data):
    global battery_percent
    battery_percent = data
	
def call_imu_data(data):
    global imu_data
    imu_data = data

def call_rb_data(data):
	global rb_data
	rb_data = data

def rotatebodytoglobal(V,rb):
    oAngle = euler_from_quaternion(rb.pose.orientation.x,rb.pose.orientation.y,rb.pose.orientation.z,rb.pose.orientation.w)
    phi = oAngle[0]
    theta = oAngle[1]
    psi = oAngle[2]
    R = np.array([[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)], [sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta)], [cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)]])
    invR = np.linalg.inv(R)
    detR = np.linalg.det(invR)
    return detR, np.dot(invR,V)


def filter_acc(imu, rb, battery): ######## takes accelerometer data ########
	global acc_f_p, acc_i_p, acc, mocap_p, mocap_i, velocity_f_p, velocity_k_p, velocity_k, velocity_i_p, velocity, yaw_rate_f_p, yaw_rate_i_p, yaw_rate, yaw_mocap_p, yaw_mocap_i, u_p, v_p, w_p, velocity_x, Q, P_c, P_p	
	
	## Calibrated Accelerometer Values for different CFs
	cf_uri = rospy.get_param('uri')
	l = len(cf_uri)
	if cf_uri[10:l]=='30/1M':
		cf_cal=np.array([0.109156544, -0.076713547, 0.00043244])
	if cf_uri[10:l]=='90/1M':
		cf_cal = np.array([0.217275, -0.145452, 0.163200])
	if cf_uri[10:l]=='50/2M':
		cf_cal = np.array([-0.00572555, 0.0561713, 0.001252885])

	######### Removing accelerometer biases for testn ###############
	cf_cal = np.array([0, 0, 0])
	######## Initialization of values################
	fin_data1 = Imu()
	fin_data1.header = imu.header





	## Filter Coefficient

	alpha= 0.3

	#Acceleration Update
	
	acc[0] = alpha*acc_f_p[0] + (1-alpha)*acc_i_p[0]
	acc[1] = alpha*acc_f_p[1] + (1-alpha)*acc_i_p[1]
	acc[2] = alpha*acc_f_p[2] + (1-alpha)*acc_i_p[2]
	
	#Angular velocity update
	
	gyro = np.array([imu.angular_velocity.x,-imu.angular_velocity.y,-imu.angular_velocity.z])

	## Biasing Values for CF1

	acc_i_p = np.subtract(np.array([-imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]), cf_cal)

	acc_f_p = acc

	# Orientation update from acc
	#roll_a = atan(acc[1]/(acc[2]))*180/pi
	#pitch_a = atan(-acc[0]/(((acc[1]**2 + acc[2]**2)**0.5)))*180/pi

	oAngle = euler_from_quaternion(rb.pose.orientation.x,rb.pose.orientation.y,rb.pose.orientation.z,rb.pose.orientation.w)
	roll = oAngle[0]
	pitch = oAngle[1]
	yaw_mocap = oAngle[2]
		
	#Position and velocity from MoCap   
	# velocity = np.transpose([0,0,0])
	
	velocity[0] = alpha*velocity_f_p[0] + (1-alpha)*velocity_i_p[0]
	velocity[1] = alpha*velocity_f_p[1] + (1-alpha)*velocity_i_p[1]
	velocity[2] = alpha*velocity_f_p[2] + (1-alpha)*velocity_i_p[2]
		
	mocap_i = np.array([rb.pose.position.x,rb.pose.position.y,rb.pose.position.z])

	velocity_i_p = (mocap_i-mocap_p)/0.01	
	velocity_f_p = velocity
	
	mocap_p = mocap_i
	
	########################## Velocity from aceelerometer ########################
	
	acc_bias = np.array([-0.0696, 0.1383, 0.0137])
	acc_t = acc - acc_bias
	
	u_c = u_p + 0.01*(acc_t[0]-9.81*sin(pitch)+gyro[2]*v_p-gyro[1]*w_p)
	v_c = v_p + 0.01*(acc_t[1]+9.81*cos(pitch)*sin(roll)+gyro[0]*w_p-gyro[2]*u_p)
	w_c = w_p + 0.01*(acc_t[2]+9.81*cos(pitch)*cos(roll)+gyro[1]*u_p-gyro[0]*v_p)

	u_p = u_c
	v_p = v_c
	w_p = w_c
	
	velocity_x = u_c
	V = np.array([u_c,v_c,w_c])
	detR, Rv = rotatebodytoglobal(V.transpose(),rb)	
	Temp = Imu()

	Temp.linear_acceleration.x = u_c
	Temp.linear_acceleration.y = v_c
	Temp.linear_acceleration.z = w_c

	##################### Kalman Update #############################
	A = np.array([[0,gyro[2],-gyro[1]],[-gyro[2],0,gyro[0]],[gyro[1],-gyro[0],0]])	
	P_c = P_p
	for i in range(9):
	  	P_c = P_c + (0.001/10)*(np.dot(A,P_c)+np.dot(P_c,np.transpose(A))+Q)
	
	L = P_c*np.linalg.inv(R+P_c)	
	velocity_k = velocity_k_p+ np.dot(L,np.transpose(velocity)-velocity_k_p)
	P_p = np.dot((np.identity(3)-L),P_c)

	velocity_k_p = velocity_k
	Rb = np.array([[cos(pitch)*cos(yaw_mocap), cos(pitch)*sin(yaw_mocap), -sin(pitch)], [sin(roll)*sin(pitch)*cos(yaw_mocap)-cos(roll)*sin(yaw_mocap), sin(roll)*sin(pitch)*sin(yaw_mocap)+cos(roll)*cos(yaw_mocap), sin(roll)*cos(pitch)], [cos(roll)*sin(pitch)*cos(yaw_mocap)+sin(roll)*sin(yaw_mocap), cos(roll)*sin(pitch)*sin(yaw_mocap)-sin(roll)*cos(yaw_mocap), cos(roll)*cos(pitch)]])
   	velocity_k_b = np.dot(Rb,velocity_k)


	#Yaw rate from MoCap
	alpha1= 0.4
	yaw_rate = alpha1*yaw_rate_f_p + (1-alpha1)*yaw_rate_i_p
	
	yaw_mocap_i = yaw_mocap	
		
	yaw_rate_i_p = (yaw_mocap_i - yaw_mocap_p)/0.01
	yaw_rate_f_p = yaw_rate
	
	yaw_mocap_p = yaw_mocap_i	
	
	################################# update final data through assignment ################
	fin_data1.header = rb.header
	fin_data1.linear_acceleration.x = velocity_k[0]
	fin_data1.linear_acceleration.y = velocity_k[1]
	fin_data1.linear_acceleration.z = velocity_k[2]
	
	
	fin_data1.orientation.x = yaw_mocap*180/pi
	fin_data1.orientation.y = rb.pose.position.x
	fin_data1.orientation.z = rb.pose.position.y
	fin_data1.orientation.w = rb.pose.position.z
	
	fin_data1.angular_velocity.x = velocity[0]
	fin_data1.angular_velocity.y = velocity[1]
	fin_data1.angular_velocity.z = velocity[2]
	
	#fin_data1.angular_velocity.x = acc[0]
	#fin_data1.angular_velocity.y = acc[1]
	#fin_data1.angular_velocity.z = acc[2]
	
	#fin_data1.angular_velocity.x = velocity[0]
	#fin_data1.angular_velocity.y = velocity[1]
	#fin_data1.angular_velocity.z = velocity[2]
	
	fin_data1.orientation_covariance = (battery.data, velocity_k_b[0], velocity_k_b[1], velocity_k_b[2], velocity[0], roll, pitch, yaw_rate, yaw_mocap)
	
	return fin_data1

def routine(): ## main function to run..... creates publishers,subscribers and runs all the above functions to publish the final data to the roscore #####

    rospy.init_node('filtered_accdata', anonymous=False)
    print("connected to Filter")
    global imu_data, rb_data, battery_percent

    #rb_s = message_filters.Subscriber("/vrpn_client_node/crazyflie1/pose",PoseStamped)
    #imu_s = message_filters.Subscriber("/crazyflie1/imu", Imu)
    #ts = message_filters.TimeSynchronizer([rb_s, imu_s], 10)
    #ts.registerCallback(call_rb_data, call_imu_data)

    rospy.Subscriber("pose",PoseStamped, call_rb_data)
    rospy.Subscriber("imu", Imu, call_imu_data)
    rospy.Subscriber("battery", Float32, call_battery)
    pub1 = rospy.Publisher('Filtered_Acc',Imu, queue_size=200)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
	a = rospy.get_rostime()
	# print(a)	
    	fin_data1 = filter_acc(imu_data, rb_data, battery_percent)
    	rospy.loginfo(fin_data1) # this function prints the data on the terminal...also it gets written in the node log file and rosout
	pub1.publish(fin_data1)
	cf_uri = rospy.get_param('uri')
	a = rospy.get_rostime()	
    	rate.sleep()
	# print(a)
	

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
#	time.sleep(23)
	routine()	
