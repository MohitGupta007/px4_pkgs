"""
Python implementation of Offboard Control

"""


import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleOdometry

from aruco_msgs.msg import MarkerArray

from math import sqrt


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.status_sub = self.create_subscription(VehicleOdometry,
                                                   '/fmu/out/vehicle_odometry',
                                                   self.vehicle_odom_callback,
                                                   qos_profile)

        self.markers_sub = self.create_subscription(MarkerArray,
                                                    '/camera/marker_publisher/markers',
                                                    self.markers_callback,
                                                    qos_profile=0)
        self.markers_data_list = []
        self.current_markers_list = []
        self.current_mission_status = "TAKEOFF"
        self.goal = [] # des_x, des,y, des_z, des_yaw
        self.first_valid_odom = False
        self.first_veh_pos = []

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0

        self.timer_period = 0.02  
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)

    def vehicle_odom_callback(self,msg):
        self.veh_pos = msg.position
        if(not self.first_valid_odom):
            self.first_veh_pos = msg.position
            self.first_valid_odom = True

    def markers_callback(self,msg):
        self.markers_data_list = msg.markers
        self.current_markers_list = []
        for i in msg.markers:
            self.current_markers_list.append(i.id)


    def timer_callback(self):
        if(not self.first_valid_odom):
            self.get_logger().info("Waiting to receive first valid odom.")
            return

        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()
        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        if(self.current_mission_status=="TAKEOFF"):
            self.takeoff()
        elif(self.current_mission_status=="ARUCO1"):
            self.move_to_aruco(ar_id = 1)
        elif(self.current_mission_status=="MOVE"):
            self.move_ahead()
        elif(self.current_mission_status=="ARUCO2"):
            self.move_to_aruco(ar_id = 2)
        elif(self.current_mission_status=="PRE_LAND"):
            self.precision_land(ar_id = 2)
        elif(self.current_mission_status=="LAND"):
            self.land()
        else:
            self.get_logger().info("Mission Complete.")
            exit()


    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    # Takeoff vehicle
    def takeoff(self):
        # self.goal = [self.first_veh_pos[0],self.first_veh_pos[1],-2,0.0]
        self.goal = [0.0,-0.2,-2,0.0]
        if(self.publish_offboard_control_mode(mode="position")):
            self.current_mission_status = "ARUCO1"
            self.get_logger().info("Takeoff complete.")
            self.get_logger().info("Moving to first aruco marker." )

    # Hover over aruco marker
    def move_to_aruco(self,ar_id):
        try:
            ar_idx = self.current_markers_list.index(ar_id)
        except:
            self.get_logger().info("Could not find aruco.")
            self.publish_offboard_control_mode(mode="position")
            return

        ar_pos = self.markers_data_list[ar_idx].pose.pose.position
        self.goal = [self.veh_pos[0]-5*ar_pos.x*self.timer_period,self.veh_pos[1]-5*ar_pos.y*self.timer_period,-2.0,0.0]
        self.publish_offboard_control_mode(mode="position")
        err = sqrt((ar_pos.x)**2 + (ar_pos.y)**2)
        print("Error to aruco: ",err)
        if(err<0.3 and ar_id==1):
            self.current_mission_status = "MOVE"
            self.get_logger().info("Over aruco id: " + str(ar_id))
            self.get_logger().info("Moving forward...")
        elif(err<0.3 and ar_id==2):
            self.current_mission_status = "PRE_LAND"
            self.get_logger().info("Over aruco id: " + str(ar_id))
            self.get_logger().info("Starting precision landing...")

    # Moving in +x direction with constant speed
    def move_ahead(self):
        self.goal = [self.veh_pos[0],self.veh_pos[1]+0.3,-2.0,0.0]
        self.publish_offboard_control_mode(mode="position")
        if 2 in self.current_markers_list:
            self.current_mission_status = "ARUCO2"

    def precision_land(self,ar_id):
        try:
            ar_idx = self.current_markers_list.index(ar_id)
        except:
            self.get_logger().info("Could not find aruco.")
            self.get_logger().info("Landing...")
            self.current_mission_status = "LAND"
            return

        ar_pos = self.markers_data_list[ar_idx].pose.pose.position
        print(ar_pos.z)
        self.goal = [self.veh_pos[0]-5*ar_pos.x*self.timer_period,self.veh_pos[1]-5*ar_pos.y*self.timer_period,self.veh_pos[2]+0.1,0.0]
        self.publish_offboard_control_mode(mode="position")
        if(abs(ar_pos.z)<0.7):
            self.get_logger().info("Landing...")
            self.current_mission_status = "LAND"

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,0.0)
        self.get_logger().info("Land command sent")
        self.current_mission_status="COMPLETED"

    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''
    def publish_offboard_control_mode(self,mode):
        offb_msg = OffboardControlMode()
        traj_msg = TrajectorySetpoint()
        
        if(mode == 'position'):
            offb_msg.position = True
            offb_msg.velocity = False
            offb_msg.acceleration = False
            offb_msg.attitude = False
            offb_msg.body_rate = False
            traj_msg.position = [float(self.goal[0]),float(self.goal[1]),float(self.goal[2])]
            err = sqrt((self.goal[0]-self.veh_pos[0])**2 + (self.goal[1]-self.veh_pos[1])**2 + (self.goal[2]-self.veh_pos[2])**2)

        elif(mode == 'velocity'):
            offb_msg.position = False
            offb_msg.velocity = True
            offb_msg.acceleration = False
            offb_msg.attitude = False
            offb_msg.body_rate = False
            traj_msg.velocity = self.goal[0:3]
            err = 0.01

        traj_msg.yaw = 1.57 #self.goal[3]  # [-PI:PI]
        traj_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        offb_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        
        self.offboard_control_mode_publisher_.publish(offb_msg)
        self.trajectory_setpoint_publisher_.publish(traj_msg)

        if(err<0.2):
            return True
        else:
            return False

    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
