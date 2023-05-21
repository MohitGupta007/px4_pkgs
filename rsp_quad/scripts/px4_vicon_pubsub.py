
'''
Written by Adam Garlow

This node follows a straightforward sub/pub framework to take mocap messages
and publish them on the desired px4 microRTPS bridge topic

MoCap messages received on topic: '/vicon/X500_v2_IRcam/X500_v2_IRcam'

PX4 messages published on topic: '/fmu/vehicle_visual_odometry/in'

'''

# Standard library imports



# Third-party imports
import rclpy
from rclpy.node import Node

from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleOdometry

from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import PoseStamped

from numpy import NaN
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Pubsub node class definition
class MoCapPubSub(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('px4_mocap_pubsub') # Initialize node

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize subscriber to mocap(VICON) topic
        self.mocap_sub = self.create_subscription(RigidBodies, 
            '/rigid_bodies', self.mocap_callback, 10)

        # Initialize subscriber to PX4 timesync topic
        self.timesync_sub = self.create_subscription(TimesyncStatus, 
            "/fmu/out/timesync_status", self.timesync_callback,qos_profile=self.qos_profile)

        # self.odom_sub = self.create_subscription(VehicleOdometry, 
        #     "/fmu/out/vehicle_odometry", self.odom_callback, qos_profile=self.qos_profile)

        self.timesync = 0

        self.i=0

        self.init_x = 0
        self.init_y = 0
        self.init_z = 0

        # Initialize publisher to PX4 vehicle_visual_odometry topic
        self.mocap_pub = self.create_publisher(VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 10)

        #Test publisher to visualize in RViz
        # self.mocap_test_pub = self.create_publisher(PoseStamped, 
        #     '/mocap_px4_pose_check', 10)

        self.get_logger().info("PX4 mocap pub-sub node initialized")


    # Callback for when new mocap message recieved to publish to PX4
    def mocap_callback(self, msg):

        msg_px4 = VehicleOdometry() # Message to be sent to PX4

        msg_px4.timestamp = self.timesync # Set timestamp
        msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample

        # Transfer data from mocap message to PX4 message
        # VICON Front, Left, Up to PX4 Front, Right, Down
        # Position/orientation components
        msg_px4.pose_frame = 2 # FRD from px4 message

        mocap_x = msg.rigidbodies[0].pose.position.x# - self.init_x
        mocap_y = msg.rigidbodies[0].pose.position.y# - self.init_y -
        mocap_z = msg.rigidbodies[0].pose.position.z# - self.init_z
        msg_px4.position = [mocap_x, mocap_z, -mocap_y]

        if(self.i==0):
            self.init_x = msg.rigidbodies[0].pose.position.x
            self.init_y = msg.rigidbodies[0].pose.position.y
            self.init_z = msg.rigidbodies[0].pose.position.z
            self.i=1

        mocap_rx = msg.rigidbodies[0].pose.orientation.x
        mocap_ry = msg.rigidbodies[0].pose.orientation.y
        mocap_rz = msg.rigidbodies[0].pose.orientation.z
        mocap_rw = msg.rigidbodies[0].pose.orientation.w
        msg_px4.q = [mocap_rw, mocap_rx, mocap_rz, -mocap_ry]
        # msg_px4.q = [NaN,NaN,NaN,NaN]
        
        # Velocity components (unknown)
        msg_px4.velocity_frame = 2 # FRD from px4 message
        msg_px4.velocity = [NaN, NaN, NaN]
        msg_px4.angular_velocity = [NaN, NaN, NaN]

        # Variances
        msg_px4.position_variance = [0.0, 0.0, 0.0]
        msg_px4.orientation_variance = [0.0, 0.0, 0.0]
        msg_px4.velocity_variance = [0.0, 0.0, 0.0]

        self.mocap_pub.publish(msg_px4) # Publish to PX4

        

    # def odom_callback(self,msg):
    #     test_px4 = PoseStamped()

    #     # test_px4.header.stamp = self.timesync
    #     test_px4.header.frame_id = "map"

    #     test_px4.pose.position.x = float(msg.position[0])
    #     test_px4.pose.position.y = float(msg.position[1])
    #     test_px4.pose.position.z = float(msg.position[2])

    #     test_px4.pose.orientation.w = float(msg.q[0])
    #     test_px4.pose.orientation.x = float(msg.q[1])
    #     test_px4.pose.orientation.y = float(msg.q[2])
    #     test_px4.pose.orientation.z = float(msg.q[3])


    #     self.mocap_test_pub.publish(test_px4)


    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):

        # self.get_logger().info('CC Timesync')
        # self.get_logger().info(str(msg.timestamp))
        self.timesync = msg.timestamp

def main(args=None):
    rclpy.init(args=args)

    px4_mocap_pubsub = MoCapPubSub()

    rclpy.spin(px4_mocap_pubsub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()