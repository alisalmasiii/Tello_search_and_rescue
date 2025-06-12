import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
#from transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone')
        self.state = 'IDLE'

        # ROS QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(String, '/drone1_comm', self.message_callback, qos_profile)
        self.create_subscription(Point, '/drone1/position', self.follow_callback, qos_profile)
        self.create_subscription(Odometry, '/drone2/odom', self.odom_callback, qos_profile)

        self.cmd_pub = self.create_publisher(Twist, '/drone2/cmd_vel', 10)
        self.msg_pub = self.create_publisher(String, '/drone2_comm', qos_profile)

        self.current_position = Point()
        self.target_position = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        self.yaw = euler[2]  # yaw = rotation around Z-axis
        self.orientation_q = orientation_q
    def message_callback(self, msg):
        if msg.data == 'TAKEOFF_AND_GOTO_SAFE_POINT' and self.state == 'IDLE':
            self.get_logger().info("Received takeoff command.")
            self.state = 'GO_TO_SAFE_POINT'
            self.target_position = Point(x=self.current_position.x, y=self.current_position.y, z=0.8)  # a safe distance from drone 1
            return
        elif msg.data == 'AT_HOME_POSITION':
            self.get_logger().info("Received Landing at Home Signal.")
            self.state = 'LANDING'
            return

    def timer_callback(self):
        if self.state == 'GO_TO_SAFE_POINT':
            self.get_logger().info("Moving to safe point.")
            if self.reached_target(self.target_position):
                return
        elif self.state == 'LANDING':
            self.perform_landing()   
            return     
        elif self.state == 'DONE':
            self.cmd_pub.publish(Twist())
            return    

    def follow_callback(self, msg):
        if self.state == 'FOLLOW':
            self.msg_pub.publish(String(data='READY_TO_FOLLOW'))
            self.move_towards(msg)

    def move_towards(self, target):
        self.get_logger().info("Follow drone 1")
        twist = Twist()
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance < 0.15:
            return

        twist.linear.x = 0.1 * dx
        twist.linear.y = 0.1 * dy
        twist.linear.z = 0.1 * dz

        self.cmd_pub.publish(twist)

    def reached_target(self, target):
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z

        if math.sqrt(dx**2 + dy**2 + dz**2) < 0.3:
            self.get_logger().info("Reached safe point. Sending ready message.")
            self.state = 'FOLLOW'
            return 
        
    def perform_landing(self):
        if self.current_position.z <= 0.2:
            self.get_logger().info("Landed successfully.")
            self.state = 'DONE'
            self.cmd_pub.publish(Twist())
            return

        twist = Twist()
        twist.linear.z = -0.2  # descend slowly
        self.cmd_pub.publish(twist)
        self.get_logger().info("Landing")

def main(args=None):
    rclpy.init(args=args)
    node = FollowerDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
