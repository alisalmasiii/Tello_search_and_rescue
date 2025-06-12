import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
#from transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from math import atan2, sin, cos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
import time

class BoundedSearchWithMarker(Node):
    def __init__(self):
        super().__init__('bounded_search_with_marker')

        # ROS QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.create_subscription(Odometry, '/drone1/odom', self.odom_callback, qos_profile)
        self.create_subscription(Image, '/drone1/image_raw', self.image_callback, qos_profile)
        # Communication to drone 2
        self.msg_pub = self.create_publisher(String, '/drone1_comm', qos_profile)
        self.ready_sub = self.create_subscription(String, '/drone2_comm', self.message_callback, qos_profile)
        self.pos_pub = self.create_publisher(Point, '/drone1/position', qos_profile)  # Position broadcasting

        # State
        self.state = 'IDLE'
        self.bridge = CvBridge()
        self.position = Point()
        self.yaw = 0.0
        self.orientation_q = None

        # Home and target tracking
        self.home_position = None
        self.target_position = None

        # Bounded area
        self.bound_x_min = 0.0
        self.bound_x_max = 30.0
        self.bound_y_min = 0.0
        self.bound_y_max = 30.0

        #obstacle avoidance 
        self.obstacle_detected = False
        self.lost_drone= 0

        # Generate lawnmower path
        self.waypoints = self.generate_lawnmower_waypoints(step_size=1.0)
        self.current_waypoint_index = 0
        self.goal = self.waypoints[self.current_waypoint_index] if self.waypoints else Point()

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Initialized drone node with marker-based search.")

    def generate_lawnmower_waypoints(self, step_size):
        waypoints = []
        y = self.bound_y_min
        flip = False
        while y <= self.bound_y_max:
            x_range = reversed([self.bound_x_min + i * step_size for i in range(int((self.bound_x_max - self.bound_x_min) / step_size) + 1)]) if flip else \
                      [self.bound_x_min + i * step_size for i in range(int((self.bound_x_max - self.bound_x_min) / step_size) + 1)]
            for x in x_range:
                pt = Point()
                pt.x = x
                pt.y = y
                waypoints.append(pt)
            y += step_size
            flip = not flip
            #print(waypoints)
        return waypoints

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        self.yaw = euler[2]  # yaw = rotation around Z
        self.orientation_q = orientation_q


    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))
    
    def message_callback(self, msg):
        if msg.data == 'READY_TO_FOLLOW':
            self.get_logger().info("Drone 2 is ready. Begin position sharing.")
            self.state = 'RETURN_HOME'
            self.waypoints = [self.home_position]
            self.current_waypoint_index = 0

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers (unchanged)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            self.get_logger().info(f"Detected ArUco marker IDs: {ids.flatten()}")

        # HSV-based tree (green object) detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Adjusted HSV range for green
        lower = (0, 25, 30)
        upper = (22, 100, 120)
        mask = cv2.inRange(hsv, lower, upper)
        '''
        # Morphological operations to reduce noise
        kernel = np.ones((4, 4), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        '''
        # Find contours and filter by area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2900:  # Ignore small green regions
                self.obstacle_detected = True
                self.state = 'OBSTACLE_DETECTED'
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Tree", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.get_logger().info(f"Tree detected with area {area:.1f} at x={x}, y={y}")

        if ids is not None:
            for marker_id in ids.flatten():
                if marker_id == 0:
                    if self.state == 'IDLE':
                        self.get_logger().info("Marker ID 0 seen. Starting search.")
                        self.home_position = Point(x=self.position.x, y=self.position.y, z=self.position.z)
                        self.state = 'SEARCH'
                    elif self.state == 'RETURN_HOME':
                        self.get_logger().info("Returned to Marker ID 0. Mission complete.")
                        self.state = 'LANDING'
                elif marker_id == 1:
                    self.get_logger().info("Target Marker ID 1 found. Sending message to drone2.")
                    self.target_position = Point(x=self.position.x, y=self.position.y, z=self.position.z)
                    self.state = 'WAIT_FOR_DRONE2'
                    self.lost_drone += 1

        # Display the debug image (optional)
        cv2.imshow("Detection", frame)
        cv2.waitKey(1)

    def control_loop(self):
        if self.state == 'IDLE':
            self.cmd_pub.publish(Twist())  # wait for ID 0
            return
        if self.state == 'WAIT_FOR_DRONE2':
            if self.lost_drone == 1:
                twist = Twist()
                twist.linear.x = 0.3
                self.cmd_pub.publish(twist)
                time.sleep(0.15)
                self.lost_drone += 1
            self.cmd_pub.publish(Twist())  # wait for message
            # Send message to Drone 2 to take off and go to safe location
            self.msg_pub.publish(String(data='TAKEOFF_AND_GOTO_SAFE_POINT'))
            self.pos_pub.publish(self.position)
            return
        elif self.state == 'LANDING':
            self.perform_landing()
            self.msg_pub.publish(String(data='AT_HOME_POSITION'))
            return
        elif self.state == 'DONE':
            self.cmd_pub.publish(Twist())  # completely done
            return
        
        if self.state == 'RETURN_HOME':
            # Broadcast current position to drone 2
            self.pos_pub.publish(self.position)

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached_Lost drone not found")
            self.cmd_pub.publish(Twist())
            return

        # Obstacle avoidance logic
        if self.state == 'OBSTACLE_DETECTED':
            self.get_logger().warn("obstacle detected! Avoiding...")
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
            twist = Twist()
            twist.linear.y = -0.33
            twist.linear.x = 0.2
            self.cmd_pub.publish(twist)
            self.goal = self.waypoints[self.current_waypoint_index]
            inc_x = self.goal.x - self.position.x
            inc_y = self.goal.y - self.position.y
            distance = (inc_x ** 2 + inc_y ** 2) ** 0.5
            self.state = 'SEARCH'
            time.sleep(0.25)

            twist = Twist()
            if distance > 2:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
                self.current_waypoint_index += 3
                return
            self.obstacle_detected = False

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached_Lost drone not found")
            self.cmd_pub.publish(Twist())
            return
        
        self.goal = self.waypoints[self.current_waypoint_index]
        inc_x = self.goal.x - self.position.x
        inc_y = self.goal.y - self.position.y
        distance = (inc_x ** 2 + inc_y ** 2) ** 0.5
        angle_to_goal = atan2(inc_y, inc_x)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw)
        #print(self.yaw)

        twist = Twist()
        if distance < 0.3:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
            self.current_waypoint_index += 1
        elif abs(angle_diff) > 0.1:
            twist.angular.z = 0.2 * angle_diff / abs(angle_diff)
        else:
            twist.linear.x = 0.23

        self.cmd_pub.publish(twist)

    def perform_landing(self):
        if self.position.z <= 0.1:
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
    node = BoundedSearchWithMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
