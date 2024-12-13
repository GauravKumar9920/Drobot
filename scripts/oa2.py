#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class LaserObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('laser_obstacle_avoidance')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

        self.goal_x = 5.0
        self.goal_y = 7.0
        self.goal_z = 0.0   
        self.current_x = 0 
        self.current_y = 0
        self.angle_diff = 0
        self.x_vel = 0


    def quaternion_to_euler(self, x, y, z, w):
        import math
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # X = math.atan2(t0, t1)

        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return Z

    def odom_callback(self, msg):
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y   

        current_orientation_x = msg.pose.pose.orientation.x
        current_orientation_y = msg.pose.pose.orientation.y
        current_orientation_z = msg.pose.pose.orientation.z
        current_orientation_w = msg.pose.pose.orientation.w

        current_yaw = self.quaternion_to_euler(current_orientation_x, current_orientation_y, current_orientation_z, current_orientation_w)  

        distance = self.get_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)    
        
        print(self.angle_diff)
        global x_vel
        if distance < 1:
            self.x_vel = 0.0
            self.angle_diff = 0.0
        else:
            self.x_vel = 0.5
            self.angle_diff = self.get_angle(self.current_x, self.current_y, self.goal_x, self.goal_y) - current_yaw - 1.57
        print("distance diff: ", distance, self.current_x, self.current_y, self.goal_x, self.goal_y)

    def laser_callback(self, msg):
        num_regions = 5  # Number of desired regions
        region_width = (msg.angle_max - msg.angle_min) / num_regions  

        regions = []
        for i in range(num_regions):
            start_index = int(i * len(msg.ranges) / num_regions)
            end_index = int((i + 1) * len(msg.ranges) / num_regions)
            min_distance = min(min(msg.ranges[start_index:end_index]), 10)
            regions.append(min_distance)
        self.get_logger().info(str(regions))  # Convert to string for logging
        
        self.take_action(regions)

    def get_distance(self, x1, y1, x2, y2):
        return ((x2 - x1)**2 + (y2 - y1)**2)**0.5   
    
    def get_angle(self, x1, y1, x2, y2):
        import math
        return math.atan2(y2 - y1, x2 - x1)

    def take_action(self, regions):
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''

        threshold = 0.3  # Obstacle detection threshold



        if all(region > threshold for region in regions):  
            state_description = 'case 1 - nothing'
            linear_x = 0.5*(float(self.x_vel)) 
            # linear_x = 0
            # linear_y = 0.5
            angular_z = 3*(float(self.angle_diff))

        elif regions[2] < threshold:  # Check 'front' region directly
            state_description = 'case 2 - front'
            linear_x = 0.5
            # linear_y = 0.1
            angular_z = -0.1
            
        elif regions[1] < threshold:  # 'fright'
            state_description = 'case 3 - fright'
            # linear_x = -0.8
            # linear_y = 0.1
            linear_x = 0 + float(self.x_vel)
            angular_z = -0.3 + 0.1*(float(self.angle_diff))
        elif regions[3] < threshold:  # 'fleft'
            state_description = 'case 4 - fleft'
            linear_x = 0.8
            # linear_y = 0.1
            angular_z = 0.3  + 0.1*(float(self.angle_diff))
        else:
            linear_x = -0.5
            linear_y = 0.1


        self.get_logger().info(state_description)
        msg.linear.x = float(linear_x)
        # msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
