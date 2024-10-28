# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Empty, Bool
# from drone_utils.drone_object import DroneObject
# from time import sleep


# class DronePositionControl(Node):
#     def __init__(self):
#         Node.__init__(self, 'drone_position_control')
#         # DroneObject.__init__(self)

#         self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.pubPosCtrl = self.create_publisher(Bool, '~/posctrl', 10)
#         self.timer = self.create_timer(0.1, self.start_drone)
        
#     def start_drone(self):
#         # Publish takeoff command
#         self.takeoff_publisher.publish(Empty())
#         self.get_logger().info('Drone takeoff')

#         # # Set the m_posCtrl flag to True
#         # bool_msg = Bool()
#         # bool_msg.data = True
#         # self.pubPosCtrl.publish(bool_msg)
#         # self.get_logger().info('Position control mode set to True')

#         # # Send a command to move the drone to a defined pose
#         # self.move_to(10.0, 2.0, 10.0)  # Example pose coordinates

#         # Destroy the timer to avoid repeating
#         self.timer.cancel() 

#     def move_to(self, x: float, y: float, z: float):
#         """
#         Move the drone to a specific position
#         :param x: X position in m
#         :param y: Y position in m
#         :param z: Z position in m
#         :return: True if the command was sent successfully, False if drone is not flying
#         """
#         # if not self.isFlying:
#         #     self.get_logger().warn("Drone is not flying, cannot move.")
#         #     return False

#         twist_msg = Twist()
#         twist_msg.linear.x = x
#         twist_msg.linear.y = y
#         twist_msg.linear.z = z
#         twist_msg.angular.x = 0.0
#         twist_msg.angular.y = 0.0
#         twist_msg.angular.z = 0.0
        
#         self.cmd_vel_publisher.publish(twist_msg)
#         self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')
#         return True


# def main(args=None):
#     rclpy.init(args=args)
#     drone_position_control_node = DronePositionControl()
#     rclpy.spin(drone_position_control_node)
#     drone_position_control_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import rclpy
# from std_msgs.msg import Empty
# from drone_utils.drone_object import DroneObject

# class DronePositionControl(DroneObject):
#     def __init__(self):
#         super().__init__('simple_drone')


#         self.takeOff()
#         self.get_logger().info('Drone takeoff')

#         # Set the m_posCtrl flag to True
#         self.posCtrl(True)
#         self.get_logger().info('Position control mode set to True')

#         # Send a command to move the drone to a defined pose
#         self.move_drone_to_pose(10.0, 2.0, 5.0)  # Example pose coordinates

#     def move_drone_to_pose(self, x, y, z):
#         # Override the move_drone_to_pose method if specific behavior is needed
#         super().moveTo(x, y, z)
#         self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')


# def main(args=None):
#     rclpy.init(args=args)
#     drone_position_control_node = DronePositionControl()
#     rclpy.spin(drone_position_control_node)
#     drone_position_control_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        
        # Publisher to control the drone's movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        
        # Parameters for circular motion
        self.radius = 2.0  # Radius of the circle (radius = 1, radius is one tile)
        self.speed = 0.5   # Linear speed of the drone
        self.delay = 0.1   # Delay between each movement step (adjusts smoothness and radius)
        
        self.move_to_offset()
        # Start moving in a circle
        self.move_in_circle()

    def move_in_circle(self):
        start_time = time.time()
        
        while rclpy.ok():
            # Calculate time-based angle to move in a circle
            elapsed_time = time.time() - start_time
            angle = elapsed_time * (self.speed / self.radius)
            
            # Stop after completing one full circle (2π radians)
            if angle >= 2 * math.pi:
                break
            
            # Define circular motion by updating x and y based on the angle
            linear_vec = Twist()
            linear_vec.linear.x = self.speed * math.cos(angle)   # Forward/Backward
            linear_vec.linear.y = self.speed * math.sin(angle)   # Left/Right
            
            # Publish the twist message
            self.cmd_vel_publisher.publish(linear_vec)
            self.get_logger().info(f"Moving in circle - x: {linear_vec.linear.x}, y: {linear_vec.linear.y}")
            
            # Delay to control the speed and radius of the circular motion
            time.sleep(self.delay)

        # Stop movement after completing the circle
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info("Completed one full circle and stopped.")

    def move_to_offset(self, duration=2):
        # Move to an initial offset position relative to the object
        offset_twist = Twist()
        # offset_twist.linear.x = 0.0
        offset_twist.linear.y = -1.0
        # offset_twist.linear.z = z_offset

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(offset_twist)
            time.sleep(0.1)  # Small delay to control publishing rate

        # Stop movement
        offset_twist.linear.x = 0.0
        offset_twist.linear.y = 0.0
        offset_twist.linear.z = 0.0
        self.cmd_vel_publisher.publish(offset_twist)
        time.sleep(1)  # Wait for stability before starting the circle

def main(args=None):
    rclpy.init(args=args)
    circle_mover = CircleMover()
    rclpy.spin(circle_mover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
