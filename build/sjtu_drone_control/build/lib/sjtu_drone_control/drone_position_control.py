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

import rclpy
from std_msgs.msg import Empty
from drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('simple_drone')


        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        # Send a command to move the drone to a defined pose
        self.move_drone_to_pose(10.0, 2.0, 5.0)  # Example pose coordinates

    def move_drone_to_pose(self, x, y, z):
        # Override the move_drone_to_pose method if specific behavior is needed
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')


def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl()
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()