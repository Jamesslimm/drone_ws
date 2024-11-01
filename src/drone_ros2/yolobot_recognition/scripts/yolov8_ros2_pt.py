#!/usr/bin/env python3

import math
import time
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
import threading
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import csv
import sensor_msgs_py.point_cloud2 as pc2

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class CounterManager:
    def __init__(self, limit):
        self.counter = 0
        self.limit = limit
        self.active = False

    def increment(self):
        if self.active:
            self.counter += 1
            return self.counter >= self.limit
        return False

    def reset(self):
        self.counter = 0
        self.active = False

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # Initialize segmentation model for detecting solar panels
        self.mode = 'front'
        self.model = YOLO('/home/james/drone_ws/src/drone_ros2/yolobot_recognition/scripts/solar_panel.pt')
        self.yolov8_inference = Yolov8Inference()

        self.mask_np = np.zeros((360, 640), dtype=np.uint8)

        self.subscription_front_camera = self.create_subscription(
            Image,
            '/simple_drone/depth_front/image_raw', #'/simple_drone/front/image_raw',
            self.camera_callback_front,
            10)
        
        self.subscription_front_depth_camera = self.create_subscription(
            PointCloud2,                              
            '/simple_drone/depth_front/points', 
            self.depth_camera_callback_front,       
            10                                       
        )
        
        self.subscription_bottom_camera = self.create_subscription(
            Image,
            '/simple_drone/depth_bottom/image_raw', #'/simple_drone/bottom/image_raw',
            self.camera_callback_bottom,
            10)
        
        self.subscription_bottom__depth_camera = self.create_subscription(
            PointCloud2,                              # Message type
            '/simple_drone/depth_bottom/points', # Topic name
            self.depth_camera_callback_bottom,         # Callback function
            10                                         # Queue size
        )
        
        # Publishers
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1)

        # Parameters for object allignment
        self.counter = 0
        self.counter_limit = 200  # Adjust as needed for the duration threshold
        self.mode_switched = False

        # Parameters
        self.distance_front = False
        self.distance_front_done = False
        self.distance_bottom = False
        self.distance_bottom_done = False
        self.max_dist_front = 0.0
        self.min_dist_front = 0.0
        self.max_dist_bottom = 0.0
        self.min_dist_bottom = 0.0
        self.total_mask_area = 0.0
        self.front_counter_manager = CounterManager(limit=200)   # Front counter limit
        self.bottom_counter_manager = CounterManager(limit=150)  # Bottom counter limit

    def camera_callback_front(self, data):
        if self.mode == 'front':
            
            img = bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model(img)

            # Frame dimensions
            frame_width = img.shape[1]
            frame_height = img.shape[0]
            center_region_width = 40  # Central region width
            center_region_height = 40  # Central region height
            bottom_threshold = frame_height - 30  # 30 pixels from the bottom

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            for r in results:
                masks = r.masks  # Segmentation masks

                if masks is not None:
                    for mask in masks.data:
                        # Calculate the centroid of the mask
                        self.mask_np = mask.cpu().numpy().astype('uint8')
                        M = cv2.moments(self.mask_np)
                        if M['m00'] != 0:
                            center_x = int(M['m10'] / M['m00'])
                            center_y = int(M['m01'] / M['m00'])

                            # Define movement logic
                            linear_vec = Vector3()
                            angular_vec = Vector3()

                            # Step 1: Center the object in the frame (x and y axes)
                            central_x_min = (frame_width - center_region_width) // 2
                            central_x_max = (frame_width + center_region_width) // 2
                            central_y_min = (frame_height - center_region_height) // 2
                            central_y_max = (frame_height + center_region_height) // 2

                            if center_x < central_x_min:
                                linear_vec.y = 0.2  # Move left
                            elif center_x > central_x_max:
                                linear_vec.y = -0.2  # Move right

                            if center_y < central_y_min:
                                linear_vec.z = 0.2  # Move up
                            elif center_y > central_y_max:
                                linear_vec.z = -0.2  # Move down
                                
                            if (central_x_min <= center_x <= central_x_max and 
                                central_y_min <= center_y <= central_y_max):
                                self.counter += 1
                                if self.counter >= self.counter_limit:
                                    self.distance_front = True
                                    self.counter = 0


                            twist = Twist(linear=linear_vec, angular=angular_vec)
                            self.cmd_vel_publisher.publish(twist)

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    def depth_camera_callback_front(self, msg):
        if self.distance_front:
            depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            depth_data = np.array([point[2] for point in depth_points])
            image_width = 640
            image_height = 360

            depth_image = depth_data.reshape((image_height, image_width))
            depth_image_normalized = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))

            mask_resized = cv2.resize(self.mask_np, (image_width, image_height))

            # Set all points outside the mask to np.nan
            fused_depth_image = np.where(mask_resized > 0, depth_image_normalized, np.nan)

            # Define the middle column index
            middle_column_index = image_width // 2

            # Extract only the middle column within the mask
            middle_column = fused_depth_image[:, middle_column_index]

            # Find the max and min values in the middle column within the mask
            self.max_dist_front = np.nanmax(middle_column)
            self.min_dist_front = np.nanmin(middle_column)
            
            max_position = (np.nanargmax(middle_column), middle_column_index)
            min_position = (np.nanargmin(middle_column), middle_column_index)

            self.get_logger().info(f"Largest value in middle column of fused depth image: {self.max_dist_front} at position: {max_position}")
            self.get_logger().info(f"Smallest value in middle column of fused depth image: {self.min_dist_front} at position: {min_position}")

            # Plot the fused depth intensity map with markers
            # plt.imshow(fused_depth_image, cmap='gray')
            # plt.colorbar(label='Masked Normalized Depth')
            # plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            # plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            # plt.legend()
            # plt.title("Front Fused Depth Intensity Map with Max and Min Values in Middle Column")
            # plt.xlabel("Width")
            # plt.ylabel("Height")
            # plt.show()

            self.distance_front = False
            self.distance_front_done = True
            self.mode = 'bottom'

    def camera_callback_bottom(self, data):
        if self.mode == 'bottom':

            img = bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model(img)

            # Define movement logic
            linear_vec = Vector3()
            angular_vec = Vector3()
            
            # Frame dimensions
            frame_width = img.shape[1]
            frame_height = img.shape[0]
            center_region_width = 40
            center_region_height = 40

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            cmd_vel_zero = True  # Track if cmd_vel publishes all zeros
            for r in results:

                masks = r.masks  # Segmentation masks

                if masks is not None:

                    if len(masks.data) > 1: 
                        linear_vec.z = 1 # Move up
                        cmd_vel_zero = False

                    for mask in masks.data:
                        # Calculate the centroid of the mask
                        self.mask_np = mask.cpu().numpy().astype('uint8') * 255 

                        # Calculate mask area and add it to the total
                        mask_area = cv2.countNonZero(self.mask_np)
                        self.total_mask_area += mask_area

                        # Calculate object height in the frame
                        mask_coords = np.where(self.mask_np > 0)
                        if mask_coords[0].size > 0:  # Ensure the mask is not empty
                            object_height = mask_coords[0].max() - mask_coords[0].min()

                            # Check if object height is at least three times smaller than frame height
                            if object_height >= frame_height / 2:
                                linear_vec.z = 0.2  # Move up
                                cmd_vel_zero = False

                        M = cv2.moments(self.mask_np)
                        if M['m00'] != 0:
                            center_x = int(M['m10'] / M['m00'])
                            center_y = int(M['m01'] / M['m00'])

                            # Define the central region boundaries
                            central_x_min = (frame_width - center_region_width) // 2
                            central_x_max = (frame_width + center_region_width) // 2
                            central_y_min = (frame_height - center_region_height) // 2
                            central_y_max = (frame_height + center_region_height) // 2

                            # Move towards the center
                            if center_x < central_x_min:
                                linear_vec.y = 0.2  # Move left
                                cmd_vel_zero = False
                            elif center_x > central_x_max:
                                linear_vec.y = -0.2  # Move right
                                cmd_vel_zero = False

                            if center_y < central_y_min:
                                linear_vec.x = 0.2  # Move forward
                                cmd_vel_zero = False
                            elif center_y > central_y_max:
                                linear_vec.x = -0.2  # Move backward
                                cmd_vel_zero = False

                            # Increment or reset counter based on cmd_vel status
                            if cmd_vel_zero and (self.distance_bottom_done == False):
                                self.bottom_counter_manager.activate()
                                if self.bottom_counter_manager.increment():
                                    self.distance_bottom = True
                                    self.bottom_counter_manager.reset()
                            else:
                                self.bottom_counter_manager.reset()

                else:
                    linear_vec.x = 0.5  # Move forward
                    linear_vec.z = 1.0  # Move up

                if not self.distance_bottom_done:
                    twist = Twist(linear=linear_vec, angular=angular_vec)
                    self.cmd_vel_publisher.publish(twist)

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    def depth_camera_callback_bottom(self, msg):
        if self.distance_bottom:
            depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            depth_data = np.array([point[2] for point in depth_points])
            image_width = 640
            image_height = 360

            depth_image = depth_data.reshape((image_height, image_width))
            depth_image_normalized = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))

            mask_resized = cv2.resize(self.mask_np, (image_width, image_height))

            # Set all points outside the mask to np.nan
            fused_depth_image = np.where(mask_resized > 0, depth_image_normalized, np.nan)

            # Define the middle column index
            middle_column_index = image_width // 2

            # Extract only the middle column within the mask
            middle_column = fused_depth_image[:, middle_column_index]

            # Find the max and min values in the middle column within the mask
            self.max_dist_bottom = np.nanmax(middle_column)
            self.min_dist_bottom = np.nanmin(middle_column)
            
            max_position = (np.nanargmax(middle_column), middle_column_index)
            min_position = (np.nanargmin(middle_column), middle_column_index)

            self.get_logger().info(f"Largest value in middle column of fused depth image: {self.max_dist_bottom} at position: {max_position}")
            self.get_logger().info(f"Smallest value in middle column of fused depth image: {self.min_dist_bottom} at position: {min_position}")

            # Plot the fused depth intensity map with markers
            # plt.imshow(fused_depth_image, cmap='gray')
            # plt.colorbar(label='Masked Normalized Depth')
            # plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            # plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            # plt.legend()
            # plt.title("Bottom Fused Depth Intensity Map with Max and Min Values in Middle Column")
            # plt.xlabel("Width")
            # plt.ylabel("Height")
            # plt.show()

            self.distance_bottom = False
            self.distance_bottom_done = True

            # Call the plotting function with parameters
            self.plot_right_angle_triangle(self.max_dist_bottom, self.min_dist_bottom, self.max_dist_front, self.min_dist_front)

    def plot_right_angle_triangle(self, max_dist_bottom, min_dist_bottom, max_dist_front, min_dist_front):
        # Calculate the height and length of the triangle
        height = abs(max_dist_bottom - min_dist_bottom)
        length = abs(max_dist_front - min_dist_front)

        # Calculate the angles in degrees
        angle_opposite_height = math.degrees(math.atan(height / length))
        angle_opposite_length = 90 - angle_opposite_height

        # Define the triangle vertices based on (0, 0) as the right angle
        vertices = [(0, 0), (length, 0), (0, height)]
        
        # Unzip vertices for plotting
        x_vals, y_vals = zip(*vertices, vertices[0])  # Closing the triangle
        
        # Plotting the triangle
        plt.figure()
        plt.plot(x_vals, y_vals, 'bo-', label='Depth Triangle')
        plt.fill(x_vals, y_vals, color='skyblue', alpha=0.5)  # Optional fill for visualization

        # Annotate the angles
        plt.text(0.5 * length, -0.1 * height, f'{angle_opposite_height:.2f}°', ha='center', color='purple')
        plt.text(-0.1 * length, 0.5 * height, f'{angle_opposite_length:.2f}°', ha='center', color='purple')
        plt.text(0.1 * length, 0.1 * height, '90°', ha='center', color='purple')

        # Plot labels and title
        plt.xlabel("Front Depth Difference")
        plt.ylabel("Bottom Depth Difference")
        plt.title("Right Angle Triangle based on Depth Values with Angles")
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

