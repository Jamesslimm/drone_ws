#!/usr/bin/env python3

import math
import time
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
import threading
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import csv
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Now create the subscription with the QoS profile
        self.gps_subscriber = self.create_subscription(
            NavSatFix,                  # Message type
            '/simple_drone/gps/nav',    # Topic name (string)
            self.gps_callback,          # Callback function
            qos_profile                 # QoS profile
        )

        self.subscription_front_camera = self.create_subscription(
            Image,
            '/simple_drone/front_camera/image_raw', #'/simple_drone/front/image_raw',
            self.camera_callback_front,
            10)
        
        self.subscription_front_depth_camera = self.create_subscription(
            PointCloud2,                              
            '/simple_drone/front_camera/points', 
            self.depth_camera_callback_front,       
            10                                       
        )
        
        self.subscription_bottom_camera = self.create_subscription(
            Image,
            '/simple_drone/down_camera/image_raw', #'/simple_drone/bottom/image_raw',
            self.camera_callback_bottom,
            10)
        
        self.subscription_bottom__depth_camera = self.create_subscription(
            PointCloud2,                              # Message type
            '/simple_drone/down_camera/points', # Topic name
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
        self.allign_droppoint = False
        self.max_dist_front = 0.0
        self.min_dist_front = 0.0
        self.max_dist_bottom = 0.0
        self.min_dist_bottom = 0.0
        self.total_mask_area = 0.0
        self.front_counter_manager = CounterManager(limit=200)   # Front counter limit
        self.bottom_counter_manager = CounterManager(limit=150)  # Bottom counter limit

        # Initial GPS and flag values
        self.droppoint_x = 0.0
        self.droppoint_y = 0.0
        self.start_gpd_nav = False
        self.gps_data = None
        self.gps_coordinates_array = None
        self.current_target_index = 0
        self.target_reached = False
        
        # Placeholder for coordinates array
        self.gps_coordinates_array = None

    def gps_callback(self, msg):
        # Update GPS data whenever a new message arrives
        self.gps_data = msg
        self.get_logger().info(f"New gps coordinates received")
        # self.get_logger().info(f"GPS message: {msg}")

        if self.start_gpd_nav:
            if self.current_target_index < len(self.gps_coordinates_array):
                # Get current target coordinates from gps_coordinates_array
                target_coords = self.gps_coordinates_array[self.current_target_index]
                lat, long, alt = target_coords

                alt_diff = self.gps_data.altitude - alt
                lat_diff = self.gps_data.latitude - lat
                long_diff = self.gps_data.longitude - long
                self.get_logger().info(f"Altitude difference: {alt_diff}")
                self.get_logger().info(f"Latitude difference: {lat_diff}")
                self.get_logger().info(f"Longitude difference: {long_diff}")

                # Move towards the target coordinates
                self.move_to_gps(lat_diff, long_diff, alt_diff)

                # If target is reached, proceed to the next one
                if self.target_reached:
                    self.current_target_index += 1
                    self.target_reached = False  # Reset for the next target
            else:
                self.start_gpd_nav = False

    def camera_callback_front(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        if self.mode == 'front':
            
            results = self.model(img)

            # Frame dimensions
            frame_width = img.shape[1]
            frame_height = img.shape[0]
            center_region_width = 40  # Central region width
            center_region_height = 60  # Central region height
            bottom_threshold = frame_height - 30  # 30 pixels from the bottom

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            for r in results:
                masks = r.masks  # Segmentation masks
                confidences = r.boxes.conf

                if masks is not None:
                    for mask, confidence in zip(masks.data, confidences):

                        if confidence > 0.7:
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
        depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if self.distance_front:
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
            plt.imshow(mask_resized, cmap='gray')
            # plt.colorbar(label='Masked Normalized Depth')
            # plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            # plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            # plt.legend()
            plt.title("Depth Intensity Map")
            plt.xlabel("Width")
            plt.ylabel("Height")
            plt.show()

            # Plot the fused depth intensity map with markers
            plt.imshow(fused_depth_image, cmap='gray')
            plt.colorbar(label='Masked Normalized Depth')
            plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            plt.legend()
            plt.title("Front Fused Depth Intensity Map with Max and Min Values in Middle Column")
            plt.xlabel("Width")
            plt.ylabel("Height")
            plt.show()

            self.distance_front = False
            self.distance_front_done = True
            self.mode = 'bottom'

    def camera_callback_bottom(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        if self.mode == 'bottom':
            solar_panel = False
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
                confidences = r.boxes.conf

                if masks is not None:
                    for mask, confidence in zip(masks.data, confidences):

                        if confidence > 0.7:
                            solar_panel = True

                            # if len(masks.data) > 1: 
                            #     linear_vec.z = 1.0 # Move up
                            #     cmd_vel_zero = False

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
                                    
                                    if not self.allign_droppoint:
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

                                    else:
                                        # Calculate bounding box around the mask
                                        x, y, w, h = cv2.boundingRect(self.mask_np)

                                        # Center of the bounding box
                                        bbox_top_corner_x = x + w
                                        bbox_top_corner_y = y #+ h

                                        # Apply offset to the center of the bounding box
                                        center_x = int(bbox_top_corner_x - self.droppoint_x)
                                        center_y = int(bbox_top_corner_y + self.droppoint_y)

                                        # Determine x and y offsets from the center of the frame to the target point
                                        # offset_x = target_x - frame_width // 2
                                        # offset_y = target_y - frame_height // 2

                                        central_x_min = (frame_width - center_region_width) // 2
                                        central_x_max = (frame_width + center_region_width) // 2
                                        central_y_min = (frame_height - center_region_height) // 2
                                        central_y_max = (frame_height + center_region_height) // 2

                                        # Adjust linear_vec based on the offset to move the drone towards the target point
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
                                            self.bottom_counter_manager.reset()

                                            if self.allign_droppoint:
                                                if self.gps_data:
                                                    altitude_when_done = self.gps_data.altitude
                                                    gps_lat_when_done = self.gps_data.latitude
                                                    gps_long_when_done = self.gps_data.longitude

                                                    # Define GPS coordinate array with the specified points
                                                    self.gps_coordinates_array = np.array([
                                                        # [gps_lat_when_done, gps_long_when_done, altitude_when_done],  # Current GPS data
                                                        [0, 0, altitude_when_done],  # Zero lat/long, current altitude
                                                        [0, 0, 0],  # Origin
                                                        [0, 0, altitude_when_done],  # Zero lat/long, current altitude
                                                        [gps_lat_when_done, gps_long_when_done, altitude_when_done]  # Current GPS data
                                                    ])

                                                    self.get_logger().info(f"GPS Coordinates Array: {self.gps_coordinates_array}")

                                                self.distance_bottom_done = True
                                                self.start_gpd_nav = True
                                            else:
                                                self.distance_bottom = True

                                    else:
                                        self.bottom_counter_manager.reset()

                else:
                    linear_vec.x = 0.5  # Move forward
                    linear_vec.z = 1.0  # Move up

                if not self.distance_bottom_done:
                    twist = Twist(linear=linear_vec, angular=angular_vec)
                    self.cmd_vel_publisher.publish(twist)

            annotated_frame = results[0].plot()
            if solar_panel:#if masks is not None:
                # Mark the offset point on the frame
                cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 165, 255), -1)  # Orange marker
                cv2.putText(annotated_frame, "Offset Point", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    def depth_camera_callback_bottom(self, msg):
        depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if self.distance_bottom:
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

            # Calculate the height in pixels of the solar panel in the depth image
            height_pixels = abs(max_position[0] - min_position[0])
            
            # Calculate the real-world height scale
            real_world_height = abs(self.max_dist_front - self.min_dist_front)
            scale = real_world_height / height_pixels  # Real world meters per pixel
            
            # Find the maximum width of the mask in pixels
            masked_columns = np.any(mask_resized > 0, axis=0)
            max_width_start = np.argmax(masked_columns)
            max_width_end = image_width - np.argmax(masked_columns[::-1]) - 1
            max_width_pixels = max_width_end - max_width_start
            
            # Calculate the real-world width by multiplying the pixel width by the scale
            real_world_width = max_width_pixels * scale

            # Logging the results
            self.get_logger().info(f"Real-world height: {real_world_height} meters")
            self.get_logger().info(f"Scale (meters per pixel): {scale}")
            self.get_logger().info(f"Max width of mask in pixels: {max_width_pixels}")
            self.get_logger().info(f"Real-world width of the mask: {real_world_width} meters")

            # Convert y = 0.025 meters to plot pixels
            y_real = 0.025  # meters
            self.droppoint_y = y_plot = y_real / scale  # pixels

            # Calculate x in plot coordinates using 30 degrees
            angle_degrees = self.plot_right_angle_triangle(self.max_dist_bottom, self.min_dist_bottom, self.max_dist_front, self.min_dist_front)
            angle_radians = np.deg2rad(angle_degrees)
            self.droppoint_x = x_plot = y_plot * np.tan(angle_radians)

            # Calculate the plot position based on the middle of the image
            plot_x = int(max_width_end - x_plot)
            plot_y = int(min_position[0] + y_plot)

            plt.imshow(mask_resized, cmap='gray')
            # plt.colorbar(label='Masked Normalized Depth')
            # plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            # plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            # plt.legend()
            plt.title("Depth Intensity Map")
            plt.xlabel("Width")
            plt.ylabel("Height")
            plt.show()

            # Plot the fused depth intensity map with markers
            plt.imshow(fused_depth_image, cmap='gray')
            plt.colorbar(label='Masked Normalized Depth')
            
            # Mark max and min positions for height calculation
            plt.scatter(max_position[1], max_position[0], color='red', label='Max Value')
            plt.scatter(min_position[1], min_position[0], color='blue', label='Min Value')
            
            # Mark the points used to measure maximum width
            plt.scatter(max_width_start, min_position[0], color='green', label='Width Start')
            plt.scatter(max_width_end, min_position[0], color='purple', label='Width End')

            # Plot the calculated point using the given y and angle x
            plt.scatter(plot_x, plot_y, color='orange', label='Calculated Point (y=0.025m, x=30°)')

            # Plot details
            plt.legend()
            plt.title("Bottom Fused Depth Intensity Map with Max and Min Values in Middle Column")
            plt.xlabel("Width")
            plt.ylabel("Height")
            plt.show()

            self.distance_bottom = False
            self.allign_droppoint = True
            # Call the plotting function with parameters
            # self.plot_right_angle_triangle(self.max_dist_bottom, self.min_dist_bottom, self.max_dist_front, self.min_dist_front)

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
        
        self.get_logger().info(f"Angle of inclination: {angle_opposite_height} degrees")

        # Plotting the triangle
        # plt.figure()
        # plt.plot(x_vals, y_vals, 'bo-', label='Depth Triangle')
        # plt.fill(x_vals, y_vals, color='skyblue', alpha=0.5)  # Optional fill for visualization

        # # Annotate the angles
        # plt.text(0.5 * length, 0.1 * height, f'{angle_opposite_height:.2f}°', ha='center', color='purple')
        # plt.text(0.1 * length, 0.5 * height, f'{angle_opposite_length:.2f}°', ha='center', color='purple')
        # plt.text(0.1 * length, 0.1 * height, '90°', ha='center', color='purple')

        # Plot labels and title
        # plt.xlabel("Front Depth Difference")
        # plt.ylabel("Bottom Depth Difference")
        # plt.title("Right Angle Triangle based on Depth Values with Angles")
        # plt.legend()
        # plt.grid()
        # plt.show()
        
        return angle_opposite_height

    def move_to_gps(self, lat, long, alt):
        """Move the drone to a GPS coordinate (latitude, longitude, altitude)."""
        if not self.gps_data:
            self.get_logger().info(f"GPS info not received")
            return  # Wait until GPS data is available

        # Threshold to check if the target is reached
        alt_reach = False
        lat_reach = False
        long_reach = False
        reach_threshold = 0.00000005
        z_reach_threshold = 0.1

        # Set the linear movement based on differences
        linear_vec = Vector3()
        if abs(alt) < z_reach_threshold:
            linear_vec.z = 0.0
            alt_reach = True
        elif alt > 0:
            linear_vec.z = -0.3
        else:
            linear_vec.z = 0.3

        if abs(lat) < reach_threshold:
            linear_vec.y = 0.0
            lat_reach = True
        elif lat > 0:
            linear_vec.y = 0.1
        else:
            linear_vec.y = -0.1

        if abs(long) < reach_threshold:
            linear_vec.x = 0.0
            long_reach = True
        elif long > 0:
            linear_vec.x = 0.1
        else:
            linear_vec.x = -0.1

        # Publish movement commands
        twist = Twist(linear=linear_vec, angular=Vector3())
        self.cmd_vel_publisher.publish(twist)

        # Check if all coordinates (alt, lat, long) are within reach threshold
        if alt_reach and lat_reach and long_reach:
            self.get_logger().info("Target reached. Moving to next point.")
            self.target_reached = True
        else:
            self.target_reached = False

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

