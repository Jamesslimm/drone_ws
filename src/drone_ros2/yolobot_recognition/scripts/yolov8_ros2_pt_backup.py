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

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # Initialize segmentation model for detecting solar panels
        self.mode = 'front'
        self.model = YOLO('/home/james/drone_ws/src/drone_ros2/yolobot_recognition/scripts/solar_panel.pt')
        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/front/image_raw',
            self.camera_callback,
            10)
        
        self.subscription_bottom = self.create_subscription(
            Image,
            '/simple_drone/depth_bottom/image_raw', #'/simple_drone/bottom/image_raw',
            self.camera_callback_bottom,
            10)
        
        self.subscription_bottom = self.create_subscription(
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

        # Parameters for circular motion
        self.circling = True
        self.radius = 1.0  # Radius of the circle (radius = 1, radius is one tile)
        self.speed = 0.5   # Linear speed of the drone
        self.delay = 0.1   # Delay between each movement step (adjusts smoothness and radius)
        self.total_mask_area = 0.0

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
                        linear_vec.z = 0.2  # Move up
                        cmd_vel_zero = False
                         
                    for mask in masks.data:
                        # Calculate the centroid of the mask
                        mask_np = mask.cpu().numpy().astype('uint8') * 255 

                        # Calculate mask area and add it to the total
                        mask_area = cv2.countNonZero(mask_np)
                        self.total_mask_area += mask_area

                        M = cv2.moments(mask_np)
                        if M['m00'] != 0:
                            center_x = int(M['m10'] / M['m00'])
                            center_y = int(M['m01'] / M['m00'])

                            # Define the central region boundaries
                            central_x_min = (frame_width - center_region_width) // 2
                            central_x_max = (frame_width + center_region_width) // 2
                            central_y_min = (frame_height - center_region_height) // 2
                            central_y_max = (frame_height + center_region_height) // 2

                            if center_x < central_x_min:
                                self.get_logger().info("Move left")
                                linear_vec.y = 0.2  # Move left
                                cmd_vel_zero = False
                            elif center_x > central_x_max:
                                self.get_logger().info("Move right")
                                linear_vec.y = -0.2  # Move right
                                cmd_vel_zero = False

                            if center_y < central_y_min:
                                self.get_logger().info("Move forward")
                                linear_vec.x = 0.2  # Move forward
                                cmd_vel_zero = False
                            elif center_y > central_y_max:
                                self.get_logger().info("Move backward")
                                linear_vec.x = -0.2  # Move backward
                                cmd_vel_zero = False

                            # Increment or reset counter based on cmd_vel status
                            if cmd_vel_zero and self.circling:
                                self.counter += 1
                                if self.counter >= self.counter_limit:
                                    self.counter = 0

                                    self.circling = False

                                    # Print segmented mask here
                                    # Display the segmented mask    
                                    # cv2.imshow("Segmented Mask", mask_np)
                                    # cv2.waitKey(0) 

                                    # Ignore these
                                    # self.mode = 'Bottom_depth'
                                    # Circle process here
                                    # threading.Thread(target=self.move_to_offset).start()
                            else:
                                self.counter = 0

                else:
                    linear_vec.x = 1.0  # Move forward

                if self.circling:
                    twist = Twist(linear=linear_vec, angular=angular_vec)
                    self.cmd_vel_publisher.publish(twist)

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        if self.mode == 'front':
            
            img = bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model(img)

            # Frame dimensions
            frame_width = img.shape[1]
            frame_height = img.shape[0]
            bottom_threshold = frame_height - 30  # 30 pixels from the bottom

            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            cmd_vel_zero = True  # Track if cmd_vel publishes all zeros
            for r in results:
                masks = r.masks  # Segmentation masks

                if masks is not None:
                    for mask in masks.data:
                        # Calculate the centroid of the mask
                        mask_np = mask.cpu().numpy().astype('uint8')
                        M = cv2.moments(mask_np)
                        if M['m00'] != 0:
                            center_x = int(M['m10'] / M['m00'])
                            center_y = int(M['m01'] / M['m00'])

                            # Define movement logic
                            linear_vec = Vector3()
                            angular_vec = Vector3()

                            if center_x < (frame_width - 40) // 2:
                                linear_vec.y = 0.2  # Move left
                                cmd_vel_zero = False
                            elif center_x > (frame_width + 40) // 2:
                                linear_vec.y = -0.2  # Move right
                                cmd_vel_zero = False

                            if center_y < bottom_threshold:
                                linear_vec.z = 0.2  # Move up
                                cmd_vel_zero = False

                            twist = Twist(linear=linear_vec, angular=angular_vec)
                            self.cmd_vel_publisher.publish(twist)

                            # Increment or reset counter based on cmd_vel status
                            if cmd_vel_zero:
                                self.counter += 1
                                if self.counter >= self.counter_limit:
                                    self.mode = 'bottom'
                                    self.counter = 0
                            else:
                                self.counter = 0

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    # def depth_camera_callback_bottom(self, data):
    #     if self.mode == 'Bottom_depth':
    #         # Convert the input image to OpenCV format
    #         img = bridge.imgmsg_to_cv2(data, "passthrough")
    #         img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
    #         # Run the segmentation model
    #         results = self.model(img)
            
    #         # Ensure header information is up-to-date
    #         self.yolov8_inference.header.frame_id = "inference"
    #         self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
            
    #         # Process each result in the detection
    #         for r in results:
    #             masks = r.masks  # Segmentation masks

    #             if masks is not None:
    #                 for mask in masks.data:
    #                     # Convert each mask to a NumPy array in black and white
    #                     mask_np = mask.cpu().numpy().astype(np.uint8) * 255

    #                     # # Display or save the mask
    #                     # cv2.imshow("Segmentation Mask", mask_np)  # Displays the mask
    #                     # cv2.waitKey(1)

    #                     # # Print mask information (optional)
    #                     # self.get_logger().info("Segmentation mask generated and displayed.")
            
    #         annotated_frame = results[0].plot()

    #         img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

    #         self.img_pub.publish(img_msg)
    #         self.yolov8_pub.publish(self.yolov8_inference)
    #         self.yolov8_inference.yolov8_inference.clear()


    def depth_camera_callback_bottom(self, msg):
        if not self.circling:  # Proceed only when circling is False
            # Extract the point cloud data from the message
            depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            # Convert the point cloud to a 2D depth array
            # You may need to know the specific dimensions of the depth image for reshaping
            depth_data = np.array([point[2] for point in depth_points])  # Extracting Z (depth) values
            image_width = 640  # Set according to your camera's resolution
            image_height = 360
            
            # Reshape the depth data to match image dimensions
            depth_image = depth_data.reshape((image_height, image_width))
            
            # Normalize the depth image for intensity plotting
            depth_image_normalized = (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))
            
            # Plot the intensity map
            plt.imshow(depth_image_normalized, cmap='gray')
            plt.colorbar(label='Normalized Depth')
            plt.title("Depth Intensity Map")
            plt.xlabel("Width")
            plt.ylabel("Height")
            plt.show()

    def move_to_offset(self, duration=1):
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

        threading.Thread(target=self.move_in_circle).start()

    def move_in_circle(self):
        start_time = time.time()
        positions = []
        
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

            positions.append((linear_vec.linear.x, linear_vec.linear.y, self.total_mask_area))
            
            # Publish the twist message
            self.cmd_vel_publisher.publish(linear_vec)
            self.get_logger().info(f"Moving in circle - x: {linear_vec.linear.x}, y: {linear_vec.linear.y}")
            
            # Delay to control the speed and radius of the circular motion
            time.sleep(self.delay)

        # Stop movement after completing the circle
        stop_twist = Twist()
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info("Completed one full circle and stopped.")

        # Plotting
        x_vals = [pos[0] for pos in positions]
        y_vals = [pos[1] for pos in positions]
        
        # Normalize area values to be between 0 and 1
        mask_areas = np.array([pos[2] for pos in positions])
        norm_areas = (mask_areas - mask_areas.min()) / (mask_areas.max() - mask_areas.min())
        
        # Use a colormap and plot points with color intensity based on area
        colors = cm.viridis(norm_areas)
        
        plt.figure(figsize=(8, 8))
        plt.scatter(x_vals, y_vals, color=colors, s=20)  # s=20 controls the marker size
        plt.colorbar(cm.ScalarMappable(cmap='viridis'), label="Normalized Mask Area Intensity")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Circular Path with Mask Area-Based Color Intensity")
        plt.show()

        # Export data to a CSV file
        with open('circle_movement_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X Position', 'Y Position', 'Mask Area', 'Normalized Mask Area'])  # Header
            for x, y, area, norm_area in zip(x_vals, y_vals, mask_areas, norm_areas):
                writer.writerow([x, y, area, norm_area])


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

# #!/usr/bin/env python3

# import cv2
# from ultralytics import YOLO
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist, Vector3

# from yolov8_msgs.msg import InferenceResult
# from yolov8_msgs.msg import Yolov8Inference

# bridge = CvBridge()

# class CameraSubscriber(Node):

#     def __init__(self):
#         super().__init__('camera_subscriber')

#         # self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt')
#         self.model = YOLO('/home/james/drone_ws/src/drone_ros2/yolobot_recognition/scripts/solar_panel.pt')
#         self.yolov8_inference = Yolov8Inference()

#         self.subscription = self.create_subscription(
#             Image,
#             '/simple_drone/bottom/image_raw',
#             self.camera_callback,
#             10)

#         self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
#         self.img_pub = self.create_publisher(Image, "/inference_result", 1)
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1)

#     def camera_callback(self, data):
#         img = bridge.imgmsg_to_cv2(data, "bgr8")
#         results = self.model(img)

#         # Frame dimensions
#         frame_width = 640
#         frame_height = 384
#         center_region_width = 40
#         center_region_height = 40

#         self.yolov8_inference.header.frame_id = "inference"
#         self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

#         for r in results:
#             boxes = r.boxes
#             for box in boxes:
#                 self.inference_result = InferenceResult()
#                 b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates (top, left, bottom, right)
#                 c = box.cls
#                 class_name = self.model.names[int(c)]
#                 self.inference_result.class_name = class_name
#                 self.inference_result.top = int(b[0])
#                 self.inference_result.left = int(b[1])
#                 self.inference_result.bottom = int(b[2])
#                 self.inference_result.right = int(b[3])
#                 self.yolov8_inference.yolov8_inference.append(self.inference_result)
#                 print(class_name.lower())

#                 # Check if the detected object is a 'person' and determine movement direction
#                 if class_name.lower() == 'human':
#                     center_x = int((b[0] + b[2]) / 2)
#                     center_y = int((b[1] + b[3]) / 2)

#                     # Draw a red circle at the center of the bounding box
#                     cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)

#                     # Define the central region boundaries
#                     central_x_min = (frame_width - center_region_width) // 2
#                     central_x_max = (frame_width + center_region_width) // 2
#                     central_y_min = (frame_height - center_region_height) // 2
#                     central_y_max = (frame_height + center_region_height) // 2

#                     # Check if the center of the bounding box is outside the target central region

#                     linear_vec = Vector3()
#                     angular_vec = Vector3()

#                     if center_x < central_x_min:
#                         self.get_logger().info("Move left")
#                         linear_vec.y = 0.2 # Left
#                     elif center_x > central_x_max:
#                         self.get_logger().info("Move right")
#                         linear_vec.y = -0.2 # Right

#                     if center_y < central_y_min:
#                         self.get_logger().info("Move forward")
#                         linear_vec.x = 0.2 # Front
#                     elif center_y > central_y_max:
#                         self.get_logger().info("Move backward")
#                         linear_vec.x = -0.2 # Back
#                     # else:
#                     #     self.get_logger().info("Centered")
                    
#                     twist = Twist(linear=linear_vec, angular=angular_vec)
#                     self.cmd_vel_publisher.publish(twist)

#         annotated_frame = results[0].plot()
#         img_msg = bridge.cv2_to_imgmsg(annotated_frame)

#         self.img_pub.publish(img_msg)
#         self.yolov8_pub.publish(self.yolov8_inference)
#         self.yolov8_inference.yolov8_inference.clear()

# if __name__ == '__main__':
#     rclpy.init(args=None)
#     camera_subscriber = CameraSubscriber()
#     rclpy.spin(camera_subscriber)
#     rclpy.shutdown()

# #!/usr/bin/env python3

# import cv2
# from ultralytics import YOLO
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# from yolov8_msgs.msg import InferenceResult
# from yolov8_msgs.msg import Yolov8Inference

# bridge = CvBridge()

# class Camera_subscriber(Node):

#     def __init__(self):
#         super().__init__('camera_subscriber')

#         self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt')

#         self.yolov8_inference = Yolov8Inference()

#         self.subscription = self.create_subscription(
#             Image,
#             '/simple_drone/bottom/image_raw',
#             # '/simple_drone/front/image_raw',
#             self.camera_callback,
#             10)
#         self.subscription 

#         self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
#         self.img_pub = self.create_publisher(Image, "/inference_result", 1)

#     def camera_callback(self, data):

#         img = bridge.imgmsg_to_cv2(data, "bgr8")
#         results = self.model(img)

#         self.yolov8_inference.header.frame_id = "inference"
#         self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

#         for r in results:
#             boxes = r.boxes
#             for box in boxes:
#                 self.inference_result = InferenceResult()
#                 b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
#                 c = box.cls
#                 class_name = self.model.names[int(c)]
#                 self.inference_result.class_name = class_name
#                 self.inference_result.top = int(b[0])
#                 self.inference_result.left = int(b[1])
#                 self.inference_result.bottom = int(b[2])
#                 self.inference_result.right = int(b[3])
#                 self.yolov8_inference.yolov8_inference.append(self.inference_result)

#                 # Check if the detected class is 'person' and draw a center marker
#                 if class_name.lower() == 'person':
#                     center_x = int((b[0] + b[2]) / 2)
#                     center_y = int((b[1] + b[3]) / 2)
#                     cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)  # Draw a red circle at the center of the bounding box

#             #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

#         annotated_frame = results[0].plot()
#         img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

#         self.img_pub.publish(img_msg)
#         self.yolov8_pub.publish(self.yolov8_inference)
#         self.yolov8_inference.yolov8_inference.clear()

# if __name__ == '__main__':
#     rclpy.init(args=None)
#     camera_subscriber = Camera_subscriber()
#     rclpy.spin(camera_subscriber)
#     rclpy.shutdown()
