from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    parameters=[{
          'frame_id':'simple_drone/front_cam_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':True,
          'odom_frame_id':'simple_drone/odom',
          "use_sim_time":True,
          'qos':1,
        #   'subscribe_depth':True,
        #   'use_action_for_goal':True,
        #   'Reg/Force3DoF':'true',
          'topic_queue_size':20,
          'queue_size':20}]

    remappings=[
        #   ('odom', 'simple_drone/odom'),
          ('rgb/image', '/simple_drone/depth_front/image_raw'),
          ('rgb/camera_info', '/simple_drone/depth_front/camera_info'),
          ('depth/image', '/simple_drone/depth_front/depth/image_raw')]

    return LaunchDescription([

        # Nodes to launch
        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
    ])