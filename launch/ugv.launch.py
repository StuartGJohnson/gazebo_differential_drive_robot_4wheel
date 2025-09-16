import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import perform_substitutions
from launch import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml
import numpy as np

def launch_setup(context, *args, **kwargs):
    my_arg_value = LaunchConfiguration("world").perform(context)
    return my_arg_value
    # print(f"Resolved string: {my_arg_value}")  # Now a real string
    # # Use my_arg_value to construct nodes, paths, etc.
    # return []

def generate_launch_description():
    # Define the robot's name and package name
    robot_name = "differential_drive_robot_4wheel"
    package_name = "gazebo_differential_drive_robot_4wheel"

    # Define a launch argument for the world file, defaulting to "empty.sdf"

    # Get the world file
    # my_world_file = os.path.join(
    #      get_package_share_directory(package_name),
    #      'worlds',
    #      'ugv_world.world'
    # )

    #print(my_world_file)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/sjohnson/WorldGeneration/scene_stuff.sdf',
        description='Specify the world file base for Gazebo (e.g., /path/farmhouse)'
    )

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='ugv.xacro',
        description='Specify the robot file base for Gazebo (in the model dir))'
    )

    # Define launch arguments for initial pose
    # x_arg = DeclareLaunchArgument(
    #     'x', default_value='0.0', description='Initial X position')
    #
    # y_arg = DeclareLaunchArgument(
    #     'y', default_value='0.0', description='Initial Y position')
    #
    # z_arg = DeclareLaunchArgument(
    #     'z', default_value='0.5', description='Initial Z position')
    #
    # roll_arg = DeclareLaunchArgument(
    #     'R', default_value='0.0', description='Initial Roll')
    #
    # pitch_arg = DeclareLaunchArgument(
    #     'P', default_value='0.0', description='Initial Pitch')
    #
    # yaw_arg = DeclareLaunchArgument(
    #     'Y', default_value='0.0', description='Initial Yaw')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')

    # retrieve robot_start_xy from world definition
    # strip extension from path
    # cfg_world_path = world_file + '.yml'
    #
    # with open(cfg_world_path, 'r') as file:
    #     data = yaml.safe_load(file)
    # robot_start_xy = np.array(data['robot_start_xy'])

    # x = LaunchConfiguration('x')
    # y = LaunchConfiguration('y')
    # z = LaunchConfiguration('z')
    # roll = LaunchConfiguration('R')
    # pitch = LaunchConfiguration('P')
    # yaw = LaunchConfiguration('Y')
    # x = robot_start_xy[0]
    # y = robot_start_xy[1]
    # z = 0.25
    # roll = 0.0
    # pitch = 0.0
    # yaw = 0.0

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge_ugv.yaml'
    )

    gz_bridge_params_qos_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge_qos.yaml'
    )

    # Prepare to include the Gazebo simulation launch file
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    # Include the Gazebo launch description with specific arguments
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        # -s is headless mode
        launch_arguments={
            'gz_args': [f'-r -s -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    def launch_spawn(context: LaunchContext, *args, **kwargs):
        world_name = LaunchConfiguration("world").perform(context)
        world_file_root = os.path.splitext(world_name)[0]
        model_file = LaunchConfiguration("robot").perform(context)
        # retrieve robot_start_xy from world definition
        # strip extension from path
        cfg_world_path = world_file_root + '.yml'
        print(cfg_world_path)

        with open(cfg_world_path, 'r') as file:
            data = yaml.safe_load(file)
        robot_start_xy = np.array(data['robot_start_xy'])
        x = float(robot_start_xy[0])
        y = float(robot_start_xy[1])
        print(x,y)
        z = 0.005
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Set paths to Xacro model and configuration files
        robot_model_path = os.path.join(
            get_package_share_directory(package_name),
            'model',
            model_file
        )
        # Process the Xacro file to generate the URDF representation of the robot
        robot_description = xacro.process_file(robot_model_path).toxml()
        # print(robot_description)

        # Create a node to spawn the robot model in the Gazebo environment
        spawn_model_gazebo_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-string', robot_description,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z),
                '-R', str(roll),
                '-P', str(pitch),
                '-Y', str(yaw),
                '-allow_renaming', 'false'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # Create a node to publish the robot's state based on its URDF description
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description, 'use_sim_time': True}
            ],
            output='screen'
        )

        return [spawn_model_gazebo_node, robot_state_publisher_node]


    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args','-p',
            f'config_file:={gz_bridge_params_path}',
#            '--params-file', f'{gz_bridge_params_qos_path}'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    gt_bridge_node = Node(
        package = 'gazebo_differential_drive_robot_4wheel',
        executable = 'gt_bridge_node',
        arguments=['differential_drive_robot_4wheel'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/image'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    start_gazebo_ros_depth_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/depth_image'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        robot_arg,
        gazebo_launch,
        # x_arg,
        # y_arg,
        # z_arg,
        # roll_arg,
        # pitch_arg,
        # yaw_arg,
        OpaqueFunction(function=launch_spawn),
        #robot_state_publisher_node,
        gz_bridge_node,
        gt_bridge_node,
        start_gazebo_ros_depth_image_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd
    ])
