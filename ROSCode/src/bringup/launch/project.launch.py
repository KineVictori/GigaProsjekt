from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    This function generates the launch description for starting multiple ROS 2 nodes and included launch files.
    It allows setting parameters for UR robot type, robot IP, and more from the command line.
    """

    # Define launch arguments that can be set from the command line
    ur_type = LaunchConfiguration('ur_type', default='ur3')  # UR robot type (e.g., ur3)
    robot_ip = LaunchConfiguration('robot_ip', default='143.25.150.133')  # Robot IP address (133 = Eevee, 7 = Squirtle. Both are UR3 robots)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')  # Use mock hardware if true
    initial_joint_controller = LaunchConfiguration('initial_joint_controller', default='scaled_joint_trajectory_controller')  # Controller for joints
    launch_rviz = LaunchConfiguration('launch_rviz', default='false')  # Launch RViz if true
    video_device = LaunchConfiguration('video_device', default='/dev/video0')  # Launch RViz if true

    # Include the ur_robot_driver launch file with specified arguments
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_mock_hardware': use_mock_hardware,
            'initial_joint_controller': initial_joint_controller,
        }.items()
    )

    # Include the ur_moveit_config launch file with specified arguments
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
        }.items()
    )

    # Start the camera detection node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        parameters=[{'video_device': video_device}]
    )
    
    # Start the camera detection node
    detection_node = Node(
        package='camera_detection',
        executable='detection_node',
        name='detection_node'
    )

    object_pointer_node = Node(
        package='object_pointer',
        executable='object_pointer_node',
        name='object_pointer_node',
    )

    # Start the custom robot controller node
    robot_controller_node = Node(
        package='custom_robot_controller',
        executable='robot_controller_node',
        name='robot_controller_node'
    )

    # Return the launch description with all nodes and included launch files
    return LaunchDescription([
        ur_control_launch,
        ur_moveit_launch,
        #camera_node,
        #detection_node,
        object_pointer_node,
        robot_controller_node,
    ])
