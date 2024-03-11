from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
import xacro

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="False to use the standard mock of ros2",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "mqtt_broker_ip",
            default_value="10.7.7.246",
            description="The IP of the MQTT broker to communicate with the COMAU LPC.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mqtt_client_id",
            default_value="comau_c5gopen_hw_mqtt_client",
            description="Name of the MQTT client.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mqtt_port",
            default_value="1883",
            description="Port of the MQTT broker.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mqtt_loop_timeout",
            default_value="1",
            description="Maximum loop timeout for the MQTT client.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_fdb_pos_name",
            default_value="robot/arm1/real_joints_positions",
            description="Topic name for robot position feedback.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_fdb_vel_name",
            default_value="robot/arm1/real_joints_velocities",
            description="Topic name for robot velocity feedback.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_cmd_name",
            default_value="robot/arm1/target_joints_trajectory",
            description="Topic name for robot position command.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "read_only",
            default_value="false",
            description="If the robot provides only the feedback without receiving commands. ",
        )
    )

    description_package = "nj-220-27_description"
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controllers_file = LaunchConfiguration("controllers_file")
    mqtt_broker_ip = LaunchConfiguration("mqtt_broker_ip")
    mqtt_client_id = LaunchConfiguration("mqtt_client_id") 
    mqtt_port = LaunchConfiguration("mqtt_port")
    mqtt_loop_timeout = LaunchConfiguration("mqtt_loop_timeout")
    topic_fdb_pos_name = LaunchConfiguration("topic_fdb_pos_name")
    topic_fdb_vel_name = LaunchConfiguration("topic_fdb_vel_name")
    topic_cmd_name = LaunchConfiguration("topic_cmd_name")
    read_only = LaunchConfiguration("read_only")
    
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf", "nj-220-27.xacro.urdf"]),
            " ", "use_fake_hardware:=", use_fake_hardware,
            " ", "mqtt_broker_ip:=", mqtt_broker_ip,
            " ", "mqtt_client_id:=", mqtt_client_id,
            " ", "mqtt_port:=", mqtt_port,
            " ", "mqtt_loop_timeout:=", mqtt_loop_timeout,
            " ", "topic_fdb_pos_name:=", topic_fdb_pos_name,
            " ", "topic_fdb_vel_name:=", topic_fdb_vel_name,
            " ", "topic_cmd_name:=", topic_cmd_name,
            " ", "read_only:=", read_only,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}    

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("comau_c5gopen_ros2"),
            "config",
            controllers_file,
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
    )
    
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nj-220-27_moveit_config'),'launch', 'move_group.launch.py')]),)
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nj-220-27_moveit_config'),'launch', 'moveit_rviz.launch.py')]),)
    
    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
    ]

    return LaunchDescription( declared_arguments + nodes_to_start )



