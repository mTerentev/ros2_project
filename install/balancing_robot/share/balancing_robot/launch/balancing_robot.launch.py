import launch
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions.node import Node

def generate_launch_description():

    gazebo = ExecuteProcess(cmd = ["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", "-s", "libgazebo_ros_init.so"])

    spawn = Node(package="balancing_robot", executable="spawn")

    controller = Node(package="balancing_robot", executable="controller")

    imu = Node(package="balancing_robot", executable="imu_decoder")

    tilt_regulator = Node(name="tilt_regulator", package="balancing_robot", executable="regulator", parameters=[{
        "P": 1.5,
        "I": 0.2,
        "D": 1.0,
        "K": 10.0,
        "value_topic": "balancing_robot/tilt",
        "control_topic": "balancing_robot/torque",
        "target_topic" : "balancing_robot/tilt_control"
    }])

    vel_regulator = Node(name="vel_regulator", package="balancing_robot", executable="regulator", parameters=[{
        "P": 0.8,
        "I": 0.01,
        "D": 3.0,
        "K": 0.1,
        "value_topic": "balancing_robot/velocity",
        "control_topic": "balancing_robot/tilt_control",
        "target_topic" : "balancing_robot/velocity_control"
    }])

    teleop = Node(package="balancing_robot", executable="teleop_node")

    return LaunchDescription([
        teleop,
        gazebo,
        spawn,
        controller,
        imu,
        tilt_regulator,
        vel_regulator,
    ])