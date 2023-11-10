import launch
from launch import LaunchDescription
import launch_ros.actions
# from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext

import os


def generate_launch_description():
    cond = False
    for file in os.listdir('.'):
        if file == '2017-10-31-22-06-52':
            cond = True
            break

    pkg_path = FindPackageShare('floor_demo').perform(LaunchContext())

    if not cond:
        print("\n\n\n\n\n################################################################################")
        print("ROS bag not extracted.\nTrying to extract from \'{}rosbag2.zip\'".format(pkg_path+'/'))
        os.system("unzip " + pkg_path +"/rosbag2.zip")
        print("################################################################################\n\n\n\n\n")

    return LaunchDescription([
        launch_ros.actions.Node(
            package="floor_demo", executable="floor_demo", output="screen"
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d'+'config/rviz2.rviz']
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2','bag','play','2017-10-31-22-06-52/']
        )
    ])