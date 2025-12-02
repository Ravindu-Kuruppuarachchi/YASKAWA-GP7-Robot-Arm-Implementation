import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get Paths
    pkg_gp7_sim = get_package_share_directory('gp7_sim')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # 2. Set the Mesh Path (Environment Variable)
    # This fixes the "Unable to find file" error automatically
    set_env_vars = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_gp7_sim, '..', '..', 'src'), ':' + os.environ.get('HOME') + '/ros2_ws/src']
    )

    # 3. Launch Gazebo with the World (Boxes)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {os.path.join(pkg_gp7_sim, "worlds", "project.sdf")}'}.items(),
    )

    # 4. Spawn the Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_gp7_sim, 'urdf', 'gp7.urdf'),
            '-name', 'gp7_robot',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # 5. Start the Bridge (MATLAB Connection)
    # Bridges all 6 joints + clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/gp7_robot/joint/joint_s/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/joint_l/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/joint_u/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/joint_r/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/joint_b/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/joint_t/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/finger_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/gp7_robot/joint/right_outer_knuckle_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_env_vars,
        gazebo,
        spawn_robot,
        bridge
    ])
