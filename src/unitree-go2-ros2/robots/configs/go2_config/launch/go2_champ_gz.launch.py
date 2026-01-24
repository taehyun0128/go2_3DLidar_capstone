import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------
    # Launch args
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")
    rviz = LaunchConfiguration("rviz")
    lite = LaunchConfiguration("lite")

    world = LaunchConfiguration("world")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go2")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false")
    declare_lite = DeclareLaunchArgument("lite", default_value="false")

    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.sdf",
        description="GZ Sim world (SDF). e.g. empty.sdf or /abs/path/to/world.sdf"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    # -------------------------
    # Paths
    # -------------------------
    config_pkg_share = get_package_share_directory("go2_config")
    descr_pkg_share = get_package_share_directory("go2_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    # ✅ Go2 VLP xacro
    default_model_path = os.path.join(descr_pkg_share, "xacro", "robot_VLP.xacro")

    # ✅ (핵심1) xacro 실행 문자열 공백 문제 해결: "xacro " 로 써야 함
    # ✅ (핵심2) YAML 파싱 방지: ParameterValue(value_type=str)
    robot_description = ParameterValue(
        Command(["xacro ", default_model_path]),
        value_type=str
    )

    # -------------------------
    # (A) GZ Sim
    # -------------------------
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world, "-v", "4"],
        output="screen"
    )


    # -------------------------
    # (B) /clock bridge (선택이지만 추천)
    # -------------------------
    clock_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "-r", "/world/empty/clock:=/clock",
            "-p", "lazy:=false",
        ],
        output="screen",
    )

    # -------------------------
    # (C) robot_state_publisher
    # -------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description
        }],
    )

    # -------------------------
    # (D) Spawn robot (ros_gz_sim create)
    # -------------------------
    spawn_go2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "empty",                 # ✅ 여기 world 서비스 이름. 대부분 empty.
            "-name", robot_name,
            "-allow_renaming", "true",
            "-param", "robot_description",
            "-x", world_init_x,
            "-y", world_init_y,
            "-z", world_init_z,
            "-Y", world_init_heading,
        ],
        parameters=[{
            # ✅ create 노드도 같은 robot_description(ParameterValue)을 받게 해야 YAML 파싱 안 터짐
            "robot_description": robot_description
        }],
    )

    # -------------------------
    # (E) Spawn controllers
    # -------------------------
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )

    traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )

    spawn_controllers = TimerAction(
        period=8.0,
        actions=[jsb_spawner, traj_spawner]
    )

    # -------------------------
    # (F) CHAMP bringup (✅ 그대로 두되, include 방식은 예전처럼)
    # -------------------------
    champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "gazebo": "true",
            "lite": lite,
            "rviz": rviz,
            "joint_controller_topic": "joint_trajectory_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    # -------------------------
    # (G) gz_ros2_control plugin path
    # -------------------------
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.path.expanduser("~/gz_ros2_control_ws/install/gz_ros2_control/lib")
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_name,
        declare_rviz,
        declare_lite,
        declare_world,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,

        set_gz_plugin_path,

        gz_sim,
        clock_bridge,

        rsp,
        TimerAction(period=2.0, actions=[spawn_go2]),
        spawn_controllers,

        champ_bringup,
    ])
