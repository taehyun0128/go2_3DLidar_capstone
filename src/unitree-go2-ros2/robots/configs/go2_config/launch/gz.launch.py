import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------
    # Launch args
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")
    gui = LaunchConfiguration("gui")

    world = LaunchConfiguration("world")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")  # yaw (rad)

    lite = LaunchConfiguration("lite")
    rviz = LaunchConfiguration("rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock if true"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use Gazebo GUI"
    )

    # Gazebo Classic(.world) 말고, gz sim이 읽을 수 있는 SDF world를 써야 함.
    # 기본은 empty.sdf (gz 설치되어 있으면 보통 동작)
    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.sdf",
        description="GZ Sim world (SDF). e.g. empty.sdf or /abs/path/to/world.sdf"
    )

    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    # -------------------------
    # Paths
    # -------------------------
    config_pkg_share = get_package_share_directory("go2_config")
    descr_pkg_share = get_package_share_directory("go2_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    # go2_description xacro
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")

    # xacro -> robot_description
    robot_description = Command(["xacro ", default_model_path])

    # -------------------------
    # (A) CHAMP bringup (Gazebo Classic용 gazebo.launch는 제거)
    # joint_controller_topic은 네가 실제로 띄운 controller에 맞춰야 함
    # (지금은 joint_trajectory_controller를 쓰고 있으니 그 토픽으로)
    # -------------------------
    bringup_ld = Node(
        package="launch_ros", executable="placeholder", output="screen",
        condition=None
    )
    # ↑ launch_ros placeholder는 안 쓰고, 아래처럼 IncludeLaunchDescription을 쓰는 게 정석인데
    #    네가 준 원본 구조 유지하려면 include를 그대로 가져오는 형태가 좋아.
    #    (placeholder는 제거용. 아래 include를 실제로 씀.)

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

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
            # ✅ Classic의 joint_group_effort_controller 대신, 지금 네가 쓰는 trajectory controller 토픽으로
            "joint_controller_topic": "joint_trajectory_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    # -------------------------
    # (B) GZ Sim 실행
    # gui:=false면 headless로 돌리고 싶은데 옵션이 환경마다 달라서
    # 여기서는 가장 단순하게 그냥 gz sim을 실행(네가 수동으로 하던 방식)
    # -------------------------
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world, "-v", "4"],
        output="screen"
    )

    # -------------------------
    # (C) robot_state_publisher
    # gz_ros2_control 플러그인이 여기에서 robot_description을 가져감
    # -------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description
        }],
    )

    # -------------------------
    # (D) 로봇 스폰 (ros_gz_sim create)
    # -param 로 robot_description을 create 노드 자체 파라미터로 넣어서 스폰
    # -------------------------
    spawn_go2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", "empty",                 # ✅ world 이름(서비스에 보이는 이름). 대부분 empty
            "-name", robot_name,
            "-allow_renaming", "true",
            "-param", "robot_description",
            "-x", world_init_x,
            "-y", world_init_y,
            "-z", world_init_z,
            "-Y", world_init_heading,          # yaw (rad)
        ],
        parameters=[{
            "robot_description": robot_description
        }],
    )

    # -------------------------
    # (E) Controller spawners
    # controller_manager가 올라오는데 시간이 걸릴 수 있으니 TimerAction으로 지연
    # -------------------------
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
    )

    spawn_controllers = TimerAction(
        period=8.0,  # 환경 따라 5~10초로 조절
        actions=[jsb_spawner, traj_spawner]
    )

    # -------------------------
    # (F) 환경변수 (gz_ros2_control 플러그인 경로)
    # 런치에서 고정해주면 터미널마다 export 안 해도 됨
    # -------------------------
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.path.expanduser("~/gz_ros2_control_ws/install/gz_ros2_control/lib")
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_robot_name,
        declare_lite,
        declare_gui,
        declare_world,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,

        set_gz_plugin_path,

        # 순서상: gz sim -> rsp -> spawn -> controllers -> champ
        gz_sim,
        rsp,
        TimerAction(period=2.0, actions=[spawn_go2]),
        spawn_controllers,

        champ_bringup,
    ])

