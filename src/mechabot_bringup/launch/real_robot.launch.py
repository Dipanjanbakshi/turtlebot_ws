import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    serial_port = LaunchConfiguration("serial_port") #Port for ESP32
    map_name = LaunchConfiguration("map_name")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="False"
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0", 
        description="Serial port for ESP32"
    )

    camera_driver= Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[
                {
                    'width': 640,
                    'height': 480,
                    'format': 'YUYV',
                }
            ],
            output='screen'
        )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="my_house"
    )


    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechabot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
        launch_arguments={
            "serial_port": serial_port #Passes serial port to urdf
        }.items()
    )

    lidar_driver= Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/ttyUSB1",
            "serial_baudrate": 460800,
            "frame_id": "lidar_link",
            "inverted": False,
            "angle_compensate": True,
            "angle_compensate_multiple": 1,
            "scan_mode": "DenseBoost", #Standard, #Express, #Stability
            "range_min": 0.05
        }],
        output="screen"
    )


    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechabot_controller"),
                "launch",
                "joystick.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    imu_driver_node = Node(
        package="mechabot_firmware",
        executable="mpu6050_driver.py",
    )

    lcd_driver_node = Node(
        package="mechabot_firmware",
        executable="tft_display.py",
    )

    ultrasonic_driver_node = Node(
        package="mechabot_firmware",
        executable="hcsr04_driver.py",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechabot_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False",
            "map_name": map_name
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    navigation_delayed = TimerAction(
    period=5.0,  # wait 5 sec
    actions=[navigation]
    )


    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechabot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        serial_port_arg,
        map_name_arg,
        hardware_interface,
        lidar_driver,
        controller,
        joystick,
        imu_driver_node,
        # lcd_driver_node,
        # ultrasonic_driver_node,
        localization,
        # slam,
        # navigation,
        navigation_delayed,
        camera_driver,
    ])