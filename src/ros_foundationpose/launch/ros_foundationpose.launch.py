import launch
import launch_ros
import os
DEMO1_SCRIPT_PATH="/home/ubuntu/main_ws/ros_foundationpose/src/ros_foundationpose/FoundationPose/run_demo1.py"


def generate_launch_description():

    delete_photo_node=launch_ros.actions.Node(
        package='ros_foundationpose',
        executable='delete_photo_node'
    )

    take_photo_node=launch_ros.actions.Node(
        package='ros_foundationpose',
        executable='take_photo_node'
    )

    demo_node= launch.actions.ExecuteProcess(
        # 核心命令：和你终端输入的「python3 run_demo1」完全一致
        cmd=['python3', DEMO1_SCRIPT_PATH],
        name='run_demo1',
        output='screen',  # 必须加：否则看不到脚本的打印输出
        additional_env={'PYTHONUNBUFFERED': '1'},  # 必须加：Python输出实时显示
        cwd=os.path.dirname(DEMO1_SCRIPT_PATH)  # 可选：设置脚本的运行目录（等同于先cd到脚本目录）
    )

    get_position_node=launch_ros.actions.Node(
        package='ros_foundationpose',
        executable='get_position_node'
    )




    return launch.LaunchDescription([
        delete_photo_node,
        take_photo_node,
        demo_node,
        get_position_node
    ])