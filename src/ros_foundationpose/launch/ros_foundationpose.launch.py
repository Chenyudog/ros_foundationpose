import launch
import launch_ros



def generate_launch_description():

    delete_photo_node=launch_ros.actions.Node(
        package='ros_foundationpose',
        executable='delete_photo_node'
    )

    take_photo_node=launch_ros.actions.Node(
        package='ros_foundationpose',
        executable='take_photo_node'
    )






    return launch.LaunchDescription([
        delete_photo_node,
        take_photo_node
    ])