import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='node_with_args',
            node_executable='my_node',
            output='screen',
            arguments=[launch.substitutions.CommandLineArgument('arg')],
        ),
    ])
