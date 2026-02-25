from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def get_processed_launch_objects(context):
    uav_name = LaunchConfiguration('uav_name').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    aer_node = ComposableNode(
        package='aer_auto_exposure_gradient',
        plugin='exp_node::ExpNode',
        name='aer_node',
        namespace=uav_name,
        parameters=[{
            'image_topic': f'/{uav_name}/{camera_name}/image_raw',
            'shutter_speed_apply_topic': f'/{uav_name}/expose_us',
            'img_proc_loop_hz': 2,
            'optimizer_loop_hz': 20,
            'shutter_update_method': 'gradient',
            'grad_k': 0.05,
            'enable_plotter': True,
            'gamma_x_offset': 0.0,
            'curve_fit_method': 'log_quadratic',
            'do_sweep': False
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='autoexposure',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[aer_node],
    )

    return [container]

def generate_launch_description():
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV namespace'
    )

    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='bluefox',
        description='Camera name used in topic namespace'
    )

    return LaunchDescription([
        declare_uav_name,
        declare_camera_name,
        OpaqueFunction(function=get_processed_launch_objects),
    ])
