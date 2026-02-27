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
            # --- Image source ---
            'image_topic': f'/{uav_name}/{camera_name}/image_raw',

            # --- Actuator output topics ---
            'shutter_speed_apply_topic': f'/{uav_name}/expose_us',
            'gain_apply_topic': f'/{uav_name}/gain_db',
            'led_apply_topic': '',           # empty = LED disabled

            # --- Actuator slices: order defines priority (first = used first) ---
            # Portions must sum to <= 1.0; remainder is unused headroom.
            'actuator_order': ['shutter', 'gain', 'led'],
            'shutter_portion': 0.4,          # 30 % of [0,1] drives shutter
            'shutter_max_us': 5000,           # shutter range: 0 – 3000 µs
            'gain_portion': 0.6,             # 50 % of [0,1] drives gain
            'gain_max': 12.0,                # gain range: 0 – 12 dB
            'led_portion': 0.0,              # 20 % of [0,1] drives LED
            'led_max': 40.0,                 # LED range: 0 – 40 W

            # --- Optimizer initial state ---
            'initial_exposure_level': 0.1,   # normalized [0,1] starting point

            # --- Optimizer method ---
            'shutter_update_method': 'gradient',   # 'simple' | 'gradient' | 'shim'

            # --- Shim params ---
            'shim_update_function': '2018',         # '2014' | '2018' (only for shim)
            'kp': 0.02,

            # --- Gradient optimizer params ---
            'optimizer_loop_hz': 20,
            'grad_k': 0.05,

            # --- Simple optimizer params ---
            'simple_step_size': 0.01,        # step per gamma-index unit (simple method)

            # --- Image processing loop rates ---
            'img_proc_loop_hz': 2,
            'startup_delay': 1,

            # --- Gamma / curve-fit settings ---
            'shutter_update_method': 'gradient',
            'curve_fit_method': 'log_quadratic',    # 'quadratic' | 'log_quadratic'
            'gamma_range': 2.0,
            'gamma_num_points': 9,
            'gamma_x_offset': 0.0,

            # --- Sweep (debug) ---
            'do_sweep': False,
            'sweep_steps': 100,

            # --- Plotter (requires WITH_PLOTTER build flag) ---
            'enable_plotter': True,

            'shutter_limit_topic': f'/{uav_name}/shutter_limit'
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
