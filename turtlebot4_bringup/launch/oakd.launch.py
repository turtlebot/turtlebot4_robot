import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    camera_name  = LaunchConfiguration('camera_name',   default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '0.0')

    mode         = LaunchConfiguration('mode', default = 'depth')
    lrcheck      = LaunchConfiguration('lrcheck', default = True)
    extended     = LaunchConfiguration('extended', default = False)
    subpixel     = LaunchConfiguration('subpixel', default = True)
    confidence   = LaunchConfiguration('confidence', default = 200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default = 5)

    robot_model = DeclareLaunchArgument('model', 
        default_value='standard',
        choices=['standard', 'lite'],
        description='Turtlebot4 Model')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'camera_name' : camera_name,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())

    stereo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_node',
            output='screen',
            parameters=[{'camera_name': camera_name},
                        {'mode': mode},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel}])

    oakd_standard_stf = launch_ros.actions.Node(
            name='oakd_standard_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.0596', '0', '0.17933', '0', '0', '0', 'base_link', 'oak-d-base-frame'],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_lite_stf = launch_ros.actions.Node(
            name='oakd_lite_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.09', '0', '0.18588622', '0', '0', '0', 'base_link', 'oak-d-base-frame'],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    ld = LaunchDescription()
    ld.add_action(robot_model)
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)

    ld.add_action(stereo_node)
    ld.add_action(urdf_launch)
    ld.add_action(oakd_standard_stf)
    ld.add_action(oakd_lite_stf)

    return ld
