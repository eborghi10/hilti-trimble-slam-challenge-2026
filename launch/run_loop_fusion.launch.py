from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


launch_args = [
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="Full path to loop_fusion config YAML. If empty, uses default.",
    ),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = os.path.join(
            get_package_share_directory("challenge_tools_ros"),
            "config", "hilti_loop_fusion", "loop_fusion_config.yaml"
        )

    # loop_fusion_node takes config file as a CLI argument (argv[1])
    node = Node(
        package="loop_fusion",
        executable="loop_fusion_node",
        name="loop_fusion_node",
        output="screen",
        arguments=[config_path],
        remappings=[
            # Map OpenVINS topics to what loop_fusion subscribes to
            ("/vins_estimator/odometry", "/ov_msckf/odomimu"),
            ("/vins_estimator/keyframe_pose", "/ov_msckf/loop_pose"),
            ("/vins_estimator/keyframe_point", "/ov_msckf/loop_feats"),
            ("/vins_estimator/extrinsic", "/ov_msckf/loop_extrinsic"),
            # Remap output topic to namespaced version
            ("/odometry_rect", "/loop_fusion/odometry_rect"),
        ],
    )

    return [node]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
