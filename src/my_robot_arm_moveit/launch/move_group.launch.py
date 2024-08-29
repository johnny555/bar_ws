from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_robot_arm", package_name="my_robot_arm_moveit").to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ""
    return generate_move_group_launch(moveit_config)
