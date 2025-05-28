# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch
#
#
# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("crx10ia", package_name="crx10ia_moveit_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import OpaqueFunction
from moveit_commander import MoveGroupCommander
import rospy

def adjust_j2_j3_target(context):
    """前置補償 Fanuc 手臂 J2/J3 互補機制"""
    rospy.init_node('adjust_j2_j3_node', anonymous=True)

    # 建立 MoveGroupCommander 以取得當前狀態
    group = MoveGroupCommander("manipulator")

    # 取得目前的關節角度
    joint_positions = group.get_current_joint_values()

    # J2 在索引 1，J3 在索引 2
    j2_current = joint_positions[1]
    j3_calculated = joint_positions[2]

    # Fanuc J2/J3 補償公式
    joint_positions[2] = j3_calculated + j2_current

    # 設定調整後的角度目標
    group.set_joint_value_target(joint_positions)

    rospy.loginfo(f"[J2/J3 補償] J2: {j2_current:.2f} rad | J3(補償後): {joint_positions[2]:.2f} rad")

    # 執行計劃
    plan = group.plan()
    group.execute(plan, wait=True)

def generate_launch_description():
    """產生 Launch 描述"""
    # 取得 MoveIt 設定
    moveit_config = MoveItConfigsBuilder("crx10ia", package_name="crx10ia_moveit_config").to_moveit_configs()

    # 加入前置補償邏輯
    adjust_j2_j3_node = OpaqueFunction(function=adjust_j2_j3_target)

    # 生成 move_group 啟動描述
    move_group_launch = generate_move_group_launch(moveit_config)

    # 將補償節點加入啟動流程
    move_group_launch.entities.insert(0, adjust_j2_j3_node)

    return move_group_launch
