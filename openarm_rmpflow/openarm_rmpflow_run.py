# =====================================================================
# §7-1 OpenARM 左臂 RmpFlow —— GUI 原生 Play 版（繞開 World/SimulationManager）
# 2026-07-20 實測會動：貼進 Isaac Sim 6.0 容器 isaac-intern1 的 Script Editor 執行，
# 執行後「手動按工具列 ▶ Play」，左臂即朝目標點做點對點運動並收斂 hold。
#
# 為什麼不用 World：本環境 World.reset()/reset_async() 初始化物理必失敗
#   （RuntimeError: Physics context is not initialized；6.0 SimulationManager
#    不把 stage 上的 /PhysicsScene 註冊成 default）。見手冊坑⑩。
#   → 改用 GUI Play 起物理 + omni.physx step callback 驅動。
# 兩個必備前置：坑⑤（補 PhysX articulation 旗標）、坑⑨（set_gains，否則亂甩）。
# =====================================================================
import numpy as np
import omni.usd, omni.physx, omni.timeline, builtins
from pxr import UsdPhysics, PhysxSchema
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot_motion.motion_generation")
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
try:
    from isaacsim.core.prims import SingleArticulation as ArtClass
except Exception:
    from isaacsim.core.api.robots import Robot as ArtClass

TOP_USD    = "/home/Intern_1/openarm_ws/src/openarm_description/urdf/robot/openarm_bimanual/openarm_bimanual.usda"
ROBOT_DESC = "/home/Intern_1/openarm_ws/src/openarm_description/rmpflow/openarm_left_lula_description.yaml"
RMP_CONFIG = "/home/Intern_1/openarm_ws/src/openarm_description/rmpflow/openarm_left_rmpflow_config.yaml"
URDF_PATH  = "/home/Intern_1/openarm_ws/src/openarm_description/urdf/robot/openarm_bimanual.urdf"
EE_FRAME   = "openarm_left_hand_tcp"
PRIM_PATH  = "/World/openarm"
TARGET_POS = np.array([0.45, 0.30, 0.25])
TARGET_ORI = np.array([0.0, 1.0, 0.0, 0.0])
PHYSICS_DT = 1.0 / 60.0

# 1) 載 USD（沒載才載）
stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(PRIM_PATH).IsValid():
    add_reference_to_stage(usd_path=TOP_USD, prim_path=PRIM_PATH)

# 2) 找 articulation root + 坑⑤ PhysX 旗標
root_prim = None
for prim in stage.Traverse():
    if prim.GetPath().pathString.startswith(PRIM_PATH) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        root_prim = prim; break
assert root_prim is not None, "找不到 ArticulationRootAPI，USD 拖錯層"
PhysxSchema.PhysxArticulationAPI.Apply(root_prim).CreateArticulationEnabledAttr(True)
ROOT_PATH = str(root_prim.GetPath())
print("[setup] articulation root =", ROOT_PATH)

# 3) 建物件（不建 World、不 reset）
robot   = ArtClass(prim_path=ROOT_PATH, name="openarm_native")
rmpflow = RmpFlow(robot_description_path=ROBOT_DESC, rmpflow_config_path=RMP_CONFIG,
                  urdf_path=URDF_PATH, end_effector_frame_name=EE_FRAME, maximum_substep_size=0.00334)

# 4) 第一個 physics step 才 initialize（此時 Play 已起、sim view 才存在）
_g = {"inited": False, "art_rmp": None, "idx_map": None, "n": 0}

def _lazy_init():
    robot.initialize()
    robot.get_articulation_controller().set_gains(               # 坑⑨：不加就亂甩
        kps=np.full(robot.num_dof, 10000.0), kds=np.full(robot.num_dof, 1000.0))
    dq = np.zeros(robot.num_dof)
    for i, n in enumerate(robot.dof_names):
        if "joint4" in n: dq[i] = 1.2                            # joint4 limit 下界 0，給 0 會卡
    robot.set_joint_positions(dq)
    robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=dq))
    rmpflow.set_robot_base_pose(*robot.get_world_pose())
    active = rmpflow.get_active_joints()
    # ArticulationMotionPolicy 內部照 active joints 建 ArticulationSubset，
    # 回傳的 action 只含左臂 7 軸的 joint_indices → 右臂/夾爪不被碰。
    _g["idx_map"] = {n: robot.get_dof_index(n) for n in active}
    _g["art_rmp"] = ArticulationMotionPolicy(robot, rmpflow, default_physics_dt=PHYSICS_DT)
    rmpflow.update_world()
    rmpflow.set_end_effector_target(target_position=TARGET_POS, target_orientation=TARGET_ORI)
    print(f"[init] OK num_dof={robot.num_dof} idx={_g['idx_map']}")

def on_physics_step(dt):
    try:
        if not _g["inited"]:
            _lazy_init(); _g["inited"] = True; return
        a = _g["art_rmp"].get_next_articulation_action()
        robot.apply_action(a)
        if _g["n"] % 60 == 0:
            q = robot.get_joint_positions()
            meas = [round(float(q[j]), 3) for j in _g["idx_map"].values()]
            print(f"  step {_g['n']:4d} 實測={meas}")
        _g["n"] += 1
    except Exception as e:
        print("[callback err]", e)

# 5) 掛 physics step 事件；handle 存 builtins 避免被 GC
builtins._RMP_SUB = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)

# 5b) 掛 timeline 事件：按 ■ Stop 會拆掉 physics view（robot handle 失效 → callback 拋
#     'NoneType' has no attribute 'astype'、雙臂軟掉）。這裡在 Stop 時把 inited 歸零，
#     下次按 ▶ Play 的第一個 physics step 就自動重新 initialize()+set_gains → 不再軟掉。
def _on_timeline(e):
    if e.type == int(omni.timeline.TimelineEventType.STOP):
        _g["inited"] = False
        print("[timeline] STOP → 已重置，下次 Play 會自動重新初始化")
builtins._RMP_TL = omni.timeline.get_timeline_interface() \
    .get_timeline_event_stream().create_subscription_to_pop(_on_timeline)

print("[ready] callback 已掛（含 Stop→Play 自動復活）。★ 現在按工具列 ▶ Play，左臂開始動。")
print("        停止/清除：import builtins; builtins._RMP_SUB.unsubscribe(); builtins._RMP_TL.unsubscribe()")
