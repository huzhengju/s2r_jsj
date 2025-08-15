import os
import sys
import traceback
import glfw
import mujoco
import argparse
import threading
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, PointCloud2, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# 设置路径
current_dir = os.path.dirname(os.path.realpath(__file__))
project_root = os.path.dirname(current_dir)
models_dir = os.path.join(project_root, "models")
ply_dir = os.path.join(project_root, "models", "3dgs")
os.environ["DISCOVERSE_ASSERT_DIR"] = models_dir
sys.path.insert(0, project_root)

# 导入自定义模块
from discoverse.robots_env.mmk2_base import MMK2Cfg
from discoverse.examples.ros2.mmk2_ros2 import MMK2ROS2

# 修改TaskInfo类定义，使其可以接收参数
class TaskInfo:
    def __init__(self, instruction=None, target_gadget=None):
        self.scoring = {"a": False, "b": False, "c": False, "d": False}
        self.scoring_time = {"a": -1., "b": -1., "c": -1., "d": -1.}
        self.instruction = instruction if instruction else ""
        self.target_gadget = target_gadget if target_gadget else ""
        self.round = 0
        self.target_box_name = ""
        self.target_box_qpos_id = -1
        self.target_prop_type = ""
        self.target_prop_name = ""
        self.target_prop_qpos_id = -1
        self.table_direction = ""
        self.drawer_layer = ""
        self.placement_target = ""
    
    def reset(self):
        for k in self.scoring:
            self.scoring[k] = False
        for k in self.scoring_time:
            self.scoring_time[k] = -1.
    
    def check_mission_done(self, round_id):
        if round_id == 1:
            return self.scoring["a"] and self.scoring["b"] and self.scoring["c"]
        elif round_id == 2:
            return self.scoring["a"] and self.scoring["b"] and self.scoring["c"]
        elif round_id == 3:
            return self.scoring["a"] and self.scoring["b"] and self.scoring["c"] and self.scoring["d"]
        return False

# 辅助函数
def box_within_cabinet(box_pos):
    return box_pos[0] > -1.0 and box_pos[0] < 1.0 and box_pos[1] < -1.5 and box_pos[2] < 1.0

def prop_in_gripper(gripper_left_pos, gripper_right_pos, prop_pos, prop_type):
    dist_left = np.linalg.norm(gripper_left_pos - prop_pos)
    dist_right = np.linalg.norm(gripper_right_pos - prop_pos)
    return dist_left < 0.1 and dist_right < 0.1

def prop_within_table(prop_pos, table_direction):
    if table_direction == "left":
        return prop_pos[1] > 0 and prop_pos[2] > 0.75
    elif table_direction == "right":
        return prop_pos[1] < 0 and prop_pos[2] > 0.75
    return False

# 位置信息
s2r2025_position_info = {
    "cabinet": {
        "position": [
            [[0.5, -1.6, 0.5], [0.707, 0, 0, 0.707]],
            [[-0.5, -1.6, 0.5], [0.707, 0, 0, 0.707]],
            [[0.5, -1.6, 1.0], [0.707, 0, 0, 0.707]],
            [[-0.5, -1.6, 1.0], [0.707, 0, 0, 0.707]]
        ],
        "info": [
            "left_second_floor",
            "right_second_floor",
            "left_third_floor",
            "right_third_floor"
        ]
    },
    "table": {
        "position": [
            [[0.0, 1.0, 0.75], [0.707, 0, 0, 0.707]],
            [[0.0, -1.0, 0.75], [0.707, 0, 0, 0.707]],
            [[1.0, 0.0, 0.75], [0.707, 0, 0, 0.707]],
            [[-1.0, 0.0, 0.75], [0.707, 0, 0, 0.707]]
        ],
        "info": [
            "front,left,right",
            "front,back,left,right",
            "front,back,left",
            "front,back,right"
        ]
    },
    "carton": {
        "position": [
            [[0.1, 0.1, 0.1], [0.707, 0, 0, 0.707]],
            [[-0.1, 0.1, 0.1], [0.707, 0, 0, 0.707]]
        ]
    },
    "disk": {
        "position": [
            [[0.1, -0.1, 0.1], [0.707, 0, 0, 0.707]],
            [[-0.1, -0.1, 0.1], [0.707, 0, 0, 0.707]]
        ]
    },
    "sheet": {
        "position": [
            [[0.0, 0.0, 0.1], [0.707, 0, 0, 0.707]],
            [[0.0, 0.0, 0.2], [0.707, 0, 0, 0.707]]
        ]
    }
}

# 配置
cfg = MMK2Cfg()
cfg.mjcf_file_path = os.path.join(models_dir, "mjcf/5_store.xml")
cfg.timestep = 0.003
cfg.decimation = 2
cfg.init_key = "pick"
cfg.sync = True
cfg.headless = False
cfg.render_set = {
    "fps": 24,
    "width": 1280,
    "height": 720
}

# 更新物体列表
cfg.obj_list = [
    "agv_link", "slide_link", "head_pitch_link", "head_yaw_link",
    "lft_arm_base", "lft_arm_link1", "lft_arm_link2", 
    "lft_arm_link3", "lft_arm_link4", "lft_arm_link5",
    "lft_arm_link6", "lft_finger_left_link", "lft_finger_right_link",
    "rgt_arm_base", "rgt_arm_link1", "rgt_arm_link2",
    "rgt_arm_link3", "rgt_arm_link4", "rgt_arm_link5",
    "rgt_arm_link6", "rgt_finger_left_link", "rgt_finger_right_link",
    "blue", "toolbox", "hat", "drill", "whitebox"  # 添加drill和whitebox
]

# 更新3D模型路径字典
cfg.gs_model_dict = {
    "agv_link": os.path.join(ply_dir, "mmk2", "agv_link.ply"),
    "slide_link": os.path.join(ply_dir, "mmk2", "slide_link.ply"),
    "head_pitch_link": os.path.join(ply_dir, "mmk2", "head_pitch_link.ply"),
    "head_yaw_link": os.path.join(ply_dir, "mmk2", "head_yaw_link.ply"),
    "lft_arm_base": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "lft_arm_link1": os.path.join(ply_dir, "airbot_play", "link1.ply"),
    "lft_arm_link2": os.path.join(ply_dir, "airbot_play", "link2.ply"),
    "lft_arm_link3": os.path.join(ply_dir, "airbot_play", "link3.ply"),
    "lft_arm_link4": os.path.join(ply_dir, "airbot_play", "link4.ply"),
    "lft_arm_link5": os.path.join(ply_dir, "airbot_play", "link5.ply"),
    "lft_arm_link6": os.path.join(ply_dir, "airbot_play", "link6.ply"),
    "lft_finger_left_link": os.path.join(ply_dir, "airbot_play", "left.ply"),
    "lft_finger_right_link": os.path.join(ply_dir, "airbot_play", "right.ply"),
    "rgt_arm_base": os.path.join(ply_dir, "airbot_play", "arm_base.ply"),
    "rgt_arm_link1": os.path.join(ply_dir, "airbot_play", "link1.ply"),
    "rgt_arm_link2": os.path.join(ply_dir, "airbot_play", "link2.ply"),
    "rgt_arm_link3": os.path.join(ply_dir, "airbot_play", "link3.ply"),
    "rgt_arm_link4": os.path.join(ply_dir, "airbot_play", "link4.ply"),
    "rgt_arm_link5": os.path.join(ply_dir, "airbot_play", "link5.ply"),
    "rgt_arm_link6": os.path.join(ply_dir, "airbot_play", "link6.ply"),
    "rgt_finger_left_link": os.path.join(ply_dir, "airbot_play", "left.ply"),
    "rgt_finger_right_link": os.path.join(ply_dir, "airbot_play", "right.ply"),
    "background": os.path.join(ply_dir, "scene", "s2r2025_new", "5_stroe_translated.ply"),
    "blue": os.path.join(ply_dir, "s2r2025_new", "blue.ply"),
    "toolbox": os.path.join(ply_dir, "s2r2025_new", "toolbox.ply"),
    "hat": os.path.join(ply_dir, "s2r2025_new", "hat.ply"),
    "drill": os.path.join(ply_dir, "s2r2025_new", "drill.ply"),
    "whitebox": os.path.join(ply_dir, "s2r2025_new", "brownbox.ply")
    # "whitebox": os.path.join(ply_dir, "s2r2025_new", "greybox.ply")
    # "whitebox": os.path.join(ply_dir, "s2r2025_new", "yellowbox.ply")
    }

cfg.obs_rgb_cam_id = [0, 1, 2]
cfg.obs_depth_cam_id = [0]
cfg.use_gaussian_renderer = True
cfg.lidar_s2_sim = True

class S2RNode(MMK2ROS2):
    # 更新物体分类
    gadgets_names = ["blue", "toolbox", "hat", "drill"]  # 添加drill
    box_names = ["whitebox"]  # 添加whitebox
    
    def __init__(self, config):
        super().__init__(config)
        self.taskInfos = {
            "round1": TaskInfo(),
            "round2": TaskInfo(),
            "round3": TaskInfo()
        }
        # 设置任务指令和目标物体
        self.taskInfos["round1"].instruction = "Move the blue box to the left table."
        self.taskInfos["round1"].target_gadget = "blue"
        self.taskInfos["round2"].instruction = "Place the drill in the toolbox."
        self.taskInfos["round2"].target_gadget = "drill"  # 改为drill
        self.taskInfos["round3"].instruction = "Put the hat on the drill and place it on the whitebox."
        self.taskInfos["round3"].target_gadget = "hat"
        
        self.round_id = self.config.round_id
        self.first_recv_cmd_time = -1.
        self.cmd_recv_lock = threading.Lock()

        self.clock_publisher_ = self.create_publisher(Clock, '/clock', 10)
        timer_period = 0.01
        self.clock_timer = self.create_timer(timer_period, self.clock_callback)

    def clock_callback(self):
        msg = Clock()
        msg.clock.sec = int(self.mj_data.time)
        msg.clock.nanosec = int((self.mj_data.time - int(self.mj_data.time)) * 1e9)
        self.clock_publisher_.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        with self.cmd_recv_lock:
            if self.first_recv_cmd_time < 0.:
                self.first_recv_cmd_time = self.mj_data.time
        super().cmd_vel_callback(msg)

    def cmd_spine_callback(self, msg: Float64MultiArray):
        with self.cmd_recv_lock:
            if self.first_recv_cmd_time < 0.:
                self.first_recv_cmd_time = self.mj_data.time
        super().cmd_spine_callback(msg)

    def cmd_head_callback(self, msg: Float64MultiArray):
        with self.cmd_recv_lock:
            if self.first_recv_cmd_time < 0.:
                self.first_recv_cmd_time = self.mj_data.time
        super().cmd_head_callback(msg)

    def cmd_left_arm_callback(self, msg: Float64MultiArray):
        with self.cmd_recv_lock:
            if self.first_recv_cmd_time < 0.:
                self.first_recv_cmd_time = self.mj_data.time
        super().cmd_left_arm_callback(msg)

    def cmd_right_arm_callback(self, msg: Float64MultiArray):
        with self.cmd_recv_lock:
            if self.first_recv_cmd_time < 0.:
                self.first_recv_cmd_time = self.mj_data.time
        super().cmd_right_arm_callback(msg)

    def post_load_mjcf(self):
        super().post_load_mjcf()
        
        # 添加关键帧验证
        if hasattr(self.mj_model, 'nkey'):
            print(f"模型关键帧数量: {self.mj_model.nkey}")
            for i in range(self.mj_model.nkey):
                key = self.mj_model.key(i)
                print(f"关键帧 {i}: qpos长度={len(key.qpos)}, 期望长度={self.mj_model.nq}")
                if len(key.qpos) != self.mj_model.nq:
                    print(f"错误: 关键帧 {i} 的 qpos 长度不匹配!")
        else:
            print("警告: 模型中没有定义关键帧")
        
        self.gadgets_info = {}
        for n in self.gadgets_names:
            body_id = self.mj_data.body(n).id
            joint_ids = np.where(self.mj_model.jnt_bodyid == body_id)[0]
            if joint_ids.size > 0:
                self.gadgets_info[n] = self.mj_model.jnt_qposadr[joint_ids[0]]

        self.boxes_info = {}
        for n in self.box_names:
            body_id = self.mj_data.body(n).id
            joint_ids = np.where(self.mj_model.jnt_bodyid == body_id)[0]
            if joint_ids.size > 0:
                self.boxes_info[n] = self.mj_model.jnt_qposadr[joint_ids[0]]

        self.props_info = {}

    def reset(self):
        ret = super().reset()
        for k in self.taskInfos:
            self.taskInfos[k].reset()
            if int(k[-1]) < 3:
                if "d" in self.taskInfos[k].scoring:
                    del self.taskInfos[k].scoring["d"]
                if "d" in self.taskInfos[k].scoring_time:
                    del self.taskInfos[k].scoring_time["d"]
        
        self.mj_data.body("blue").xpos = np.array([0.126002, -1.60464, 0.522424])
        self.mj_data.body("blue").xquat = np.array([0.492386, 0.505636, 0.500052, 0.501834])
        
        self.mj_data.body("toolbox").xpos = np.array([0.0741834, -1.55309, 1.1106])
        self.mj_data.body("toolbox").xquat = np.array([0.668304, 0.742876, 0.0288485, 0.0259333])
        
        self.mj_data.body("hat").xpos = np.array([0.103415, -1.59953, 0.836737])
        self.mj_data.body("hat").xquat = np.array([-0.134248, -0.123952, -0.672913, -0.716799])

        self.mj_data.body("drill").xpos = np.array([0.0580176, -1.65665, 0.225595])
        self.mj_data.body("drill").xquat = np.array([0.519871, 0.624074, 0.398061, 0.426395])
        
        self.mj_data.body("whitebox").xpos = np.array([0.1, -1.60047, -0.107685])
        self.mj_data.body("whitebox").xquat = np.array([0.509155, 0.489126, -0.490992, -0.510337])

        r = f"round{self.round_id}"
        if self.round_id == 1:
            print("警告: round 1")
        elif self.round_id == 2:
            self.taskInfos[r].instruction = "Place the drill in the toolbox."
            self.taskInfos[r].target_gadget = "drill"
        elif self.round_id == 3:
            self.taskInfos[r].instruction = "Put the hat on the drill and place it on the whitebox."
            self.taskInfos[r].target_gadget = "hat"
        
        print('-' * 100)
        print(f"{r}: {self.taskInfos[r].instruction}")
        return ret
    
    def printScore(self):
        r = f"round{self.round_id}"
        tinfo = self.taskInfos[r]
        print(f"{r}: {tinfo.instruction}")
        print(f"    scoring : {tinfo.scoring}")
        print(f"    scoring_time : {tinfo.scoring_time}")

    def printMessage(self):
        ret = super().printMessage()
        self.printScore()
        return ret

    def on_key(self, window, key, scancode, action, mods):
        super().on_key(window, key, scancode, action, mods)
        if action == glfw.PRESS:
            if key == glfw.KEY_C:
                pass

    def pubContactInfoOnce(self):
        print(np.array2string(self.mj_data.contact.geom, separator=',', suppress_small=True))
        
        round_str = f"round{self.round_id}"
        prop_name = self.taskInfos[round_str].target_gadget

        for bd in [prop_name, "lft_finger_left_link", "lft_finger_right_link", "rgt_finger_left_link", "rgt_finger_right_link"]:
            bd_gemo_id_range = (self.mj_model.body(bd).geomadr[0], self.mj_model.body(bd).geomadr[0]+self.mj_model.body(bd).geomnum[0])
            print(bd, bd_gemo_id_range)

        print(self.check_contact(prop_name, "lft_finger_right_link"), self.check_contact(prop_name, "lft_finger_left_link"))
        print(self.check_contact(prop_name, "rgt_finger_right_link"), self.check_contact(prop_name, "rgt_finger_left_link"))

    def check_contact(self, body1, body2):
        body1_gemo_id_range = (self.mj_model.body(body1).geomadr[0], self.mj_model.body(body1).geomadr[0]+self.mj_model.body(body1).geomnum[0])
        body2_gemo_id_range = (self.mj_model.body(body2).geomadr[0], self.mj_model.body(body2).geomadr[0]+self.mj_model.body(body2).geomnum[0])
        b1r = body1_gemo_id_range if body1_gemo_id_range[0] < body2_gemo_id_range[0] else body2_gemo_id_range
        b2r = body2_gemo_id_range if body1_gemo_id_range[0] < body2_gemo_id_range[0] else body1_gemo_id_range

        for i in range(self.mj_data.ncon):
            geom_id = sorted(self.mj_data.contact.geom[i].tolist())
            if b1r[0] <= geom_id[0] < b1r[1] and b2r[0] <= geom_id[1] < b2r[1]:
                return True
        return False

    def post_physics_step(self):
        round_str = f"round{self.round_id}"
        gadget_name = self.taskInfos[round_str].target_gadget
        gadget_pos = self.mj_data.body(gadget_name).xpos
        
        if self.round_id == 1:
            if not self.taskInfos[round_str].scoring["a"]:
                if gadget_pos[0] < 0 and gadget_pos[2] > 0.7:
                    self.taskInfos[round_str].scoring["a"] = True
                    self.taskInfos[round_str].scoring_time["a"] = self.mj_data.time - self.first_recv_cmd_time
        
        elif self.round_id == 2:
            if not self.taskInfos[round_str].scoring["a"]:
                toolbox_pos = self.mj_data.body("toolbox").xpos
                if np.linalg.norm(gadget_pos - toolbox_pos) < 0.2:
                    self.taskInfos[round_str].scoring["a"] = True
                    self.taskInfos[round_str].scoring_time["a"] = self.mj_data.time - self.first_recv_cmd_time
        
        elif self.round_id == 3:
            if not self.taskInfos[round_str].scoring["a"]:
                drill_pos = self.mj_data.body("drill").xpos
                if np.linalg.norm(gadget_pos - drill_pos) < 0.15:
                    self.taskInfos[round_str].scoring["a"] = True
                    self.taskInfos[round_str].scoring_time["a"] = self.mj_data.time - self.first_recv_cmd_time
            
            if self.taskInfos[round_str].scoring["a"] and not self.taskInfos[round_str].scoring["b"]:
                whitebox_pos = self.mj_data.body("whitebox").xpos
                if np.linalg.norm(gadget_pos - whitebox_pos) < 0.2:
                    self.taskInfos[round_str].scoring["b"] = True
                    self.taskInfos[round_str].scoring_time["b"] = self.mj_data.time - self.first_recv_cmd_time

    def checkTerminated(self):
        mission_done = False
        if self.round_id == 1:
            mission_done = self.taskInfos[f"round{self.round_id}"].scoring.get("a", False)
        elif self.round_id == 2:
            mission_done = self.taskInfos[f"round{self.round_id}"].scoring.get("a", False)
        elif self.round_id == 3:
            mission_done = self.taskInfos[f"round{self.round_id}"].scoring.get("a", False) and \
                           self.taskInfos[f"round{self.round_id}"].scoring.get("b", False)
        
        # 超时检查
        # time_since_start = self.mj_data.time
        # if (self.first_recv_cmd_time < 0. and time_since_start > 60.) or \
        #    (self.first_recv_cmd_time >= 0. and (time_since_start - self.first_recv_cmd_time) > 5 * 60.):
        #     return True
        return mission_done

    def thread_pubGameInfo(self, freq=1):
        rate1 = self.create_rate(freq)
        r = f"round{self.round_id}"
        tinfo = self.taskInfos[r]

        self.task_info_puber = self.create_publisher(String, '/s2r2025/taskinfo', 2)
        task_info_msg = String()
        task_info_msg.data = f"{r}: {tinfo.instruction}"

        self.game_info_puber = self.create_publisher(String, '/s2r2025/gameinfo', 2)
        game_info_msg = String()

        while rclpy.ok() and self.running:
            game_info_msg.data = str({
                "scoring": tinfo.scoring,
                "scoring_time": tinfo.scoring_time,
                "first_recv_cmd_time": self.first_recv_cmd_time,
            })
            self.task_info_puber.publish(task_info_msg)
            self.game_info_puber.publish(game_info_msg)
            rate1.sleep()

    def pubRos2TopicOnce(self):
        time_stamp = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = time_stamp
        self.joint_state.position = self.sensor_qpos[2:].tolist()
        self.joint_state.velocity = self.sensor_qvel[2:].tolist()
        self.joint_state.effort = self.sensor_force[2:].tolist()
        self.joint_state_puber.publish(self.joint_state)

        self.odom_msg.pose.pose.position.x = self.sensor_base_position[0]
        self.odom_msg.pose.pose.position.y = self.sensor_base_position[1]
        self.odom_msg.pose.pose.position.z = self.sensor_base_position[2]
        self.odom_msg.pose.pose.orientation.w = self.sensor_base_orientation[0]
        self.odom_msg.pose.pose.orientation.x = self.sensor_base_orientation[1]
        self.odom_msg.pose.pose.orientation.y = self.sensor_base_orientation[2]
        self.odom_msg.pose.pose.orientation.z = self.sensor_base_orientation[3]
        self.odom_puber.publish(self.odom_msg)

        head_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][0], encoding="rgb8")
        head_color_img_msg.header.stamp = time_stamp
        head_color_img_msg.header.frame_id = "head_camera"
        self.head_color_puber.publish(head_color_img_msg)

        head_depth_img = np.array(np.clip(self.obs["depth"][0]*1e3, 0, 65535), dtype=np.uint16)
        head_depth_img_msg = self.bridge.cv2_to_imgmsg(head_depth_img, encoding="mono16")
        head_depth_img_msg.header.stamp = time_stamp
        head_depth_img_msg.header.frame_id = "head_camera"
        self.head_depth_puber.publish(head_depth_img_msg)
        
        left_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][1], encoding="rgb8")
        left_color_img_msg.header.stamp = time_stamp
        left_color_img_msg.header.frame_id = "left_camera"
        self.left_color_puber.publish(left_color_img_msg)

        right_color_img_msg = self.bridge.cv2_to_imgmsg(self.obs["img"][2], encoding="rgb8")
        right_color_img_msg.header.stamp = time_stamp
        right_color_img_msg.header.frame_id = "right_camera"
        self.right_color_puber.publish(right_color_img_msg)

if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
 
    parser = argparse.ArgumentParser(description='Run server with specified parameters. \ne.g. python3 s2r_server.py --round_id 1 --random_seed 0')
    parser.add_argument('--round_id', type=int, choices=[1, 2, 3], help='tasks round index', required=True)
    parser.add_argument('--random_seed', type=int, help='random seed. If not specified, the random seed will not be set.', default=None, required=False)
    args = parser.parse_args()

    cfg.round_id = args.round_id
    if not args.random_seed is None:
        np.random.seed(args.random_seed)

    # 添加文件存在检查
    print(f"加载模型文件: {cfg.mjcf_file_path}")
    print(f"文件存在: {os.path.exists(cfg.mjcf_file_path)}")
    
    try:
        cfg.init_key = "pick"
        sim_node = S2RNode(cfg)
        obs = sim_node.reset()

        spin_thread = threading.Thread(target=lambda:rclpy.spin(sim_node))
        spin_thread.start()

        publidar_thread = threading.Thread(target=sim_node.thread_publidartopic, args=(12,))
        publidar_thread.start()

        pubtopic_thread = threading.Thread(target=sim_node.thread_pubros2topic, args=(24,))
        pubtopic_thread.start()

        pubgameinfo_thread = threading.Thread(target=sim_node.thread_pubGameInfo)
        pubgameinfo_thread.start()

        try:
            while rclpy.ok() and sim_node.running:
                obs, _, _, ter, _ = sim_node.step(sim_node.target_control)
                if ter:
                    sim_node.printScore()
                    sim_node.running = False
                    break

        except KeyboardInterrupt:
            pass

        finally:
            publidar_thread.join()
            pubtopic_thread.join()
            pubgameinfo_thread.join()
            sim_node.destroy_node()
            rclpy.shutdown()
            spin_thread.join()
            print("SERVER ROS2 shutdown")
            
    except Exception as e:
        print(f"初始化失败: {str(e)}")
        traceback.print_exc()
        rclpy.shutdown()