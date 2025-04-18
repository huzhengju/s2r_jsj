import os
os.environ["DISCOVERSE_ASSERT_DIR"] = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), 'models')
os.system("echo $DISCOVERSE_ASSERT_DIR")

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from discoverse.utils import get_site_tmat
from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.examples.ros2.mmk2_ros2 import MMK2ROS2

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray

cfg = MMK2Cfg()
cfg.mjcf_file_path = "mjcf/s2r2025_env.xml"

cfg.init_key = "pick"
cfg.sync     = False
cfg.headless = False
cfg.render_set = {
    "fps"    : 24,
    "width"  : 640,
    "height" : 480
}
cfg.obj_list = [
    "box_carton" , "box_disk"    , "box_sheet"     ,
    "carton_01"  , "disk_01"     , "sheet_01"      ,
    "disk_02"    , "sheet_02"    ,
    "apple"      , "book"        , "cup"           ,
    "kettle"     , "scissors"    , "timeclock"     ,
    "plate"      , "xbox"        , "yellow_bowl"   ,
    "toy_cabinet", "cabinet_door", "cabinet_drawer",
]

cfg.obs_rgb_cam_id = None
cfg.obs_depth_cam_id = None
cfg.use_gaussian_renderer = False
cfg.lidar_s2_sim = False

class S2RNodeVIS(MMK2ROS2):
    gadgets_names = [
        "apple"     , "book" , "cup"  , "kettle"      , "scissors",
        "timeclock" , "plate" , "xbox" , "yellow_bowl" , "toy_cabinet"
    ]
    box_names = ["box_carton", "box_disk", "box_sheet"]
    props_names = ["carton_01", "disk_01", "sheet_01", "disk_02", "sheet_02"]

    def __init__(self, config):
        super().__init__(config)

        self.recv_odom_ = False  # 是否接收到里程计数据 | Whether odometry data is received
        self.recv_joint_states_ = False  # 是否接收到关节状态 | Whether joint states are received
        self.init_jotnt_seq_ = False
        self.last_detect_res_msg = None  # 上一次检测结果 | Last detection result
        
        # 观测状态 | Observation state
        self.obs = {
            "time": None,  # 时间 | Time
            "jq": np.array([0., 0., 0.,
                            0., 0., 0., 0., 0., 0., 0., 
                            0., 0., 0., 0., 0., 0., 0.]),  # 关节角度 | Joint angles
            "base_position": [0., 0., 0.],  # 底盘位置 | Base position
            "base_orientation": [1., 0., 0., 0.],  # 底盘方向（四元数w,x,y,z） | Base orientation (quaternion w,x,y,z)
        }
        self.joint_seq = np.zeros_like(self.obs["jq"], dtype=np.int32)

        self.init_subscription()
        self.timer = self.create_timer(1./24., self.update_display)

    def reset(self):
        ret = super().reset()
        for names in [self.gadgets_names, self.box_names, self.props_names]:
            # 随机生成物体位置 | Randomly generate object positions
            for n in names:
                qadr = self.mj_model.jnt_qposadr[np.where(self.mj_model.jnt_bodyid == self.mj_data.body(n).id)[0]][0]
                self.mj_data.qpos[qadr:qadr+3] = np.array([0.0, 0.0, -2.0]) + np.random.rand(3) * 1.5

        return ret

    def init_subscription(self):
        """
        初始化所有ROS话题订阅
        Initialize all ROS topic subscriptions
        """
        self.sub_odom = self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_callback, 10)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.sub_detect = self.create_subscription(Detection2DArray, '/yolo/detections', self._detection_callback, 10)

    def odom_callback(self, msg):
        """
        里程计数据回调函数
        Odometry data callback function
        
        Args:
            msg: 里程计消息 | Odometry message
        """
        # 提取位置信息 | Extract position information
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z
        self.obs["base_position"] = [position_x, position_y, position_z]

        # 提取方向信息（四元数） | Extract orientation information (quaternion)
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w
        self.obs["base_orientation"] = [orientation_w, orientation_x, orientation_y, orientation_z]
        self.recv_odom_ = True

    def joint_states_callback(self, msg:JointState):
        """
        关节状态回调函数
        Joint states callback function
        
        Args:
            msg: 关节状态消息 | Joint states message
        """
        if not self.init_jotnt_seq_:
            joint_names = [
                "slide_joint", "head_yaw_joint", "head_pitch_joint",
                "left_arm_joint1" , "left_arm_joint2" , "left_arm_joint3" , "left_arm_joint4" , "left_arm_joint5" , "left_arm_joint6" , "left_arm_eef_gripper_joint" ,
                "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_eef_gripper_joint",
            ]

            if len(msg.name) != len(joint_names):
                print(f"Joint names length mismatch: {len(msg.name)} != {len(joint_names)}")
                return 

            msg_name = list(msg.name)
            for i, n in enumerate(joint_names):
                try:
                    self.joint_seq[i] = msg_name.index(n)
                except ValueError:
                    print(f"Joint name {n} not found in message")
                    return
            print(f"Joint sequence: {self.joint_seq}")
            self.init_jotnt_seq_ = True            

        self.obs["jq"] = np.array(msg.position)
        # 分割关节角度到各个子系统 | Split joint angles to subsystems
        self.sensor_slide_qpos = self.obs["jq"][self.joint_seq[:1]]  # 升降关节角度 | Slide joint angle
        self.sensor_head_qpos  = self.obs["jq"][self.joint_seq[1:3]]  # 头部关节角度 | Head joint angles
        self.sensor_lft_arm_qpos  = self.obs["jq"][self.joint_seq[3:9]]  # 左臂关节角度 | Left arm joint angles
        self.sensor_lft_gripper_qpos  = self.obs["jq"][self.joint_seq[9:10]]  # 左爪关节角度 | Left gripper joint angle
        self.sensor_rgt_arm_qpos  = self.obs["jq"][self.joint_seq[10:16]]  # 右臂关节角度 | Right arm joint angles
        self.sensor_rgt_gripper_qpos  = self.obs["jq"][self.joint_seq[16:17]]  # 右爪关节角度 | Right gripper joint angle
        self.recv_joint_states_ = True

    def _detection_callback(self, msg:Detection2DArray):
        self.last_detect_res_msg = msg

    def update_robot_state(self):
        """
        更新机器人状态
        Update robot state
        """
        if self.recv_odom_:
            self.mj_data.qpos[:3] = self.obs["base_position"]
            self.mj_data.qpos[3:7] = self.obs["base_orientation"]
            self.recv_odom_ = False
        
        if self.recv_joint_states_:
            self.mj_data.qpos[9:10] = self.sensor_slide_qpos
            self.mj_data.qpos[10:12] = self.sensor_head_qpos
            self.mj_data.qpos[12:18] = self.sensor_lft_arm_qpos
            self.mj_data.qpos[18:19] = self.sensor_lft_gripper_qpos / 25.
            self.mj_data.qpos[19:20] = -self.sensor_lft_gripper_qpos / 25.
            self.mj_data.qpos[20:26] = self.sensor_rgt_arm_qpos
            self.mj_data.qpos[26:27] = self.sensor_rgt_gripper_qpos / 25.
            self.mj_data.qpos[27:28] = -self.sensor_rgt_gripper_qpos / 25.
            self.recv_joint_states_ = False

    def update_object_pose(self):
        if self.last_detect_res_msg is None:
            return

        tmat_head_camera = get_site_tmat(self.mj_data, "headeye")
        tmat_base = get_site_tmat(self.mj_data, "base_link")

        for detection in self.last_detect_res_msg.detections:
            # 获取相机坐标系中的物体位置 | Get object position in camera frame
            obj_pose_wrt_cam = np.array([
                detection.results[0].pose.pose.position.x,
                detection.results[0].pose.pose.position.y,
                detection.results[0].pose.pose.position.z,
                1.0
            ])
 
            obj_world_pose = tmat_head_camera @ obj_pose_wrt_cam

            cid = detection.results[0].hypothesis.class_id
            if cid == "name":
                n = cid + "_carton"
                tmat_trans = np.eye(3)
            elif cid == "carton":
                n = cid + "_01"
                tmat_trans = np.eye(3)
            elif cid == "disk":
                n = cid + "_01"
                tmat_trans = Rotation.from_euler('y', -np.pi/2.).as_matrix()
            elif cid == "sheet":
                n = cid + "_01"
                tmat_trans = Rotation.from_euler('y', -np.pi/2.).as_matrix()
            else:
                continue

            qadr = self.mj_model.jnt_qposadr[np.where(self.mj_model.jnt_bodyid == self.mj_data.body(n).id)[0]][0]
            self.mj_data.qpos[qadr:qadr+3] = obj_world_pose[:3]
            self.mj_data.qpos[qadr+3:qadr+7] = Rotation.from_matrix(tmat_base[:3,:3] @ tmat_trans).as_quat()[[3,0,1,2]]

    def update_display(self):
        self.update_robot_state()
        self.update_object_pose()

        self.mj_data.qvel[:] = 0
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.render()

        if not self.running:
            self.get_logger().info('退出场景可视化节点')
            raise KeyboardInterrupt

if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
 
    sim_node = S2RNodeVIS(cfg)
    obs = sim_node.reset()

    try:
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        pass

    finally:
        sim_node.destroy_node()
        rclpy.shutdown()
