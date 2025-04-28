# SIM2REAL-2025

[sim阶段tutorial](doc/tutorial.md)

## Update

- [2025.4.21] 重要更新：real 阶段发布
- [2025.4.28] add act baseline

## Real阶段

恭喜各位选手进入Real阶段！

### 重要更新
！请选手务必阅读这一部分！

1. 修改头部俯仰关节旋转方向，编码器零点不变，方向与之前相反
2. 关节状态话题`/joint_states`中的关节顺序发生变化，不保证每次topic中的position、velocity固定，顺序处理方式参考docker镜像`discoverse/s2r2025_client:real_v0`路径`/workspace/s2r2025_baseline_yolo/task_base.py`中的`joint_state_callback`函数（line 411-418）
3. 仿真中的相机为理想相机模型，不存在畸变，真机头部相机的内参为：
    ```txt
    mtx:
     [[601.30624   0.      307.18915]
     [  0.      601.19196 234.84905]
     [  0.        0.        1.     ]]
    dist: (k1,k2,p1,p2,k3)
     [ 0.01781,  0.50432, -0.00077, -0.00304, -1.8273 ]
    ```
4. 通讯架构更换为了ubuntu24下的ros/jazzy，通信中间件替换为了`zenoh`，选手在仿真中调试时需要启动zenoh的通讯节点，具体操作为在仿真`s2r_server.py`之前，在server容器的终端中运行:
    ```bash
    bash /workspace/run_ros_node.sh
    ```
    注意：无需在client中启动


### 创建比赛环境

#### 拉取镜像

```bash
# 拉取server镜像
docker pull discoverse/s2r2025_server:real_v0
# 拉取client镜像
docker pull discoverse/s2r2025_client:real_v1
```

#### 创建容器

```bash
cd SIM2REAL-2025/scripts

# 检查 `create_container_server.sh`中的tag
# 创建server容器
bash create_container_server.sh 

# 检查 `create_container_client.sh`中的tag
# 创建client容器
bash create_container_client.sh 
```

#### 启动比赛

```bash
# 启动 zenoh 通讯节点
(new terminal)
cd SIM2REAL-2025/scripts
bash exec_server.sh
bash /workspace/run_ros_node.sh

# 启动一回合比赛
(new terminal)
cd SIM2REAL-2025/scripts
bash exec_server.sh
cd /workspace/SIM2REAL-2025/s2r2025
python3 s2r_server.py --round_id 1 --random_seed 99
# 注意：实际比赛时的random_seed 是随机的， 选手需要考虑各种情况

# 使用yolo baseline
(new terminal)
cd SIM2REAL-2025/scripts
bash exec_client.sh
cd /workspace/s2r2025_baseline_yolo
python3 baseline_round1_seed99.py --verbose 
# 如果报错，去掉--verbose
# 去掉--verbose可隐藏可视化窗口
# 注意 选手提交自己的镜像时，推荐把可视化窗口关闭，减小因为显示而报错的概率

# 使用act baseline
cd /workspace/s2r2025_baseline_act
python3 run.py

# tutorial在/workspace/s2r2025_baseline_yolo/tutorial.md 和 /workspace/s2r2025_baseline_act/tutorial.md
```
