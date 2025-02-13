# Installation

## Clone DISCOVERSE

```bash
mkdir S2R_ws
git clone https://github.com/TATP-233/DISCOVERSE.git --recursive
git clone https://github.com/DISCOVER-Robotics/SIM2REAL-2025.git
```

## Build server

>   ❗️ <PATH-TO-S2R_ws> 要换成本地`S2R_ws`的绝对路径，例如`/home/xxx/ws/S2R_ws`

```bash
cd S2R_ws/SIM2REAL-2025/docker
docker build -f Dockerfile.server -t discoverse:s2r_server <PATH-TO-S2R_ws>

cd S2R_ws
docker run -dit \
    --name s2r_server \
    --gpus all \
    --privileged=true \
    --network=host \
    --ipc=host \
    --pid=host \
    -e ROS_DOMAIN_ID=99 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/input:/dev/input \
    -v $(pwd):/workspace \
    discoverse:s2r_server bash
```

## Build client

>   ❗️ <YOUR-TEAM-NAME>:<TAG>要替换成参赛队伍名和TAG，例如 `team_A:v1`

```bash
cd S2R_ws/SIM2REAL-2025/docker
docker build -f Dockerfile.client -t <YOUR-TEAM-NAME>:<TAG> .

docker run -dit \
    --network=host \
    --ipc=host \
    --pid=host \
    -e ROS_DOMAIN_ID=99 \
    -e ROS_LOCALHOST_ONLY=0 \
    <YOUR-TEAM-NAME>:<TAG> bash
```

## Test ros2 communication

>   ❗️ <CLIENT_CONTAINER_ID>要替换成选手自己构建的client docker container的id，可用`docker ps -a`指令查询

```bash
# server --> client 通信测试
(new terminal)
docker exec -it s2r_server bash
ros2 topic pub /server_test std_msgs/msg/String "data: 'hello from server'"

(new terminal)
docker exec -it <CLIENT_CONTAINER_ID> bash
# 查看所有活动的topics
ros2 topic list
# 查看topic信息
ros2 topic info /server_test
# 测试订阅server发布的消息
ros2 topic echo /server_test

# client --> server 通信测试
(new terminal)
docker exec -it <CLIENT_CONTAINER_ID> bash
ros2 topic pub /client_test std_msgs/msg/String "data: 'hello from client'"

(new terminal)
docker exec -it s2r_server bash
# 测试订阅server发布的消息
ros2 topic echo /client_test

```