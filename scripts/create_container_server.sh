SCRIPT_ABS_PATH=$(realpath "$0")
S2R_DIR=$(dirname "$(dirname "$(dirname "$SCRIPT_ABS_PATH")")")
cd "$S2R_DIR"

docker rm -f s2r_server

docker run -dit --name s2r_server --gpus all \
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

xhost +