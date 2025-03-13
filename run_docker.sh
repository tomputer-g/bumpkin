xhost +local:root 
docker container prune -f 
docker run --privileged --rm -it \
    --name="frankapy_docker" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)/src/devel_packages:/home/ros_ws/src/devel_packages" \
    -v "$(pwd)/data:/home/ros_ws/data" \
    -v "$(pwd)/guide_mode.py:/home/ros_ws/guide_mode.py" \
    -v "$(pwd)/reset_joints.py:/home/ros_ws/reset_joints.py" \
    -v "/etc/timezone:/etc/timezone:ro" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    --gpus all \
    frankapy_docker bash