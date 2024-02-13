xhost local:root
XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=ros2_tb3_stages_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    ros2_tb3_stages \
    bash

echo "Done."