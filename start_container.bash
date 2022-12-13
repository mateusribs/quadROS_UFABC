
xhost local:root

XAUTH=/tmp/.docker.xauth

docker run --rm -it \
        --name=linux_gui_r1 \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --privileged \
        teste:latest \
        bash

echo "Container Closed"