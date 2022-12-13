
xhost local:root

XAUTH=/tmp/.docker.xauth

docker run --rm -it \
        --name=quad_ufabc_ros \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --privileged \
        quad_ufabc:latest \
        bash

echo "Container Closed"