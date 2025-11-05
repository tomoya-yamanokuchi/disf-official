#!/bin/bash

USER=${USER:-$(whoami)}
DEVICE_ARGS=""

docker run --rm -it --gpus all --privileged --net=host --ipc=host \
$DEVICE_ARGS \
-e LOCAL_UID=$(id -u $USER) \
-e LOCAL_GID=$(id -g $USER) \
-e DISPLAY=$DISPLAY \
-e AUDIODEV="hw:Device, 0" \
-e XAUTHORITY=/home/$(id -un)/.Xauthority \
-v $HOME/.Xauthority:/home/$(id -un)/.Xauthority \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/$USER/disf_ras:/home/cudagl/disf_ras \
-v /home/$USER/data:/home/cudagl/data \
docker_disf_ras:latest bash -c "cd /home/cudagl/disf_ras && exec bash"
