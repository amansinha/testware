xhost +
docker run --gpus all -it --rm --network="host" --name=testware -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix testware-docker
