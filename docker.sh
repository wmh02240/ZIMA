if [ -e /dev/nvidia0 ]; then
  echo "Launch with nvidia support."
  docker run \
    -it \
    -u zima \
    --name="zima_demo" \
    --net=host \
    --privileged \
    -v /home/ubuntu:/home/zima \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --runtime=nvidia \
    --device /dev/nvidia0 \
    --device /dev/nvidia-uvm \
    --device /dev/nvidia-uvm-tools \
    --device /dev/nvidiactl \
    --runtime=nvidia \
    --gpus all \
    zima:latest
else
  echo "Launch without nvidia support."
  docker run \
    -it \
    -u zima \
    --name="zima_demo" \
    --net=host \
    --privileged \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    bitsoullab/ros:zima-dev
fi
