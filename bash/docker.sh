if [ -e /dev/nvidia0 ]; then
  echo "Launch with nvidia support."
  docker run \
    -it \
    -u zima \
    --name="gtrl" \
    --net=host \
    --privileged \
    -v /home/ubuntu/workspace:/home/zima/workspace \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /home/zima/workspace \
    -v /dev:/dev \
    --runtime=nvidia \
    --device /dev/nvidia0 \
    --device /dev/nvidia-uvm \
    --device /dev/nvidia-uvm-tools \
    --device /dev/nvidiactl \
    --gpus all \
    zima:v2
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
