xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./sim_gazeboHarmonic_ws:/home/$USER/sim_gazeboHarmonic_ws/ \
           -it \
           --gpus all \
           --name ros2_jazzy_gz_harmonic arambarricalvoj/ros2-jazzy-gazebo-harmonic:latest
