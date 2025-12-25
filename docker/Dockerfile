FROM ros:humble

RUN useradd -m ubuntu && \
    usermod -aG sudo ubuntu && \
    passwd -d ubuntu && \
    sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' /home/ubuntu/.bashrc

USER ubuntu
WORKDIR /home/ubuntu

RUN sudo apt update && sudo apt upgrade -y

RUN sudo apt install -y python3-matplotlib python3-pandas

RUN rosdep update

COPY ./src pointcloud_transform_benchmark_ws/src

WORKDIR /home/ubuntu/pointcloud_transform_benchmark_ws

RUN rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

WORKDIR /home/ubuntu

RUN sudo rm -rf pointcloud_transform_benchmark_ws
