# Docker image
FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

SHELL ["/bin/bash", "-c"]
ENV PYTHONBUFFERED 1

RUN apt-get update && apt-get install -y \ 
    build-essential \
    git \
    emacs \
    tmux \
    zip \
    vim \ 
    wget \
    xdg-utils \
    apt-utils

# tensorrt
RUN wget https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
RUN apt update && apt install -y \
    libcudnn7=7.6.5.32-1+cuda10.2 \
    libcudnn7-dev=7.6.5.32-1+cuda10.2 \
    libnvinfer7=7.0.0-1+cuda10.2 \
    libnvinfer-dev=7.0.0-1+cuda10.2 \
    libnvinfer-plugin7=7.0.0-1+cuda10.2 \
    libnvinfer-plugin-dev=7.0.0-1+cuda10.2 \
    libnvonnxparsers7=7.0.0-1+cuda10.2 \
    libnvonnxparsers-dev=7.0.0-1+cuda10.2 \
    libnvparsers7=7.0.0-1+cuda10.2 \
    libnvparsers-dev=7.0.0-1+cuda10.2

COPY AutowareArchitectureProposal /AutowareArchitectureProposal
WORKDIR AutowareArchitectureProposal
COPY rosutils/simple_planning_simulator_core.cpp ./src/simulator/simple_planning_simulator/src
COPY rosutils/setup_ubuntu.sh .
RUN ./setup_ubuntu.sh


# install ros package dependencies
RUN rosdep update --rosdistro melodic && \
    rosdep install -y \
      --from-paths \
        src \
      --ignore-src --rosdistro melodic

RUN apt install -y python-catkin-tools \
    ros-melodic-roswww

RUN catkin config --extend /opt/ros/melodic && \
    catkin build -j8 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

WORKDIR /AutowareArchitectureProposal/src/perception/object_recognition/detection/lidar_apollo_instance_segmentation
RUN /bin/bash -c "source /AutowareArchitectureProposal/devel/setup.bash; cmake .; make"
WORKDIR ../tensorrt_yolo3
RUN /bin/bash -c "source /AutowareArchitectureProposal/devel/setup.bash; cmake .; make"

WORKDIR /AutowareArchitectureProposal
RUN mkdir maps
WORKDIR maps
COPY rosutils/gdown.pl .
RUN ./gdown.pl 'https://drive.google.com/open?id=1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk' rosbag_sample_map.zip
RUN unzip rosbag_sample_map.zip && \
    rm rosbag_sample_map.zip
RUN ./gdown.pl 'https://drive.google.com/open?id=1bgdo8cs8gMO05qpB3O646MhXjvpdtVLP' Town04.osm
RUN ./gdown.pl 'https://drive.google.com/open?id=1bSk2j8NiE3D-5cJPdpW_gItqexc6ryeN/' Town04.pcd


RUN pip install scipy pygame pygobject llvmlite==0.31.0 numba==0.47.0 zmq
RUN pip install colorama --upgrade 
RUN apt-get update --fix-missing && apt-get install -y \ 
    ffmpeg \
    python3-pip
RUN pip3 install numpy imageio imageio-ffmpeg


WORKDIR /AutowareArchitectureProposal
COPY rosutils/carla_simulator.launch ./src/launcher/autoware_launch/launch
COPY rosutils/mission_planning.launch ./src/launcher/planning_launch/launch/mission_planning/
COPY rosutils/carla_map.launch ./src/launcher/map_launch/launch
COPY rosutils/velodyne_VLP16.launch /AutowareArchitectureProposal/src/launcher/sensing_launch/launch/
COPY rosutils/velodyne_VLS128.launch /AutowareArchitectureProposal/src/launcher/sensing_launch/launch/

WORKDIR ../

COPY demo /demo
COPY testware /testware

ENV SDL_AUDIODRIVER dsp

WORKDIR ./demo
