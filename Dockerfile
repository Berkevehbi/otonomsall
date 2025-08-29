FROM ros:foxy

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    can-utils \
    nano \
    && rm -rf /var/lib/apt/lists/*
   

# Büyük Python paketlerini doğrudan pip ile kur (wheels olmadan)
RUN pip3 install --no-cache-dir \
    torch==2.4.1 \
    torchvision==0.19.1 \
    triton==3.0.0 \
    ultralytics==8.3.165 \
    ultralytics-thop==2.0.14 \
    opencv-python==4.12.0.88 \
    scikit-learn==1.3.2 \
    scipy==1.10.1 \
    numpy==1.21.6 \
    pandas==2.0.3 \
    matplotlib==3.7.5 \
    pillow==10.4.0 \
    fsspec==2025.3.0 \
    nvidia-cublas-cu12==12.1.3.1 \
    nvidia-cuda-cupti-cu12==12.1.105 \
    nvidia-cuda-nvrtc-cu12==12.1.105 \
    nvidia-cuda-runtime-cu12==12.1.105 \
    nvidia-cudnn-cu12==9.1.0.70 \
    nvidia-cufft-cu12==11.0.2.54 \
    nvidia-curand-cu12==10.3.2.106 \
    nvidia-cusolver-cu12==11.4.5.107 \
    nvidia-cusparse-cu12==12.1.0.106 \
    nvidia-nccl-cu12==2.20.5 \
    nvidia-nvjitlink-cu12==12.9.86 \
    nvidia-nvtx-cu12==12.1.105
    
RUN apt-get update && apt-get install -y ros-foxy-cv-bridge
RUN pip3 install yacs prefetch-generator
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rqt-image-view

RUN apt-get update && \
    apt-get install -y ros-foxy-tf-transformations

RUN pip install --no-cache-dir filterpy==1.4.5
RUN pip3 install --no-cache-dir \
    pykml==0.2.0
RUN pip3 install --no-cache-dir transforms3d
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-sensor-msgs-py && rm -rf /var/lib/apt/lists/*


# ROS workspace ve kalan adımlar klasik şekilde devam
RUN mkdir -p /root/robotaxi_ws/src
WORKDIR /root/robotaxi_ws
COPY ./src ./src
RUN rm -rf build install log
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
    
RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install
COPY ./src/robotaxi_perception/robotaxi_perception/weights/End-to-end.pth /root/robotaxi_ws/install/robotaxi_perception/share/robotaxi_perception/weights/End-to-end.pth

# Default shell açılınca otomatik source etsin!
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /root/robotaxi_ws/install/setup.bash" >> ~/.bashrc

ENV ROS_DOMAIN_ID=0
CMD ["/bin/bash"]

