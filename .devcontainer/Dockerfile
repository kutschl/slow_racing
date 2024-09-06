ARG DOCKER_REPO
ARG ROS_DISTRO
ARG IMAGE_SUFFIX
FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX
ARG USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
    
# Install additonal packages - add any that you need
RUN apt-get update && apt-get upgrade -y 

RUN apt-get install -y \
    python3-pip \
    python-is-python3 \
    ssh \
    neovim \
    git \
    nano \
    build-essential \
    cmake \
    virtualenvwrapper \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-urg-node \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-teleop-tools \
    ros-$ROS_DISTRO-serial-driver \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-amcl \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-asio-cmake-module \
    ros-$ROS_DISTRO-odom-to-tf-ros2

RUN pip install --no-cache-dir --upgrade pip

RUN pip install --no-cache-dir \
    pandas \
    transform3d

# Set the default user
USER $USERNAME
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# F1TENTH gym
RUN source /usr/share/virtualenvwrapper/virtualenvwrapper.sh && \
    mkvirtualenv gym_env && \
    workon gym_env && \
    cd ~ && \
    git clone https://github.com/f1tenth/f1tenth_gym && \
    cd f1tenth_gym && \
    pip install -e . && \
    pip install transforms3d

# MPC
RUN cd ~ && \
    git clone --recursive https://github.com/acados/acados.git

RUN source /usr/share/virtualenvwrapper/virtualenvwrapper.sh && \
    mkvirtualenv mpc_env && \
    workon mpc_env && \
    pip install PyYAML && \
    pip install trajectory_planning_helpers && \
    pip install --upgrade quadprog && \
    cd ~/acados && \
    mkdir -p build && \
    cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make install -j4 && \
    cd .. && \
    pip install -e ./interfaces/acados_template 
    
# RUN pip install
#RUN pip install --no-cache-dir \
#    "gym==0.18.3" \
#    #"numpy==1.22.0" \
#    "scipy==1.14.1" \
#    "numpy==2.1.1" \
#    transforms3d \
#    "pandas==1.3.5" \
#    "casadi==3.6.6" \
#    "quadprog==0.1.12"


# RUN pip install --no-cache-dir --upgrade\
#     pandas \
#     "scipy==1.14.1" \
#     "numpy>2.1" \
#     transforms3d \
#     casadi \
#     quadprog \
#     pybind11 \
#     trajectory_planning_helpers \
#     "matplotlib==3.9.2" \
#     "cython==3.0.11" 

# RUN cd acados && \
#     mkdir -p build && \
#     cd build && \
#     cmake -DACADOS_WITH_QPOASES=ON .. && \
#     make install -j4 

# RUN cd acados && \
#     pip install -e ./interfaces/acados_template 


#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/sim_ws/acados/lib"

#ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib && \
#ENV ACADOS_SOURCE_DIR=/path/to/acados

# Add .local/bin to PATH
RUN echo "export PATH=/home/$USERNAME/.local/bin${PATH:+:${PATH}}" >> /home/$USERNAME/.bashrc

# Source ROS environment automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /sim_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/virtualenvwrapper/virtualenvwrapper.sh" >> /home/$USERNAME/.bashrc
RUN echo "export ACADOS_SOURCE_DIR=~/acados" >> /home/$USERNAME/.bashrc
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib' >> ~/.bashrc

CMD ["/bin/bash"]