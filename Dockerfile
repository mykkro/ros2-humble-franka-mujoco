FROM osrf/ros:humble-desktop

ENV WORKSPACE=/root/ros2_ws
ENV SHAREDIR=/root/share
ENV LIBSDIR=/root/libs
ENV TOOLSDIR=/root/tools
RUN mkdir -p $WORKSPACE
RUN mkdir -p $LIBSDIR
RUN mkdir -p $TOOLSDIR

RUN apt-get -y update 
RUN apt install software-properties-common curl gnupg lsb-release python3-pip mc wget curl featherpad dos2unix tmux -y
RUN add-apt-repository universe

RUN apt update && apt install -y software-properties-common && add-apt-repository ppa:kisak/kisak-mesa && apt upgrade -y

RUN apt-get -y update && apt install ros-humble-rosbridge-server ros-humble-foxglove-bridge vim xsltproc build-essential libpoco-dev libeigen3-dev ros-cmake-modules ament-cmake-clang-format figlet toilet -y


## Install Python libraries

RUN pip3 install flask pyyaml pyzmq cbor pyquaternion numpy pandas xmlschema


## Install Foxglove

RUN cd /tmp && wget https://github.com/foxglove/studio/releases/download/v1.50.0/foxglove-studio-1.50.0-linux-amd64.deb && chmod 666 ./foxglove-studio-1.50.0-linux-amd64.deb &&  apt install ./foxglove-studio-1.50.0-linux-amd64.deb -y && rm ./foxglove-studio-1.50.0-linux-amd64.deb


## Install franka_ros2

WORKDIR $LIBSDIR

# Install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka
WORKDIR $LIBSDIR/libfranka/
RUN mkdir build
WORKDIR $LIBSDIR/libfranka/build/
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
RUN cmake --build .

# Install libfranka Debian package
RUN cpack -G DEB
RUN dpkg -i libfranka-*.deb


## Install MUJOCO

WORKDIR $TOOLSDIR

RUN wget https://github.com/deepmind/mujoco/releases/download/2.1.1/mujoco-2.1.1-linux-x86_64.tar.gz && tar -xf mujoco-2.1.1-linux-x86_64.tar.gz && rm mujoco-2.1.1-linux-x86_64.tar.gz

## Build packages

WORKDIR $WORKSPACE/src

## Copy editable/exposed packages
COPY share/panda_configs/panda-rviz-fakehw-jt/panda2_description $WORKSPACE/src/panda2_description
COPY share/panda_configs/panda-rviz-fakehw-jt/panda2_moveit_config $WORKSPACE/src/panda2_moveit_config
COPY share/pymoveit2_demos $WORKSPACE/src/pymoveit2_demos

## Fetch git repos

RUN git clone https://github.com/mcbed/franka_ros2.git -b humble franka_ros2

RUN git clone https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git -b ros2

RUN git clone https://github.com/AndrejOrsula/pymoveit2.git

RUN apt-get -y update && apt install -y ros-humble-ament-cmake-clang-format ros-humble-moveit-msgs ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster ros-humble-controller-manager ros-humble-moveit-ros-move-group ros-humble-moveit-kinematics ros-humble-moveit-planners-ompl ros-humble-moveit-ros-visualization ros-humble-xacro ros-humble-joint-trajectory-controller ros-humble-hardware-interface ros-humble-joint-state-publisher-gui ros-humble-controller-interface ros-humble-ros-ign-gazebo ros-humble-controller-manager-msgs ros-humble-ign-ros2-control ros-humble-moveit-servo ros-humble-moveit ros-humble-ros-ign-bridge

# warehouse-ros-mongo not available on jammy
# https://github.com/ros-planning/warehouse_ros_mongo/issues/71
#### RUN apt-get -y update && apt install -y ros-humble-warehouse-ros-mongo


WORKDIR $WORKSPACE
## RUN chmod 777 -R .

SHELL ["/bin/bash", "-c"]

WORKDIR $WORKSPACE

RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip cartesian_controller_simulation cartesian_controller_tests"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --cmake-args -DMUJOCO_DIR=/root/tools/mujoco-2.1.1 -DCMAKE_BUILD_TYPE=Release --packages-select cartesian_controller_simulation cartesian_controller_tests"

RUN apt-get -y update && apt install -y ros-humble-ros2-control

# Install utility scripts
# escape=`
RUN echo "export PATH=\$PATH:~/bin" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc


COPY scripts/starttmux.sh /root/bin/starttmux.sh

# Run tmux
CMD ["/root/bin/starttmux.sh"]



