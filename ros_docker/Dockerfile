FROM ros:melodic-ros-base

ENV ROS_WORKSPACE=/catkin_ws

COPY ./ros_docker/set-up-workspace /setup.sh

# Trabajaremos con una version en especifica de este paquete (v0.6.0)
# La podemos tener ya descargada o se puede obtener desde su repositorio con el tag "v0.6.0"	
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b v0.6.0
RUN chmod +x /setup.sh && /setup.sh && rm /setup.sh

WORKDIR $ROS_WORKSPACE

# Source the workspace on sign in
RUN echo ". devel/setup.bash" >> ~/.bashrc

# making sure the file modes are executable
RUN chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py


