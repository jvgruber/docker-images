FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y \
  curl \
  fluid \
  git \
  nano \
  ~nros-foxy-rqt* \
  python3-argcomplete \
  python3-colcon-common-extensions \
  ros-dev-tools \
  sudo \
  terminator \
  wget \
  && rm -rf /var/lib/apt/lists/*

# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
#   > /etc/apt/sources.list.d/ros-latest.list' \
#   && wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
#   && apt-get update \
#   && apt-get install -y \
#   python3-catkin-tools \
#   && rm -rf /var/lib/apt/lists/*

# RUN apt-get update \
#   && apt-get install -y \
#   ros-dev-tools \
#   && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Create a non-root user, create a home directory and a .config directory as some programms expect this.
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && mkdir /home/${USERNAME}/.config \
  && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

USER ${USERNAME}
# everything here will be run as the ros user!

USER root
# everything here will be run as root!
# note that the last user will be opened upon startup
# Best Practice: end the dockerfile with root as user and select at the run command from which user you want to run the container
# docker run -it --user ros ...

RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0040 /etc/sudoers.d/${USERNAME}

COPY bashrc /home/${USERNAME}/.bashrc

#ENTRYPOINT: Write a bash script that acts as a wrapper at runtime (e.g. to source the workspace), you might want to put any complex dds configuration
# Any arguments provided will be executed with the entrypoint as if it was written inside the .sh file
COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

# we can use CMD to proived arguments, if we provide arguments upon run they will replace what was originally in the CMD array!
CMD ["bash"]