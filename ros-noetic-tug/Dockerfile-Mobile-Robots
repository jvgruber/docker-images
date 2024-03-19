FROM osrf/ros:noetic-desktop-full

# installing necessary programs
# use apt-get instead of apt as it is more stable and we don't need the userfriendlyness of apt in an automated use-case.
# update apt-get for every run command as docker only reruns the changed run commands
# delete the package lists at the end of run command, this saves size and prevents not running update for a new run
# put everything into one run command to save space and prevent issues with the docker cache system

# set the environment variable "DEBIAN_FRONTEND=noninteractive" inside run command to prevent forgetting turning it on again
# this variable prevents other users from prompting while building, but it can be ommited.
ENV DEBIAN_FRONTEND=noninteractive

# Best Practice: break your lines for easier version control! (Also sort packages alphabetically)
RUN apt-get update \
  && apt-get install -y \
  curl \
  fluid \
  git \
  nano \
  ros-noetic-csm \
  ros-noetic-kobuki-msgs \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-simulations \
  sudo \
  terminator \
  wget \
  && rm -rf /var/lib/apt/lists/*

# Installations dependent on previous installs
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
  > /etc/apt/sources.list.d/ros-latest.list' \
  && wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
  && apt-get update \
  && apt-get install -y \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# copy static directories to the image
# COPY config/ /site_config/

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
# TIPP: you don't need sudo as you can always run docker exec to create a new terminal with root access

COPY bashrc /home/${USERNAME}/.bashrc

#ENTRYPOINT: Write a bash script that acts as a wrapper at runtime (e.g. to source the workspace), you might want to put any complex dds configuration
# Any arguments provided will be executed with the entrypoint as if it was written inside the .sh file
COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

# we can use CMD to proived arguments, if we provide arguments upon run they will replace what was originally in the CMD array!
CMD ["bash"]