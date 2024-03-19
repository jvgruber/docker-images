# How to use for Advanced Robotics Practicals

1. Create the file structure inside the `volume` directory:
```bash
cd volume
mkdir -p catkin_ws/src
```

2. Build the image and run a container:
```bash
\bin\bash build
\bin\bash run
```

3. When starting the container for the first time it will tell you that `/vol/catkin_ws/devel/setup.bash: No such file or directory`. This is because the necessary source command is already in the bashrc. So no worries. From now on we work inside the container

4. Create, build and source the workspace:
```bash
cd /vol/catkin_ws
catkin init
catkin_make
source devel/setup.bash
catkin config -DCMAKE_BUILD_TYPE=Release
```

5. Setup the AR Repo as described in the setup_guide.pdf.
> **NOTE:** `rosdep install --from-paths src --ignore-src -r -y`
and `sudo bash install` probably needs to be rerun every time you run a new container! So better start and stop the same container.