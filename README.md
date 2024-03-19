# My custom docker images
This is a collection of easy to use docker images I use for my personal projects. With the instructions below they should work out of the box on ubuntu 22.04 (possibly also other versions).

## How to use:
1. download the folder with the docker image you want to run
2. inside the folder create a directory `volume/dev_ws`[^1]
3. Build your image by executing the `build` script
4. Run a container by executing the `run` script
5. Place your ros workspace inside or build a new one (e.g. with `colcon build --symlink-install`)

You're ready to go!

[^1] for the `ros-noetic-tug` image please use `catkin_ws` instead.
