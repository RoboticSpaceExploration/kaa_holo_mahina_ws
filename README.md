# Ka'a holo mahina 🌙🚗
 ROS repository
 

# Setup
If you have **Ubuntu 20.04 with ROS noetic** installed you can 
simply clone and build this repository like any other ROS workspace:

```
git clone git@github.com:RoboticSpaceExploration/kaa_holo_mahina_ws.git
cd kaa_holo_mahina_ws

rosdep install --from-paths src --ignore-src -r -y

catkin build -j4
source devel/setup.bash
```

This also works for Ubuntu 18.04 (ROS melodic) and may work for other distributions as well.

### What if I don't have Ubuntu or ROS?
You can [get Docker](https://docs.docker.com/get-docker/) and run the entire workspace in a container. If your computer is logged into your [DockerHub](https://hub.docker.com/) account you can simply run:

```
docker run -it hoangln1/roselab_sim:latest bash
```

Otherwise you can build the docker image locally (from scratch) and then run the container:
```
git clone git@github.com:RoboticSpaceExploration/kaa_holo_mahina.git
cd kaa_holo_mahina

docker build . -t hoangln1/roselab_sim:latest
docker run -it hoangln1/roselab_sim:latest bash
```


If you want to use any GUI application (rviz, gazebo), you need to expose your computer's display to the container and run this command instead:

#### Linux
```
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix hoangln1/roselab_sim:latest bash
```

#### Mac
[Follow **step 1 - 5** from here](https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285). Then you can run:
```
docker run -it -e DISPLAY=${HOSTNAME}:0 -v /tmp/.X11-unix:/tmp/.X11-unix hoangln1/roselab_sim:latest bash
```

All docker commands above should open a virtual environment on your local terminal that behaves exactly
like Ubuntu 20.04 with this ROS workspace already installed.

# How to simulate the rover in gazebo
To run the open source rover in a mars-like environment run the following command:
```
roslaunch khm_gazebo mars_with_rover.launch
```