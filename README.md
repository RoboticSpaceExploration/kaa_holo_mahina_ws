# Ka'a holo mahina 🌙🚗
 ROS repository
 

# Setup
If you have **Ubuntu 18.04 with ROS melodic** installed you can 
simply clone and build this repository like any other ROS workspace:

```
git clone git@github.com:RoboticSpaceExploration/kaa_holo_mahina_ws.git
cd kaa_holo_mahina_ws

rosdep install --from-paths src --ignore-src -r -y

catkin build -j4
source devel/setup.bash
```

This may also work for other Ubuntu / ROS distributions as well.

# How to simulate the rover in gazebo
To run the open source rover on bedrock terrain run the following command:
```
roslaunch khm_gazebo bedrock.launch
```

You can modify the bedrock (or other physics and simulation) parameters in the worlds/bedrock.world file:

Dynamic friction: <mu>0.597</mu> 
Spring coefficient: <kp>7.53e7</kp>
Damping coefficient: <kd>8140</kd>

You can find more info here: [Gazebo physics parameter](https://gazebosim.org/tutorials?tut=physics_params&cat=physics)


# How to control the rover
You can publish control commands to the /cmd_vel topic, manually or [by using the keyboard](http://wiki.ros.org/teleop_twist_keyboard):

```
rostopic pub /cmd_vel
```
More info on the msg format here: [ROS Twist message](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html)

# How to record data
Some messages can be recorded through ROS, e.g. rover joint states, model states, D435 sensor, T265 sensor ...
Others have to be recorded through Gazebo, e.g. simulation parameters, physics, IMU, wheel wrench ...

To list all available ros topics you can run:
```
rostopic list
```

And all gazebo topics:
```
gz topic --list
```

You can also inspect a topic and message format with:
```
rostopic info <TOPIC_NAME>
gz topic --info <TOPIC_NAME>
```

[ROS - recording and playback](http://wiki.ros.org/rosbag/Tutorials/Recording%20and% 20playing%20back%20data)
[Gazebo - logging and playback](http://gazebosim.org/tutorials?tut=logging_playback)

For example, you can record all model states in the simulation to a rosbag named MODEL_STATES.bag by running:
```
rosbag record -O MODEL_STATES.bag /gazebo/model_states
```
Or get the collision of different bodies (wheel, rocker, terrain, ...) including forces and normals as a txt file:
```
gz topic --echo /gazebo/default/physics/contacts > contacts.txt
```

# How post-process the data

Depending on the messages you are recording, you will need to create your own script to parse them
into the desired format (txt, csv, ...). 

You may also want to use other open-soruce ressources as a starting point:
https://github.com/AtsushiSakai/rosbag_to_csv

[ROSbag to csv](https://github.com/AtsushiSakai/rosbag_to_csv)


