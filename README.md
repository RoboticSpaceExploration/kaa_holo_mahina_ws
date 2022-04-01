# Ka'a holo mahina 🌙🚗


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

This may also work on other Ubuntu / ROS distributions as well.

# How to simulate the rover in gazebo
To run the open source rover on bedrock terrain run the following command:
```
roslaunch khm_gazebo bedrock.launch
```

This will also launch an **RViz** instance, that displays the transformation tree between each link / component.

You can modify the bedrock (or other physics and simulation) parameters in the `worlds/bedrock.world` file:

Dynamic friction: `<mu>0.597</mu> `

Spring coefficient: `<kp>7.53e7</kp>`

Damping coefficient: `<kd>8140</kd>`


You can find more info here: 

[Gazebo physics parameter](https://gazebosim.org/tutorials?tut=physics_params&cat=physics)


# How to control the rover
You can publish control commands to the /cmd_vel topic [by using the keyboard](http://wiki.ros.org/teleop_twist_keyboard) or manually:

```
rostopic pub /cmd_vel
```
More info on the msg format here: 

# How to record data
Some messages can be recorded through ROS, e.g. rover joint states, model states, D435 sensor, T265 sensor ...
Others have to be recorded through Gazebo, e.g. simulation parameters, physics, IMU, contact forces ...

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


[ROS - recording and playback](http://wiki.ros.org/action/fullsearch/ROS/Tutorials/Recording%20and%20playing%20back%20data?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FRecording+and+playing+back+data%22)

[Gazebo - logging and playback](http://gazebosim.org/tutorials?tut=logging_playback)


For example, you can record all model states in the simulation to a rosbag named MODEL_STATES.bag by running:

```
rosbag record -O MODEL_STATES.bag /gazebo/model_states
```

Or get the collision of different bodies (wheel, rocker, terrain, ...) including forces and normals as a txt file:

```
gz topic --echo /gazebo/default/physics/contacts > contacts.txt
```

# How to post-process the data

Depending on the messages you are recording, you will need to create your own script to parse them
into the desired format (txt, csv, ...). 

One example to convert rover states into .csv format can be found in `/src/khm_gazebo/scripts`:

```
python rover_state_to_csv.py MODEL_STATES.bag
```

The output file `MODEL_STATE.csv` will have the format:

[timestamp (nanoseconds), 3D rover pose (meters), roll, pitch, yaw (radians), 3D linear velocity (m/s), 3D angular velocity (rad/s)] 

You may need to install other dependencies for the rosbag converter:

```
pip3 install pycryptodomex python-gnupg
```

Other open-source ressources you can use as a starting point:

[rosbag cookbook API](http://wiki.ros.org/rosbag/Cookbook)

[rosbag to csv](https://github.com/AtsushiSakai/rosbag_to_csv)


# How to modify the rover (e.g. add sensors)

The [URDF file](http://wiki.ros.org/urdf) that describes the rover can be found in `src/khm_description/urdf/khm_description.urdf`
**Visuals and collision meshes** can be found in `src/khm_description/meshes`.

The rover consists of **1 main body, 2 rockers** (left/right) and **2 bogies** (left/right).
Both rockers are connected to the body through continuous joints (freely rotating around the joint axis).
The bogies are at the front of the rover and are connected to the rockers also through continuous joints, which enables the rocker-bogie mechanism.

There are **6 wheels** in total (left/right side each with front, middle and back).
The front and back wheels are attached to the rover through **corners**, that can freely rotate around the wheel's z-axis,
which enables steering. The middle wheels are attached to the rockers and do not rotate around their z-axes.


You may modify joint limits, link positions or sensor by changing the URDF, e.g.
you can comment out computing-intensive sensors (T265, D435 sensors) to boost simulation performance.



Example ROS sensors (`/topic_prefix`):


T265 depth camera: `/camera`

D435 depth camera:  `/d435`

Rover components/states `/rover_ns`

Example Gazebo sensors:

Left bogie IMU: `/gazebo/default/rover/bogie_left/imu/imu`

Right rocker IMU: `/gazebo/default/rover/rocker_right/imu/imu`

More info here:

[URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf)

[Add sensors to a robot in Gazebo](http://gazebosim.org/tutorials/?tut=add_laser)

[Build a robot in Gazebo](http://gazebosim.org/tutorials?cat=build_robot)


