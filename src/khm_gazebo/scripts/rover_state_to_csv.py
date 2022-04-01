
import rosbag, sys, csv
# import time
# import string
# import os 
# import shutil 
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion

if (len(sys.argv) != 2):
	print ("Wrong arguments\n Usage: python rover_state_to_csv.py <PATH_TO_BAGFILE>.bag")
	sys.exit(1)

filename = str(sys.argv[1])

print("Opening file:", filename)
bag = rosbag.Bag(filename, 'r')
csvfilename = filename[:-4]+'.csv'

with open(csvfilename, 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=' ', quotechar='|')
    writer.writerow(["timestamp_ns", "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw", \
                    "linear_vel_x", "linear_vel_y", "linear_vel_z", "angular_vel_x", "angular_vel_y", "angular_vel_z"])

    count = 0
    for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        if 'rover' in msg.name:
            rover_index = msg.name.index('rover')
            rover_pose = msg.pose[rover_index]
            rover_twist = msg.twist[rover_index]

            pos_x = rover_pose.position.x
            pos_y = rover_pose.position.y
            pos_z = rover_pose.position.z

            # Convert orientation quaternion to euler angles
            orientation_q = rover_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            lin_x = rover_twist.linear.x
            lin_y = rover_twist.linear.y
            lin_z = rover_twist.linear.z

            ang_x = rover_twist.angular.x
            ang_y = rover_twist.angular.y
            ang_z = rover_twist.angular.z
            writer.writerow([t, pos_x, pos_y, pos_z, roll, pitch, yaw, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z])
            count += 1

bag.close()
print('Saved '+str(count)+' rover states to '+csvfilename)