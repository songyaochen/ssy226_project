import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
import cv2
import pandas as pd

filePath1 = 'Documentation/Resources/Bagfiles/break_test1.bag'

bag = rosbag.Bag(filePath1)


cmd_vel_array = np.empty(0)
nav_vel_array = np.empty(0)
cmd_depth_array = np.empty(0)
nav_depth_array = np.empty(0)
cmd_time_array = np.empty(0)
nav_time_array = np.empty(0)
depth_array = np.empty(0)
depth_time_array = np.empty(0)
T = np.empty(0)

depth = None

for topic, msg, t in bag.read_messages():
   T = np.append(T, t.to_sec()) # Only wanted to save first value, did not care to implement logic
   if topic == "/mobile_base_controller/cmd_vel" and depth != None:
         cmd_vel_array = np.append(cmd_vel_array, msg.linear.x)
         cmd_depth_array = np.append(cmd_depth_array, depth)
         cmd_time_array = np.append(cmd_time_array, t.to_sec())
   elif topic == "/nav_vel"  and depth != None:
         nav_vel_array = np.append(nav_vel_array, msg.linear.x)
         nav_depth_array = np.append(nav_depth_array, depth)
         nav_time_array = np.append(nav_time_array, t.to_sec())
   elif topic == "/detection_result" :
         for detection in msg.detections:
               if detection.results[0].id == 0:
                      depth = detection.depth
                      depth_array = np.append(depth_array, depth)
                      depth_time_array = np.append(depth_time_array, t.to_sec())

t_0 = T[0]
cmd_time_array = cmd_time_array[3:] - t_0
nav_time_array = nav_time_array[3:] - t_0
depth_time_array = depth_time_array - t_0
cmd_vel_array = cmd_vel_array[3:]
nav_vel_array = nav_vel_array[3:]


plt.plot(cmd_time_array, cmd_vel_array, marker='x', markevery=5, label='Decision making')
plt.plot(nav_time_array, nav_vel_array, marker = '+', markevery = 5, label='Navigation')
plt.legend()
plt.title('Navigation and decision making reference velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

plt.savefig('Documentation/Diagrams/break_test_velocity')


plt.figure()
plt.plot(depth_time_array, depth_array)
plt.xlim([0,23])
plt.title('Distance to identified person')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')

plt.savefig('Documentation/Diagrams/break_test_distance')

plt.show()


def extract_topics_and_types(bag_file_path):
    with rosbag.Bag(bag_file_path, 'r') as bag:
        # Get information about topics and message types
        info = bag.get_type_and_topic_info()

        # Extract topics and message types
        topics_and_types = info.topics

        # Display the results
        for topic, topic_info in topics_and_types.items():
            message_type = topic_info.msg_type
            print(f"Topic: {topic}, Message Type: {message_type}")



