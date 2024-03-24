import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
import cv2


filePath = "Documentation/Resources/Bagfiles/static_Oscar.bag"

bag = rosbag.Bag(filePath)

depth_array = np.empty(0)

for topic, msg, t in bag.read_messages():
      for detection in msg.detections:
            if detection.results[0].id == 0:
                     depth_array = np.append(depth_array, detection.depth)

plt.hist(depth_array, bins=30, edgecolor='black')
print('The mean distance is: ', np.mean(depth_array))
print('The std is: ', np.std(depth_array))

plt.title('Distribution of depth measurements to identified person')
plt.xlabel('Distance (m)')
plt.ylabel('Occurences')

plt.savefig('Documentation/Diagrams/distribution_depth_measurement_to_identified_person')

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

# Replace 'your_bag_file.bag' with the actual path to your bag file
#bag_file_path = 'your_bag_file.bag'
#extract_topics_and_types(bag_file_path)
            
#print(extract_topics_and_types(filePath))
