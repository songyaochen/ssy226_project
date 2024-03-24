import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
import cv2

# Is not included in the report atm! 

filePath = "Documentation/Resources/Bagfiles/dist_wall.bag"
filePath = "Documentation/Resources/Bagfiles/dist_wall2.bag"

bag = rosbag.Bag(filePath)

images_arr = []
for topic, msg, t in bag.read_messages():
    images_arr.append(cv_bridge.CvBridge().imgmsg_to_cv2(msg,desired_encoding="32FC1"))
    #avg_img += cv_bridge.CvBridge().imgmsg_to_cv2(msg,desired_encoding="32FC1")

images_arr = np.array(images_arr)
avg_img = np.nanmean(images_arr, axis=0)

# cv2.imshow('test',avg_img)
# cv2.waitKey(0)

# print(avg_img.shape)
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
X = np.arange(0, 640, 1)
Y = np.arange(0, 480, 1)
X,Y = np.meshgrid(X,Y)
Z = avg_img
surf = ax.plot_surface(X, Y, Z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Distance')
ax.set_zlim([0.5,0.75])
plt.title('Surface plot of depth sensor')

ax.view_init(elev=20, azim=45)

plt.savefig('Documentation/Diagrams/wall_test')

fig, axs = plt.subplots(3, 3)

section1 = (images_arr[:,0:120,0:213])
axs[2,0].hist(np.nanmean(section1, axis=(1,2)), edgecolor='black')
axs[2,0].set_xticks([])
axs[2,0].yaxis.set_tick_params(labelleft=False)

section2 = (images_arr[:,120:120+120,0:213])
axs[2,1].hist(np.nanmean(section2, axis=(1,2)), edgecolor='black')

section3 = (images_arr[:,120+120:,0:213])
axs[2,2].hist(np.nanmean(section3, axis=(1,2)), edgecolor='black')

section4 = (images_arr[:,0:120,213:213+213])
axs[1,0].hist(np.nanmean(section4, axis=(1,2)), edgecolor='black')

section5 = (images_arr[:,120:120+120,213:213+213])
axs[1,1].hist(np.nanmean(section5, axis=(1,2)), edgecolor='black')

section6 = (images_arr[:,120+120:,213:213+213])
axs[1,2].hist(np.nanmean(section6, axis=(1,2)), edgecolor='black')

section7 = (images_arr[:,0:120,213+213:])
axs[0,0].hist(np.nanmean(section7, axis=(1,2)), edgecolor='black')

section8 = (images_arr[:,120:120+120,213+213:])
axs[0,1].hist(np.nanmean(section8, axis=(1,2)), edgecolor='black')

section9 = (images_arr[:,120+120:,213+213:])
axs[0,2].hist(np.nanmean(section9, axis=(1,2)), edgecolor='black')

plt.show()

# Todo: Add variance calculations and plotting

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
            
# extract_topics_and_types(filePath)
