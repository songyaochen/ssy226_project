import rosbag
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import cv_bridge
import cv2
#filePath = "/home/hasan/Documents/ssy226_5_navi_perception/Documentation/Resources/Bagfiles/static_Oscar.bag"
#filePath = "/home/hasan/Documents/ssy226_5_navi_perception/Documentation/Resources/Bagfiles/dist_wall.bag"
filePath = "/home/hasan/Documents/ssy226_5_navi_perception/Documentation/Resources/Bagfiles/break_test1.bag"
bag = rosbag.Bag(filePath)
i = 0
msg_array = []
topic_array = []
img_array = []
linear_vel_vec = []
time_vec = []
for topic ,msg ,t in bag.read_messages():
   if topic == "/mobile_base_controller/cmd_vel":
          linear_vel_vec.append(msg.linear.x)
          time_vec.append(t)

#plt.plot(time_vec,linear_vel_vec)
#plt.show()
#info = bag.get_type_and_topic_info()
#print(info)
   #dist.append(msg.detections[0].depth)
   #img_array.append(cv_bridge.CvBridge().imgmsg_to_cv2(msg,desired_encoding="32FC1"))

#img_array_np = np.array(img_array)
#average_image = np.mean(img_array_np, axis=0)

#cv2.imshow('Average Image', average_image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
#X = np.arange(0, 640, 1)
#Y = np.arange(0, 480, 1)
#X,Y = np.meshgrid(X,Y)
#Z = average_image
#surf = ax.plot_surface(X, Y, Z)
#ax.set_xlabel('X')
#ax.set_ylabel('Y')
#ax.set_zlabel('Distance')
#plt.title('Surface plot of depth sensor')
#ax.title('Surface plot of depth sensor')
#plt.show()
#ax = plt.axes(projection="3d")

#z = np.array()

#y = np.arange(len(z))
#x = np.arange(len(z[0]))

#x ,y) = np.meshgrid(x,y)

#ax.plot_surface(x,y,z)
#plt.show()
  # print(t)
    
#plt.hist(dist,color="coral",bins=30)
#plt.xlabel("Distance in m")
#plt.ylabel("Number of measurment")
#plt.title("Histogram of distance measurment")
#plt.show()
#bag.close()

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

def main():
   filePath = "/home/hasan/Documents/ssy226_5_navi_perception/Documentation/Resources/Bagfiles/dist_wall.bag"
   extract_topics_and_types(filePath)
#main()