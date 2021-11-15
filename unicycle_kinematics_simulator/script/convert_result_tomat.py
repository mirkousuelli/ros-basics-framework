import sys
import rosbag
import scipy.io as sio

# Read data from bag (raw data)
bag = rosbag.Bag(sys.argv[1])

# State published by the controller
time = []
x1 = []
x2 = []

for topic, msg, t in bag.read_messages():
    if topic == "/output":
        time.append(msg.data[0])
        x1.append(msg.data[1])
        x2.append(msg.data[2])

bag.close()

# Save data on mat file
filename = sys.argv[1]
sio.savemat(filename[:len(filename)-3]+'mat', {'t': time, 'x1': x1, 'x2': x2})

