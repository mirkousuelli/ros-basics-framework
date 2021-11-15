import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
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

# Plot data
plt.figure(1)
plt.plot(time,x1)
plt.xlabel("Time [s]")
plt.ylabel("Position [rad]")

plt.figure(2)
plt.plot(time,x2)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [rad/s]")

plt.show(block=False)

raw_input("Press Enter to End")
