# Ros2vis
```pythonimport numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import rclpy
from sensor_msgs.msg import LaserScan
from itertools import count

# Initialize ROS2 node
rclpy.init()
node = rclpy.create_node('lidar_visualizer')

# Create figure and 3D axis for plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set axis limits
ax.set_xlim(-45, 45)
ax.set_ylim(0, 100)  # This will now represent the most recent 100 frames
ax.set_zlim(0, 50)   # Adjust based on the range of your sensor
ax.set_xlabel('Horizontal Angle (degrees)')
ax.set_ylabel('Time (frames)')
ax.set_zlabel('Distance (meters)')
ax.set_title('3D Live Lidar Scan')

# Initialize empty data array
data = np.zeros((90, 100))  # Adjust the size based on the number of points in the [-45:0] and [0:45] range

# Subscriber callback function
def scan_callback(msg):
    global data
    # Convert msg.ranges to a numpy array and reshape it to match data's dimensions
    ranges = np.array(msg.ranges)
    ranges = np.clip(ranges, 0, 50)  # Clip distances to sensor's max range
    # Select only the points from the [-45:] and [0:45] range
    selected_ranges = np.concatenate((ranges[-45:], ranges[:46]))
    data = np.roll(data, -1, axis=1)
    data[:, -1] = selected_ranges

# Subscribe to LaserScan topic
node.create_subscription(LaserScan, '/scan', scan_callback, 10)

# Function to update the plot
def update(frame):
    # Clear the axis and plot the new data
    ax.cla()
    ax.set_xlim(-45, 45)
    ax.set_ylim(0, 100)  # This will now represent the most recent 100 frames
    ax.set_zlim(0, 50)   # Adjust based on the range of your sensor
    ax.set_xlabel('Horizontal Angle (degrees)')
    ax.set_ylabel('Time (frames)')
    ax.set_zlabel('Distance (meters)')
    ax.set_title('3D Live Lidar Scan at Frame: {}'.format(frame))
    
    X, Y = np.meshgrid(np.arange(-45, 46), np.arange(100))  # Adjust the range to match the selected points
    ax.plot_surface(X, Y, data.T, cmap='viridis')

    return fig,

# Create animation with an infinite loop
ani = FuncAnimation(fig, update, frames=count(), blit=False)

# Spin ROS2 node in a separate thread
import threading
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

plt.show()

# Shutdown ROS2 node
rclpy.shutdown()

# To record a ROS2 bag, run this command in a terminal:
# ros2 bag record -o my_lidar_data /scan

```
