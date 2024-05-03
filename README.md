# Ros2vis
```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import rclpy
from sensor_msgs.msg import LaserScan

# Initialize ROS2 node
rclpy.init()
node = rclpy.create_node('lidar_visualizer')

# Create figure and 3D axis for plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set axis limits
ax.set_xlim(-45, 45)
ax.set_ylim(0, 100)  # Adjust based on expected number of frames
ax.set_zlim(0, 50)   # Adjust based on the range of your sensor
ax.set_xlabel('Horizontal Angle (degrees)')
ax.set_ylabel('Time (frames)')
ax.set_zlabel('Distance (meters)')
ax.set_title('3D Live Lidar Scan')

# Initialize empty data array
data = np.zeros((360, 100))  # Adjust 360 to the number of points in msg.ranges

# Subscriber callback function
def scan_callback(msg):
    global data
    # Convert msg.ranges to a numpy array and reshape it to match data's dimensions
    ranges = np.array(msg.ranges)
    ranges = np.clip(ranges, 0, 50)  # Clip distances to sensor's max range
    data = np.roll(data, -1, axis=1)
    data[:, -1] = ranges

# Subscribe to LaserScan topic
node.create_subscription(LaserScan, '/scan', scan_callback, 10)

# Function to update the plot
def update(frame):
    # Clear the axis and plot the new data
    ax.cla()
    ax.set_xlim(-45, 45)
    ax.set_ylim(0, 100)  # Adjust based on expected number of frames
    ax.set_zlim(0, 50)   # Adjust based on the range of your sensor
    ax.set_xlabel('Horizontal Angle (degrees)')
    ax.set_ylabel('Time (frames)')
    ax.set_zlabel('Distance (meters)')
    ax.set_title('3D Live Lidar Scan at Frame: {}'.format(frame))
    
    X, Y = np.meshgrid(np.arange(360), np.arange(100))  # Adjust 360 to the number of points in msg.ranges
    ax.plot_surface(X, Y, data.T, cmap='viridis')

    return fig,

# Create animation
ani = FuncAnimation(fig, update, frames=np.arange(100), blit=False)  # Adjust frame range as needed

# Spin ROS2 node in a separate thread
import threading
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

plt.show()

# Shutdown ROS2 node
rclpy.shutdown()
```
