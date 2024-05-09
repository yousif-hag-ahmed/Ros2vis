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
# Initialize an empty list to store the hole boundaries and depths
hole_boundaries = []
hole_data = []
# Initialize a flag for hole detection
detecting_hole = False

def update(frame):
    global data, detecting_hole
    # Drop the oldest frame and add a new one
    data = np.roll(data, -1, axis=1)
    
    # Calculate the derivative
    derivative_y = abs(np.diff(data, axis=1))
    derivative_x = abs(np.diff(data[:, -1]))
    
    # Find the indices where the derivative is above a certain threshold
    hole_indices_x = np.where(derivative_x > 0.1)[0]
    hole_indices_y = np.where(derivative_y[:, -1] > 0.1)[0]
    
    if hole_indices_x.size > 0:
        # Start detecting a hole
        detecting_hole = True
        # Store the start and end indices of the hole
        hole_boundaries.append((frame, hole_indices_x[0], hole_indices_x[-1]))
        # Calculate the maximum depth of the hole
        hole_depth = np.min(data[-1, hole_indices_x])
        print("Hole maximum depth: ", hole_depth)
        # Store the hole data
        hole_data.append((frame, hole_indices_x[0], hole_indices_x[-1], hole_depth))
        print("Hole data: ", hole_data)
    elif detecting_hole and np.max(derivative_x) < 0.1:
        # If the hole has ended, plot the hole boundaries
        detecting_hole = False
        plt.figure()
        for (time_step, start, end) in hole_boundaries:
            plt.plot([start, end], [time_step, time_step], 'ro-')
        plt.title('Hole boundaries over time')
        plt.xlabel('Index')
        plt.ylabel('Time step')
        plt.show()
        # Reset the hole boundaries
        hole_boundaries.clear()

    # Clear the axis and plot the new data
    ax.cla()
    ax.set_xlim(-45, 45)
    ax.set_ylim(0, 100)
    ax.set_zlim(0, 50)
    ax.set_xlabel('Horizontal Angle (degrees)')
    ax.set_ylabel('Time (frames)')
    ax.set_zlabel('Distance (meters)')
    ax.set_title('3D Live Lidar Scan at Time: {}'.format(frame))
    
    X, Y = np.meshgrid(np.arange(-45, 46), np.arange(100))
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
