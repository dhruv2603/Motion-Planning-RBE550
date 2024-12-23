import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import argparse
import os, sys


def get_path_from_file(filepath):
### Function to read the path from the file and return the path
### The path is represented as a list of poses
### The first line of the file contains the configuration space and the size of the robot (if not a point robot)
### Input: filepath - path to the file containing the path
### Output: Configuration space, robot size (if not a point robot), List of poses
   robot_size = []
   if os.path.exists(filepath):
       with open(filepath, 'r') as f:
           lines = f.readlines()
           path = []
           info = lines[0].split()
           config_space = info[0]
           
           if config_space not in ['R2', 'SE2']:
               print("Invalid configuration space")
               sys.exit(0)
           
           if config_space == 'SE2':
               robot_size = float(info[1])


           for i in range(1, len(lines)):
               cspace_coords = lines[i].split()
               path.append([float(cspace_coords[i]) for i in range(len(cspace_coords))])


       path = list(filter(lambda x: x, path))
       return config_space, robot_size, path
   else:
       print(f"{filepath} file not found - Please provide the right path")
       sys.exit(0)


def animate_pend(path):
   # theta_values = []
   theta_values = [pair[0] for pair in path]
   
   # Pendulum parameters
   l = 1.0  # length of the pendulum
   r = 0.05  # radius of the bob


   # Setup the figure and axis
   fig, ax = plt.subplots()
   ax.set_aspect('equal')
   ax.set_xlim(-1.2 * l, 1.2 * l)
   ax.set_ylim(-1.2 * l, 1.2 * l)
   ax.set_title("Pendulum Motion Animation")


   # Initialize line and bob (circle) for the pendulum
   line, = ax.plot([], [], 'k-', lw=2)
   bob = Circle((0, 0), r, fc='b')
   ax.add_patch(bob)


   # Update function for the animation
   def update(frame):
       theta = theta_values[frame]
       x = l * np.sin(theta)  # x-coordinate of the bob
       y = -l * np.cos(theta) # y-coordinate of the bob


       # Update the line and bob position
       line.set_data([0, x], [0, y])
       bob.set_center((x, y))
       return line, bob


   # Create the animation
   ani = FuncAnimation(fig, update, frames=len(theta_values), blit=True, repeat=False)
   ani.save("pendulum_motion.gif", writer="imagemagick", fps=30)
   plt.show()


# Load the theta values from the provided file
def main():
   parser = argparse.ArgumentParser(description="Visualize pendulum trajectory in phase space.")
   parser.add_argument('--path', type=str, default='path.txt', help='Path file name containing the pendulum states')
   args = parser.parse_args()
   config_space, robot_size, path = get_path_from_file(args.path)
   # print("Loading pendulum path data from file:", args.path)
   # Plot and animate the path
   print(type(path))
   animate_pend(path)


if __name__ == "__main__":
   main()