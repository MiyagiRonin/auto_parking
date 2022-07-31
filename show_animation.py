import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pch
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

#vehicle size
HALF_WIDTH = 1.0
REAR_TO_RAC = 1.0
FRONT_TO_RAC = 3.7

env_file = open("problems/reverse_verticle_1.txt", 'r')
path_file = open("bazel-bin/output_path.txt", 'r')

def draw_vehicle_box(rac_x, rac_y, heading, front_to_rac, rear_to_rac, half_width):
   x = np.array([[rac_x], [rac_y]])
   sin = math.sin(heading)
   cos = math.cos(heading)
   matrix = np.matrix([[cos, -sin],[sin, cos]])
   left_rear =  matrix*np.array([[-rear_to_rac], [half_width]]) + x
   right_rear = matrix*np.array([[-rear_to_rac], [-half_width]])  + x
   right_front = matrix*np.array([[front_to_rac], [-half_width]])  + x
   left_front = matrix*np.array([[front_to_rac], [half_width]])  + x
   xy = np.vstack((np.transpose(left_rear), np.transpose(right_rear), 
   np.transpose(right_front), np.transpose(left_front)))
   return pch.Polygon(xy, True)

fig, ax =plt.subplots(figsize=(6,6))
#load environment
env = []
line = env_file.readline()
while line:
   line = env_file.readline().split()
   if len(line) == 0:
      break
   info = [float(x) for x in line]
   env.append(Line2D([info[0], info[2]], [info[1], info[3]]))
#load path
path = []
xs = []
ys = []
line = path_file.readline()
while line:
   line = path_file.readline().split()
   if len(line) == 0:
      break
   info = [float(x) for x in line]
   xs.append(info[0])
   ys.append(info[1])
   path.append(draw_vehicle_box(info[0], info[1], info[2], FRONT_TO_RAC, REAR_TO_RAC, HALF_WIDTH))


def update(i):
   ax.clear()
   ax.set_facecolor(plt.cm.Blues(.2))
   ax.set_xlim([-10,10])
   ax.set_ylim([-2,15])
   ax.set_aspect(1)
   ax.set_title('Parking Simulation')
   ax.add_patch(path[i])
   for line_2d in env:
      ax.add_line(line_2d)
   ax.plot(xs[:i], ys[:i], color="k")
   #[spine.set_visible(False) for spine in ax.spines.values()]

anime = FuncAnimation(
   fig = fig,
   func = update,
   frames = len(path),
   interval = 50
)

plt.show()
#anime.save('result.gif')