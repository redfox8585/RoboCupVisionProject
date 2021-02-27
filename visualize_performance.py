import sys
import matplotlib.pyplot as plt
import json
import numpy as np
import random
import math

ball_positions_json = sys.argv[1]
ball_positions_contours_json = sys.argv[2]

with open(ball_positions_json) as json_file:
    ball_positions = json.load(json_file)

with open(ball_positions_contours_json) as json_file:
    ball_positions_contours = json.load(json_file)

calculated_ball_positions_contours = ball_positions_contours["measuredBallPositions"]
calculated_ball_positions = ball_positions["measuredBallPositions"]
real_ball_positions = ball_positions["realBallPositions"]
timestamps = ball_positions["timestamps"]
x_real = []
y_real = []

x_calculated = []
y_calculated  = []

x_calculated_contours = []
y_calculated_contours  = []

distance = []
distance_contours = []

for i in range(len(real_ball_positions)):
    current_real_ball_position = real_ball_positions[i]
    current_calculated_ball_position = calculated_ball_positions[i]
    current_calculated_ball_position_contours = calculated_ball_positions_contours[i]

    x_real.append(current_real_ball_position["x"])
    y_real.append(current_real_ball_position["y"]) 
    x_calculated.append(current_calculated_ball_position["x"])
    y_calculated.append(current_calculated_ball_position["y"])

    x_calculated_contours.append(current_calculated_ball_position_contours["x"])
    y_calculated_contours.append(current_calculated_ball_position_contours["y"])

    # calculate distance between real and calculated ball position
    x_diff = current_real_ball_position["x"] - current_calculated_ball_position["x"]
    y_diff = current_real_ball_position["y"] - current_calculated_ball_position["y"]

    x_diff_contours = current_real_ball_position["x"] - current_calculated_ball_position_contours["x"]
    y_diff_contours = current_real_ball_position["y"] - current_calculated_ball_position_contours["y"]

    current_distance = math.sqrt(x_diff**2 + y_diff**2)
    current_distance_contours = math.sqrt(x_diff_contours**2 + y_diff_contours**2)

    distance.append(current_distance)
    distance_contours.append(current_distance_contours)

print("with HoughCircles\n")
print("Distance between real and calculated ball position : ",  round(np.mean(distance),4))
print("Maximum Distance between real and calculated ball position : ",  round(max(distance),4))
print("Minimum Distance between real and calculated ball position : ",  round(min(distance),4))
print("Median : ",  np.median(distance))

print("\nwith findContours\n")
print("Distance between real and calculated ball position : ",  round(np.mean(distance_contours),4))
print("Maximum Distance between real and calculated ball position : ",  round(max(distance_contours),4))
print("Minimum Distance between real and calculated ball position : ",  round(min(distance_contours),4))
print("Median : ",  np.median(distance_contours))

plt.title('Ball position on field')
plt.axis([-5, 3, -1.5, 2.5])
plt.plot(x_real,y_real, 'b', linewidth=2, label="real position")
plt.plot(x_calculated,y_calculated, 'ro', markersize=0.5, label="calculated position")
plt.ylabel('y')
plt.xlabel('x')
plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
plt.savefig('measured_and_real.png') 

plt.clf()

plt.title('Ball position on field')
plt.axis([-5, 3, -1.5, 2.5])
plt.plot(x_real,y_real, 'b', linewidth=2, label="real position")
plt.plot(x_calculated_contours,y_calculated_contours, 'yo', markersize=0.5, label="calculated position")
plt.ylabel('y')
plt.xlabel('x')
plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
plt.savefig('measured_and_real_findContours.png') 

plt.clf()

plt.title('Distance between calculated and real ball position')
plt.axis([0, 40, 0, 0.6])
plt.plot(timestamps, distance , 'r', linewidth=1, label="HoughCircles")
plt.plot(timestamps, distance_contours , 'y', linewidth=1, label="FindContours")
plt.xlabel('t')
plt.ylabel('distance')
plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
plt.savefig('distance.png') 






