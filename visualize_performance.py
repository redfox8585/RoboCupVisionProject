import sys
import matplotlib.pyplot as plt
import json
import numpy as np
import random
import math

real_positions_json = sys.argv[1]

with open(real_positions_json) as json_file:
    real_ball_positions = json.load(json_file)["positions"]

#TODO read calculated ball positions from file
calculated_ball_positions = real_ball_positions

x_real = []
y_real = []
t_real = []

x_calculated = []
y_calculated  = []

distance = []

for i in range(len(real_ball_positions)):
    current_real_ball_position = real_ball_positions[i]
    current_calculated_ball_position = calculated_ball_positions[i]

    x_real.append(current_real_ball_position["x"])
    y_real.append(current_real_ball_position["y"])
    t_real.append(current_real_ball_position["t"])

    # TODO this is just for demo
    x_diff = random.randint(-10, 10) / 100.0
    y_diff = random.randint(-10, 10) / 100.0
    cX = current_calculated_ball_position["x"] + current_calculated_ball_position["x"] * x_diff
    cY = current_calculated_ball_position["y"]  + current_calculated_ball_position["y"] * y_diff

    x_calculated.append(cX)
    y_calculated.append(cY)

    # calculate distance between real and calculated ball position
    x_diff = current_real_ball_position["x"] - cX
    y_diff = current_real_ball_position["y"] - cY
    current_distance = math.sqrt(x_diff**2 + y_diff**2)

    distance.append(current_distance)

print("Distance between real and calculated ball position : ",  round(np.mean(distance),4))
print("Maximum Distance between real and calculated ball position : ",  round(max(distance),4))
print("Minimum Distance between real and calculated ball position : ",  round(min(distance),4))

plt.title('Ball position on field')
plt.axis([-5, 5, -3, 3])
plt.plot(x_calculated,y_calculated, 'bo', markersize=1, label="calculated position")
plt.plot(x_real,y_real, 'r', linewidth=2, label="real position")
plt.ylabel('y')
plt.xlabel('x')
leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)


# plt.title('Distance between calculated and real ball position')
# plt.axis([0, 40, 0, 1])
# plt.plot(t_real, distance , 'r', linewidth=1)
# plt.xlabel('t')
# plt.ylabel('distance')



plt.show()

