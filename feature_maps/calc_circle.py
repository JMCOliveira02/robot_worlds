
from math import cos, sin
radius = 0.3
center_x = -1.0
center_y = 0.0
num_points = 36
angle_increment = 2 * 3.14159 / num_points
for i in range(num_points):
    angle = i * angle_increment
    x = radius * cos(angle) + center_x
    y = radius * sin(angle) + center_y
    print(f"Circle point {i}: ({x}, {y})")