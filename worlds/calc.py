center_x_webots = 0.2
center_y_webots = -0.2
size = 0.1
x_webots = center_x_webots - size/2
y_webots = center_y_webots + size/2
x_gimp = x_webots*100+150
y_gimp = 150-y_webots*100
print(x_gimp, y_gimp)