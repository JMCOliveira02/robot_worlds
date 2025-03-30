import math

# Center
cx, cy = 0, 0
x_length = 1
y_length = 1
rotation_theta = 45

# Half-sizes
hx = x_length / 2
hy = y_length / 2

# Convert angle to radians
theta = math.radians(rotation_theta)

# Precompute cos and sin
cos_t = math.cos(theta)
sin_t = math.sin(theta)

# Local corners relative to center
local_corners = [
    (-hx, -hy),  # bottom-left
    ( hx, -hy),  # bottom-right
    ( hx,  hy),  # top-right
    (-hx,  hy)   # top-left
]

# Rotate and translate each corner
world_corners = []
for lx, ly in local_corners:
    wx = cx + (lx * cos_t - ly * sin_t)
    wy = cy + (lx * sin_t + ly * cos_t)
    world_corners.append((wx, wy))

# Print result
for i, (x, y) in enumerate(world_corners):
    print(f"Corner {i+1}: ({x:.3f}, {y:.3f})")
