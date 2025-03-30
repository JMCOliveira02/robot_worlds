import math
import yaml
from PIL import Image, ImageDraw

yaml_file = "square_box_rectangle.yaml"
pgm_file = "square_box_rectangle.pgm"

# === Load YAML from string or file ===
with open("/home/joao/ros2_ws/src/robot_worlds/feature_maps/" + yaml_file, "r") as f:
    data = yaml.safe_load(f)

features = data["features"]

# === Image settings ===
img_size = 300
scale = 100  # 1 world unit = 100 pixels
origin = (img_size // 2, img_size // 2)

# === Helper: world to image coordinates ===
def to_img_coords(x, y):
    px = int(origin[0] + x * scale)
    py = int(origin[1] - y * scale)
    return px, py

# === Helper: rotate + translate keypoint ===
def transform_keypoint(kp, center, theta_deg):
    theta = math.radians(theta_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    x_rot = cos_t * kp['x'] - sin_t * kp['y']
    y_rot = sin_t * kp['x'] + cos_t * kp['y']
    return {
        'x': x_rot + center['x'],
        'y': y_rot + center['y']
    }

# === Create image ===
img = Image.new("L", (img_size, img_size), color=255)  # white background
draw = ImageDraw.Draw(img)

# === Process features ===
for feature in features:
    ftype = feature["type"]

    if ftype in ["square", "rectangle"]:
        center = feature["position"]
        theta = feature["orientation"]["theta"]
        keypoints = feature["keypoints"]

        transformed = [transform_keypoint(kp, center, theta) for kp in keypoints]
        polygon = [to_img_coords(kp["x"], kp["y"]) for kp in transformed]
        draw.polygon(polygon, fill=0)  # black shape

    elif ftype == "corner":
        x, y = feature["position"]["x"], feature["position"]["y"]
        px, py = to_img_coords(x, y)
        r = 3  # radius in pixels
        draw.ellipse((px - r, py - r, px + r, py + r), fill=0)  # black circle

# === Save as PGM ===
with open("/home/joao/ros2_ws/src/robot_worlds/maps/" + pgm_file, "wb") as f:
    f.write(bytearray(f'P5\n{img_size} {img_size}\n255\n', 'ascii'))
    f.write(img.tobytes())

print("Saved feature_map.pgm")
