import yaml
import matplotlib.pyplot as plt

# Load YAML data from file
def load_yaml(filename):
    with open(filename, 'r') as file:
        return yaml.safe_load(file)

# Extract and plot circle keypoints
def plot_features(data):
    for feature in data['features']:
        if feature['type'] == 'circle':
            keypoints = feature['keypoints']
            x_vals = [point['x'] for point in keypoints]
            y_vals = [point['y'] for point in keypoints]
            plt.figure(figsize=(6, 6))
            plt.plot(x_vals, y_vals, 'o', label='Circle Keypoints')
            plt.scatter([feature['position']['x']], [feature['position']['y']], c='red', label='Circle Center')
            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid(True)
            plt.legend()
            plt.title('Circle Feature Keypoints')
            plt.xlabel('x')
            plt.ylabel('y')
            plt.show()

if __name__ == '__main__':
    filename = '/home/joao/ros2_ws/src/robot_worlds/feature_maps/4x4_two_cylinders_pt3.yaml'  # Replace with the path to your YAML file
    data = load_yaml(filename)
    plot_features(data)
