import json
import matplotlib.pyplot as plt

# Load the route exported from C++
with open("../output/route_output.json", "r") as f:
    data = json.load(f)

start = data["start"]
dest = data["destination"]
path = data["path"]

x_coords = [node["x"] for node in path]
y_coords = [node["y"] for node in path]

# Insert start at the beginning if not explicitly included
if not x_coords or x_coords[0] != start["x"]:
    x_coords.insert(0, start["x"])
    y_coords.insert(0, start["y"])

plt.figure(figsize=(10, 6))
plt.plot(x_coords, y_coords, marker='o', color='b', linestyle='-', linewidth=2, label='Theta* Route')
plt.scatter([start["x"]], [start["y"]], color='g', s=100, zorder=5, label='Start')
plt.scatter([dest["x"]], [dest["y"]], color='r', s=100, zorder=5, label='Destination')

plt.title("Sailing Engine Pathfinding Debugger")
plt.xlabel("X Grid Coordinate")
plt.ylabel("Y Grid Coordinate")
plt.gca().invert_yaxis() # Match grid matrix orientation (0,0 at top-left)
plt.legend()
plt.grid(True)
plt.show()