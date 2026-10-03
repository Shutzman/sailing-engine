import json
import math
import matplotlib.pyplot as plt

# 1. Load the route exported from C++
with open("route_output.json", "r") as f:
    data = json.load(f)

start = data["start"]
dest = data["destination"]
path = data["path"]
land = data.get("land", [])
wind_field = data.get("wind_field", [])
grid_w = data.get("width", 150)
grid_h = data.get("height", 150)

raw_x = [node["x"] for node in path]
raw_y = [node["y"] for node in path]

# Insert start at the beginning if not explicitly included
if not raw_x or raw_x[0] != start["x"]:
    raw_x.insert(0, start["x"])
    raw_y.insert(0, start["y"])

# 2. Filter collinear points to only keep course changes
x_coords, y_coords = [raw_x[0]], [raw_y[0]]

for i in range(1, len(raw_x) - 1):
    dx1 = raw_x[i] - raw_x[i-1]
    dy1 = raw_y[i] - raw_y[i-1]
    dx2 = raw_x[i+1] - raw_x[i]
    dy2 = raw_y[i+1] - raw_y[i]
    
    angle1 = math.atan2(dy1, dx1)
    angle2 = math.atan2(dy2, dx2)
    
    diff = abs(angle1 - angle2)
    if diff > math.pi:
        diff = 2 * math.pi - diff
        
    if diff > 1e-5:
        x_coords.append(raw_x[i])
        y_coords.append(raw_y[i])

x_coords.append(raw_x[-1])
y_coords.append(raw_y[-1])

# 3. Plotting Setup
fig, ax = plt.subplots(figsize=(14, 9))
ax.set_facecolor('#E0F7FA') # Light blue water background

# 4. Draw the Wind Field
if wind_field:
    wx, wy, u, v = [], [], [], []
    for w in wind_field:
        spd = w["spd"]
        if spd > 0:
            wx.append(w["x"])
            wy.append(w["y"])
            # Calculate vector directions. Wind direction is where it blows FROM.
            # Y-axis is inverted in our graph, so North (0) blows South (+Y), East (90) blows West (-X)
            u_comp = -math.sin(math.radians(w["dir"]))
            v_comp = math.cos(math.radians(w["dir"]))
            u.append(u_comp * spd)
            v.append(v_comp * spd)
    
    # Render the wind as small semi-transparent arrows
    ax.quiver(wx, wy, u, v, color='#80DEEA', alpha=0.7, width=0.0015, headwidth=4, headlength=5, scale=500, zorder=1, label='Wind Field')

# 5. Plot the land blocks in yellow
if land:
    land_x = [l["x"] for l in land]
    land_y = [l["y"] for l in land]
    ax.scatter(land_x, land_y, color='#F4D03F', marker='s', s=45, zorder=2, label='Land')

# 6. Plot the route in תכלת
ax.plot(x_coords, y_coords, marker='o', color='deepskyblue', linestyle='-', linewidth=3, markersize=8, zorder=3, label='Theta* Route')
ax.scatter([start["x"]], [start["y"]], color='green', s=150, zorder=5, label='Start')
ax.scatter([dest["x"]], [dest["y"]], color='red', s=150, zorder=5, label='Destination')

ax.set_title("Sailing Engine Pathfinding Debugger")
ax.set_xlabel("X Grid Coordinate")
ax.set_ylabel("Y Grid Coordinate")
ax.invert_yaxis()
ax.set_xlim(0, grid_w)
ax.set_ylim(grid_h, 0)
ax.grid(True, color='white', linestyle='-', linewidth=0.7)
ax.legend(loc='upper right')
plt.show()