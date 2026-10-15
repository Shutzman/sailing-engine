import json
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button
import tkinter as tk
from tkinter import messagebox

# 1. Load the Route Data
with open("route_output.json", "r") as f:
    data = json.load(f)

start = data["start"]
dest = data["destination"]
path = data["path"]
land = data.get("land", [])
wind_field = data.get("wind_field", [])
metrics = data.get("metrics", {})
grid_w = data.get("width", 150)
grid_h = data.get("height", 150)

raw_x = [node["x"] for node in path]
raw_y = [node["y"] for node in path]

if not raw_x or (raw_x[0] != start["x"] or raw_y[0] != start["y"]):
    raw_x.insert(0, start["x"])
    raw_y.insert(0, start["y"])

# 2. Process Waypoints and Build Captain's Logbook
x_coords, y_coords = [raw_x[0]], [raw_y[0]]

logbook_text = "\n--- CAPTAIN'S LOGBOOK ---\n"
logbook_text += f"Departure from ({start['x']}, {start['y']})\n"

leg_counter = 1
for i in range(1, len(raw_x) - 1):
    dx1, dy1 = raw_x[i] - raw_x[i-1], raw_y[i] - raw_y[i-1]
    dx2, dy2 = raw_x[i+1] - raw_x[i], raw_y[i+1] - raw_y[i]
    
    angle1 = math.atan2(dy1, dx1)
    angle2 = math.atan2(dy2, dx2)
    
    diff = abs(angle1 - angle2)
    if diff > math.pi:
        diff = 2 * math.pi - diff
        
    if diff > 1e-5:
        x_coords.append(raw_x[i])
        y_coords.append(raw_y[i])
        
        compass_heading = (90 + math.degrees(math.atan2(dy1, dx1))) % 360
        leg_dist = math.hypot(x_coords[-1] - x_coords[-2], y_coords[-1] - y_coords[-2])
        logbook_text += f"Leg {leg_counter}: Turn to {compass_heading:03.0f}° and sail for {leg_dist:.1f} cells to ({raw_x[i]}, {raw_y[i]})\n"
        leg_counter += 1

x_coords.append(raw_x[-1])
y_coords.append(raw_y[-1])

dx_final = x_coords[-1] - x_coords[-2]
dy_final = y_coords[-1] - y_coords[-2]
final_heading = (90 + math.degrees(math.atan2(dy_final, dx_final))) % 360
final_dist = math.hypot(dx_final, dy_final)
logbook_text += f"Leg {leg_counter}: Turn to {final_heading:03.0f}° and sail for {final_dist:.1f} cells to ({dest['x']}, {dest['y']})\n"
logbook_text += "Arrived at destination.\n-------------------------\n"

# 3. Compute Metrics (with fallback if JSON metrics are missing)
computed_aerial = math.hypot(dest["x"] - start["x"], dest["y"] - start["y"])
computed_route = sum(
    math.hypot(x_coords[i] - x_coords[i-1], y_coords[i] - y_coords[i-1])
    for i in range(1, len(x_coords))
)

cost_val = metrics.get("cost", 0.0)
aerial_val = metrics.get("aerial_distance", 0.0) or computed_aerial
route_val = metrics.get("route_distance", 0.0) or computed_route

metrics_text = (
    f"\n--- TRIP METRICS ---\n"
    f"Actual Cost:     {cost_val:.1f}\n"
    f"Aerial Distance: {aerial_val:.1f} cells\n"
    f"Route Distance:  {route_val:.1f} cells\n"
    f"--------------------\n"
)

# 4. Popup Window Helper Function
def open_popup_window(title, content):
    root = tk.Tk()
    root.withdraw()  # Hide the root tkinter window frame
    messagebox.showinfo(title, content)
    root.destroy()

# 5. Main Plotter UI Setup
fig, ax = plt.subplots(figsize=(14, 9))
plt.subplots_adjust(left=0.16, top=0.86)
ax.set_facecolor('#E0F7FA')

# Draw Wind Field Quiver
quiver_plot = None
if wind_field:
    wx, wy, u, v = [], [], [], []
    for w in wind_field:
        spd = w["spd"]
        if spd > 0:
            wx.append(w["x"])
            wy.append(w["y"])
            u.append(-math.sin(math.radians(w["dir"])) * spd)
            v.append(math.cos(math.radians(w["dir"])) * spd)
    quiver_plot = ax.quiver(wx, wy, u, v, color='#80DEEA', alpha=0.7, width=0.0015, headwidth=4, scale=500, zorder=1)

# Draw Land Obstacles
land_scatter = None
if land:
    land_scatter = ax.scatter([l["x"] for l in land], [l["y"] for l in land], color='#F4D03F', marker='s', s=45, zorder=2)

# Draw Route, Start, and Destination
route_plot, = ax.plot(x_coords, y_coords, marker='o', color='#0099FF', linestyle='-', linewidth=3, markersize=5, zorder=3)
ax.scatter([start["x"]], [start["y"]], color='#27AE60', s=160, zorder=5, label='Start')
ax.scatter([dest["x"]], [dest["y"]], color='#C0392B', s=160, zorder=5, label='Destination')

ax.set_title("Tactical Navigation Plotter — Sailing Engine", pad=15, fontsize=15, fontweight='bold', color='#1A252F')
ax.invert_yaxis()
ax.set_xlim(0, grid_w)
ax.set_ylim(grid_h, 0)
ax.grid(True, color='white', linestyle='-', linewidth=0.8)

# 6. Layer Toggle Checkboxes (Left Panel)
ax_check = plt.axes([0.02, 0.70, 0.11, 0.15])
labels = ['Wind', 'Land', 'Route']
visibility = [True, True, True]
check = CheckButtons(ax_check, labels, visibility)

def toggle_layers(label):
    if label == 'Wind' and quiver_plot:
        quiver_plot.set_visible(not quiver_plot.get_visible())
    elif label == 'Land' and land_scatter:
        land_scatter.set_visible(not land_scatter.get_visible())
    elif label == 'Route':
        route_plot.set_visible(not route_plot.get_visible())
    plt.draw()

check.on_clicked(toggle_layers)

# 7. Styled Action Buttons (Top Bar) - Prints to terminal AND opens popup
ax_metrics_btn = plt.axes([0.35, 0.92, 0.14, 0.045])
btn_metrics = Button(ax_metrics_btn, 'View Metrics', color='#2980B9', hovercolor='#3498DB')

def open_metrics_popup(event):
    print(metrics_text)  # Terminal output
    open_popup_window("Trip Performance Metrics", metrics_text)  # Popup window

btn_metrics.on_clicked(open_metrics_popup)
btn_metrics.label.set_color('white')
btn_metrics.label.set_fontweight('bold')

ax_log_btn = plt.axes([0.52, 0.92, 0.14, 0.045])
btn_log = Button(ax_log_btn, "Captain's Logbook", color='#2980B9', hovercolor='#3498DB')

def open_logbook_popup(event):
    print(logbook_text)  # Terminal output
    open_popup_window("Captain's Navigational Logbook", logbook_text)  # Popup window

btn_log.on_clicked(open_logbook_popup)
btn_log.label.set_color('white')
btn_log.label.set_fontweight('bold')

plt.show()