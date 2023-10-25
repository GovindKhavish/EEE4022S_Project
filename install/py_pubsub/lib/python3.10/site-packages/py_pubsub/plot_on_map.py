import matplotlib.pyplot as plt
import csv
from matplotlib.patches import Arrow

# Create a blank canvas with specified dimensions
width, height = 12, 10
origin_x, origin_y = -4, 0

# Create a figure
fig, ax = plt.subplots()
ax.set_xlim(origin_x, origin_x + width)
ax.set_ylim(origin_y, origin_y + height)

with open('/home/riser14/ros_ws/odom_data_mppisparse_test.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    path_data = [(float(row['X']), float(row['Y']), float(row['Z']), float(row['x']), float(row['y']), float(row['z']), float(row['w'])) for row in reader]

# Extract the position data and 'x' values
# Extract the position data
x, y, z, _, _, _, _ = zip(*path_data)

# Calculate the minimum and maximum coordinates
min_x, max_x = min(x), max(x)
min_y, max_y = min(y), max(y)

# Determine the canvas size with some padding
padding = 1.0
width = max_x - min_x + 2 * padding
height = max_y - min_y + 2 * padding

# Create a figure
fig, ax = plt.subplots()
ax.set_xlim(min_x - padding, max_x + padding)
ax.set_ylim(min_y - padding, max_y + padding)

# Plot the path on the canvas
ax.plot(x, y, marker='o', markersize=0.25, color='grey', linestyle='-')

# Add arrows to represent orientation at each point
#for i in range(len(x)):
#    arrow = Arrow(x[i], y[i], path_data[i][3], path_data[i][4], width=0.1, color='blue')
#    ax.add_patch(arrow)

ind = 0
# Annotate every 5th point with the symbol 'x'
#for i in range(len(x)):
#    if i % 100 == 0:
#        if ind == 1:
#            ind += 1
#            pass
#        else:
#            ax.text(x[i], y[i], 'x', fontsize=16, ha='center', va='center')
#            ind +=1

# Display the canvas with the path, orientation, and 'x' symbols
print(ind)
plt.show()