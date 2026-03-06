import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

# ==========================================
# CONFIGURATION
# ==========================================
FILENAME = 'out.txt'  # The name of your text file

# ==========================================
# 1. PARSING THE DATA
# ==========================================
x_vals = []
y_vals = []
z_vals = []
indices = []

# Regex pattern to find x, y, z values even if there is other text on the line
# It looks for "x:", captured number, "y:", captured number, "z:", captured number
pattern = re.compile(r"x:([-\d\.]+).*y:([-\d\.]+).*z:([-\d\.]+)")

try:
    with open(FILENAME, 'r') as f:
        for i, line in enumerate(f):
            # Search for the pattern in the line
            match = pattern.search(line)
            if match:
                try:
                    # Convert strings to floats
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                    
                    x_vals.append(x)
                    y_vals.append(y)
                    z_vals.append(z)
                    indices.append(i)
                except ValueError:
                    print(f"Skipping line {i}: Could not parse numbers.")
                    
    print(f"Successfully parsed {len(indices)} data points.")

except FileNotFoundError:
    print(f"Error: The file '{FILENAME}' was not found.")
    exit()

if len(x_vals) == 0:
    print("No matching data found in file. Check your format.")
    exit()

# ==========================================
# 2. PLOTTING FOR ANOMALY DETECTION
# ==========================================

# --- Figure 1: Component Analysis (Best for spotting spikes/noise) ---
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
fig1.suptitle(f'Component Analysis (X, Y, Z over Time)\nLook for sudden spikes or dropouts here', fontsize=14)

# Plot X
ax1.plot(indices, x_vals, color='r', label='X Data')
ax1.set_ylabel('X Position')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.legend(loc='upper right')

# Plot Y
ax2.plot(indices, y_vals, color='g', label='Y Data')
ax2.set_ylabel('Y Position')
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.legend(loc='upper right')

# Plot Z
ax3.plot(indices, z_vals, color='b', label='Z Data')
ax3.set_ylabel('Z Position')
ax3.set_xlabel('Sample Index (Time)')
ax3.grid(True, linestyle='--', alpha=0.6)
ax3.legend(loc='upper right')

# --- Figure 2: 3D Spatial Plot (Best for path verification) ---
fig2 = plt.figure(figsize=(10, 8))
ax_3d = fig2.add_subplot(111, projection='3d')
ax_3d.set_title('3D Spatial Trajectory')

# Plot the path line
ax_3d.plot(x_vals, y_vals, z_vals, color='gray', alpha=0.5, linewidth=0.5)
# Plot the actual points
p = ax_3d.scatter(x_vals, y_vals, z_vals, c=indices, cmap='viridis', s=10)

ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('Z')

# Add a color bar to show time progression
cbar = plt.colorbar(p, ax=ax_3d, pad=0.1)
cbar.set_label('Time (Sample Index)')

plt.show()
