import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import MultipleLocator

def main():
    # Adjust this path if you run the script from a different directory
    csv_file = os.path.expanduser("~/Hawkeye-OS/datalog/target_world_coords.csv")
    
    if not os.path.exists(csv_file):
        print(f"Error: Could not find '{csv_file}'. Has the node recorded any data yet?")
        return

    # Load data
    df = pd.read_csv(csv_file)
    
    if df.empty:
        print("CSV is empty.")
        return

    # Normalize timestamp so it starts at t=0
    df['Time (s)'] = df['Timestamp'] - df['Timestamp'].iloc[0]

    # Calculate 3D point density for coloring
    xyz = df[['X', 'Y', 'Z']].values
    bins = 20 # Resolution of the density grid
    
    # Generate 3D histogram
    H, edges = np.histogramdd(xyz, bins=bins)
    
    # Map each point back to its grid bin to get its local density count
    idx_x = np.clip(np.digitize(df['X'], edges[0]) - 1, 0, bins - 1)
    idx_y = np.clip(np.digitize(df['Y'], edges[1]) - 1, 0, bins - 1)
    idx_z = np.clip(np.digitize(df['Z'], edges[2]) - 1, 0, bins - 1)
    
    # Assign density value to each point
    density = H[idx_x, idx_y, idx_z]
    
    # Optional: Sort points by density so denser points are plotted on top of lighter ones
    sort_idx = density.argsort()
    x_sorted = df['X'].iloc[sort_idx]
    y_sorted = df['Y'].iloc[sort_idx]
    z_sorted = df['Z'].iloc[sort_idx]
    density_sorted = density[sort_idx]

    # Create figure with 2 subplots side-by-side
    fig = plt.figure(figsize=(16, 7))

    # ---- Plot 1: 3D Scatter Plot ----
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Plot origin as a distinct mark
    ax1.scatter([0], [0], [0], color='black', marker='x', s=100, label='Origin (0,0,0)', zorder=5)
    
    # Use density for color mapping (cmap='Blues' -> darker blue for higher values)
    scatter = ax1.scatter(x_sorted, y_sorted, z_sorted, c=density_sorted, cmap='Blues', marker='o', alpha=0.8)
    
    ax1.set_xlabel('X (World) [m]')
    ax1.set_ylabel('Y (World) [m]')
    ax1.set_zlabel('Z (World) [m]')
    ax1.set_title('3D Target Position mapped by Point Density')
    fig.colorbar(scatter, ax=ax1, label='Density (Points in bin)')

    # Step 1: Find the absolute bounds, forcing them to at least include 0
    min_x = min(0.0, df['X'].min())
    max_x = max(0.0, df['X'].max())
    min_y = min(0.0, df['Y'].min())
    max_y = max(0.0, df['Y'].max())
    min_z = min(0.0, df['Z'].min())
    max_z = max(0.0, df['Z'].max())
    
    # Step 2: Determine the largest span among all axes to force 1:1:1 aspect ratio
    max_range = max(max_x - min_x, max_y - min_y, max_z - min_z)
    
    # Optional: ensure it spans at least 0.5m across so 0.1m grids don't overlap too tightly
    if max_range < 0.5:
        max_range = 0.5
        
    # Step 3: Center the spanning bounding box so it encompasses origin AND data perfectly
    mid_x = (max_x + min_x) * 0.5
    mid_y = (max_y + min_y) * 0.5
    mid_z = (max_z + min_z) * 0.5
    
    ax1.set_xlim(mid_x - max_range / 2.0, mid_x + max_range / 2.0)
    ax1.set_ylim(mid_y - max_range / 2.0, mid_y + max_range / 2.0)
    ax1.set_zlim(mid_z - max_range / 2.0, mid_z + max_range / 2.0)

    # Force the ticks on the 3D plot to be exactly 0.1m apart
    ax1.xaxis.set_major_locator(MultipleLocator(0.1))
    ax1.yaxis.set_major_locator(MultipleLocator(0.1))
    ax1.zaxis.set_major_locator(MultipleLocator(0.1))
    ax1.legend()

    # ---- Plot 2: X, Y, Z over Time ----
    ax2 = fig.add_subplot(122)
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='Origin (0m)')
    
    ax2.plot(df['Time (s)'], df['X'], label='X (Forward)', color='red', linewidth=2)
    ax2.plot(df['Time (s)'], df['Y'], label='Y (Left)', color='green', linewidth=2)
    ax2.plot(df['Time (s)'], df['Z'], label='Z (Up)', color='blue', linewidth=2)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('World Position [m]')
    ax2.set_title('World Coordinates vs Time')
    
    # Force Y-axis to cover the origin and the data
    all_min = min(0.0, df['X'].min(), df['Y'].min(), df['Z'].min())
    all_max = max(0.0, df['X'].max(), df['Y'].max(), df['Z'].max())
    
    # Add a little padding to the top and bottom (20cm)
    ax2.set_ylim(all_min - 0.2, all_max + 0.2)
    
    # Force the Y-axis (meters) on the time graph to also be exactly 0.1m apart
    ax2.yaxis.set_major_locator(MultipleLocator(0.1))
    
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()