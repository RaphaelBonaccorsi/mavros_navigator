import sys
import ast
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

def main():
    if len(sys.argv) < 3:
        print("Usage: python trajectory_analyst.py <trajectory_log_file> <obstacles_file> [x_coords_file y_coords_file]")
        sys.exit(1)

    trajectory_file = sys.argv[1]
    obstacles_file = sys.argv[2]

    # Read trajectory points (ignore timestamp and altitude)
    xs, ys = [], []
    with open(trajectory_file, "r") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 3:
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))

    # Read obstacles
    with open(obstacles_file, "r") as f:
        obstacles = ast.literal_eval(f.read())

    # Optional: Read extra x and y coordinates if provided
    extra_xs, extra_ys = None, None
    if len(sys.argv) == 5:
        x_coords_file = sys.argv[3]
        y_coords_file = sys.argv[4]
        with open(x_coords_file, "r") as xf, open(y_coords_file, "r") as yf:
            extra_xs = [float(line.strip()) for line in xf if line.strip()]
            extra_ys = [float(line.strip()) for line in yf if line.strip()]

    # Plot trajectory
    plt.figure(figsize=(10, 8))
    plt.plot(xs, ys, label="Drone Trajectory", color="blue", linewidth=2)

    # Plot obstacles
    for obs in obstacles:
        poly = Polygon(obs['polygon'])
        x, y = poly.exterior.xy
        plt.fill(x, y, alpha=0.4, label=f"Obstacle {obs['id']}")

    # Plot extra coordinates if provided
    if extra_xs is not None and extra_ys is not None:
        plt.plot(extra_xs, extra_ys, 'r--', label="Reference Path", linewidth=2)

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Drone Trajectory and Obstacles")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()