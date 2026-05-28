import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# =========================================================
# CREATE FIGURES DIRECTORY
# =========================================================

os.makedirs("figures", exist_ok=True)


# =========================================================
# LOAD CSV
# =========================================================

df = pd.read_csv("logs/flight_log.csv")


# =========================================================
# 1. 2D TRAJECTORY PLOT
# =========================================================

plt.figure(figsize=(8, 8))

plt.plot(df["x"], df["y"], linewidth=2)

# Start point
plt.scatter(
    df["x"].iloc[0],
    df["y"].iloc[0],
    s=100,
    label="Start"
)

# End point
plt.scatter(
    df["x"].iloc[-1],
    df["y"].iloc[-1],
    s=100,
    label="End"
)

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Drone Trajectory")

plt.legend()
plt.grid(True)

plt.savefig("figures/trajectory_2d.png")
plt.close()


# =========================================================
# 2. 3D TRAJECTORY PLOT
# =========================================================

fig = plt.figure(figsize=(10, 8))

ax = fig.add_subplot(111, projection='3d')

scatter = ax.scatter(
    df["x"],
    df["y"],
    df["z"],
    c=df["time"],
    cmap='viridis',
    s=8
)

# Start point
ax.scatter(
    df["x"].iloc[0],
    df["y"].iloc[0],
    df["z"].iloc[0],
    s=100,
    label="Start"
)

# End point
ax.scatter(
    df["x"].iloc[-1],
    df["y"].iloc[-1],
    df["z"].iloc[-1],
    s=100,
    label="End"
)

ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Altitude Z (m)")

ax.set_title("3D Drone Trajectory")

fig.colorbar(scatter, ax=ax, label="Time (s)")

ax.legend()

plt.savefig("figures/trajectory_3d.png")

plt.close()


# =========================================================
# 3. ALTITUDE PROFILE
# =========================================================

plt.figure(figsize=(10, 5))

plt.plot(df["time"], df["z"], linewidth=2)

plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Altitude vs Time")

plt.grid(True)

plt.savefig("figures/altitude.png")

plt.close()


# =========================================================
# 4. ALIGNMENT ERROR
# =========================================================

plt.figure(figsize=(10, 5))

plt.plot(df["time"], df["marker_x"], label="mx")
plt.plot(df["time"], df["marker_y"], label="my")

plt.xlabel("Time (s)")
plt.ylabel("Alignment Error")

plt.title("Marker Alignment Error")

plt.legend()

plt.grid(True)

plt.savefig("figures/alignment_error.png")

plt.close()


# =========================================================
# 5. MOTOR COMMANDS
# =========================================================

plt.figure(figsize=(12, 6))

plt.plot(df["time"], df["motor_fl"], label="FL")
plt.plot(df["time"], df["motor_fr"], label="FR")
plt.plot(df["time"], df["motor_rl"], label="RL")
plt.plot(df["time"], df["motor_rr"], label="RR")

plt.xlabel("Time (s)")
plt.ylabel("Motor Command")

plt.title("Motor Commands vs Time")

plt.legend()

plt.grid(True)

plt.savefig("figures/motor_commands.png")

plt.close()


# =========================================================
# 6. STATE TIMELINE
# =========================================================

states = df["state"].astype("category")
codes = states.cat.codes

plt.figure(figsize=(12, 4))

plt.plot(df["time"], codes, linewidth=2)

plt.yticks(
    range(len(states.cat.categories)),
    states.cat.categories
)

plt.xlabel("Time (s)")
plt.ylabel("State")

plt.title("State Timeline")

plt.grid(True)

plt.savefig("figures/state_timeline.png")

plt.close()


# =========================================================
# 7. SNAKE SEARCH PATTERN
# =========================================================

TAKEOFF_HEIGHT = 2.0


def generate_snake():

    waypoints = []

    for x in range(-5, 6):

        if (x + 5) % 2 == 0:
            ys = range(-5, 6)

        else:
            ys = range(5, -6, -1)

        for y in ys:
            waypoints.append((x, y, TAKEOFF_HEIGHT))

    return waypoints


waypoints = generate_snake()

snake_x = []
snake_y = []
snake_z = []

for wp in waypoints:

    snake_x.append(wp[0])
    snake_y.append(wp[1])
    snake_z.append(wp[2])


fig = plt.figure(figsize=(10, 8))

ax = fig.add_subplot(111, projection='3d')

ax.plot(
    snake_x,
    snake_y,
    snake_z,
    linewidth=2
)

ax.set_title("Snake Search Pattern")

ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_zlabel("Altitude")

plt.savefig("figures/snake_pattern.png")

plt.close()


# =========================================================
# METRICS
# =========================================================

landing_x = df["x"].iloc[-1]
landing_y = df["y"].iloc[-1]

landing_error = (landing_x**2 + landing_y**2) ** 0.5

detection_rate = df["marker_detected"].mean() * 100

total_time = df["time"].iloc[-1]

print("\n========== METRICS ==========")

print(f"Landing Error: {landing_error:.3f} m")

print(f"Detection Rate: {detection_rate:.2f}%")

print(f"Flight Time: {total_time:.2f} s")


print("\nFigures saved to /figures")