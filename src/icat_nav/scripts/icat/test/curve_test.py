import numpy as np
import matplotlib.pyplot as plt
import math

def draw_curve(p1, p2, turn_direction, radius):
    # Based on the turn direction, determine the circle's center
    if turn_direction == "left":
        cx, cy = p1[0], p1[1] + radius
    elif turn_direction == "right":
        cx, cy = p1[0], p1[1] - radius
    else:
        raise ValueError("Turn direction must be 'left' or 'right'.")

    # Calculate angles for our arc
    start_angle = math.atan2(p1[1] - cy, p1[0] - cx)
    end_angle = math.atan2(p2[1] - cy, p2[0] - cx)

    # Create the arc
    theta = np.linspace(start_angle, end_angle, 100)
    x = cx + radius * np.cos(theta)
    y = cy + radius * np.sin(theta)

    # Plot
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, '-b', label='curve')
    plt.scatter([p1[0], p2[0]], [p1[1], p2[1]], color='red', label='endpoints')
    plt.scatter(cx, cy, color='green', label='center')
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()

# Test
p1 = (0, 0)
p2 = (0, 4)
draw_curve(p1, p2, "left", 2)