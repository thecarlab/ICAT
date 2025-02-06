import numpy as np
import matplotlib.pyplot as plt

def bezier_curve(P0, P1, P2, P3, t):
    x = (1-t)**3*P0[0] + 3*(1-t)**2*t*P1[0] + 3*(1-t)*t**2*P2[0] + t**3*P3[0]
    y = (1-t)**3*P0[1] + 3*(1-t)**2*t*P1[1] + 3*(1-t)*t**2*P2[1] + t**3*P3[1]
    return (x, y)

def compute_curve_length(curve_points):
    length = 0
    for i in range(1, len(curve_points)):
        length += np.linalg.norm(np.array(curve_points[i]) - np.array(curve_points[i-1]))
    return length

def sample_waypoints(curve_points, distance):
    waypoints = [curve_points[0]]  # Start with the first point
    accumulated_distance = 0
    
    for i in range(1, len(curve_points)):
        segment = np.array(curve_points[i]) - np.array(curve_points[i-1])
        segment_length = np.linalg.norm(segment)
        
        while accumulated_distance + segment_length >= distance:
            # interpolate the next waypoint
            alpha = (distance - accumulated_distance) / segment_length
            waypoint = curve_points[i-1] + alpha * segment
            waypoints.append(waypoint)
            
            # Reset and update accumulated distance
            segment_length -= (distance - accumulated_distance)
            accumulated_distance = 0
            
        accumulated_distance += segment_length
        
    return waypoints

# points = [(48,3.25), (54,3.25), (56.5, 7), (56.5, 10.8)]
# t_values = np.linspace(0, 1, 1000)
# curve_points = [bezier_curve(*points, t) for t in t_values]

# # Compute curve length
# length = compute_curve_length(curve_points)
# print(f"Approximate Length of Curve: {length:.2f} meters")

# # Sample waypoints at 0.5m intervals
# waypoints = sample_waypoints(curve_points, 0.5)

# print("len of waypoints: ", len(waypoints))
# # Plotting
# plt.figure(figsize=(8, 8))
# plt.plot([p[0] for p in curve_points], [p[1] for p in curve_points], '-b', label="Bezier Curve")
# plt.scatter(*zip(*points), color='red', label="Control Points")
# plt.scatter([p[0] for p in waypoints], [p[1] for p in waypoints], color='green', s=50, zorder=5, label="Waypoints")
# plt.gca().set_aspect('equal', adjustable='box')
# plt.legend()
# plt.grid(True)
# plt.show()





