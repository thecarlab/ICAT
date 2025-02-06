
import numpy as np
import matplotlib.pyplot as plt

def bezier_curve(P0, P1, P2, P3):
    t = np.linspace(0, 1, 100)
    curve = np.outer((1-t)**3, P0) + np.outer(3*(1-t)**2*t, P1) + np.outer(3*(1-t)*t**2, P2) + np.outer(t**3, P3)
    
    return curve

def plot_curve(points):
    P0, P1, P2, P3 = points
    curve = bezier_curve(P0, P1, P2, P3)
    plt.figure(figsize=(8, 8))
    plt.plot(curve[:, 0], curve[:, 1], '-b', label="Bezier Curve")
    plt.scatter(*zip(*points), color='red', label="Control Points")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.grid(True)
    plt.show()

points = [(48,3.25), (54,3.25), (56.5, 7), (56.5, 10.8)]
plot_curve(points)
    
