import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
import matplotlib.colors as colors
from matplotlib.patches import Circle, Arrow
# Constants
CAR_LENGTH = 4.5  # Example length
CAR_WIDTH = 2.0   # Example width

def plot_car(x, y, yaw, the_color,alpha, car_length, car_width):
    """
    Plot a simple car shape (rectangle) given its center, heading, length, and width.
    """
    # Define car corners
    rear_left = [-car_length / 2, -car_width / 2]
    rear_right = [-car_length / 2, car_width / 2]
    front_left = [car_length / 2, -car_width / 2]
    front_right = [car_length / 2, car_width / 2]

    # Rotate and translate the corners
    corners = [rear_left, rear_right, front_right, front_left, rear_left]
    corners_rotated = [
        (
            x + (corner[0] * np.cos(yaw) - corner[1] * np.sin(yaw)),
            y + (corner[0] * np.sin(yaw) + corner[1] * np.cos(yaw))
        )
        for corner in corners
    ]

    # Extract x and y coordinates to plot
    x_vals, y_vals = zip(*corners_rotated)

    plt.plot(x_vals, y_vals, color= the_color, alpha = alpha)

def plot_trajectories(trajectories):
    """
    Plot given trajectories and car shapes.
    """
    plt.figure()

    # For each trajectory
    for trajectory in trajectories:
        # Extract x, y, yaw values
        x_vals = [point[0] for point in trajectory]
        y_vals = [point[1] for point in trajectory]

        # Plot the trajectory
        plt.plot(x_vals, y_vals, 'r-')

        # Plot the car shapes
        for point in trajectory:
            plot_car(point[0], point[1], point[2])

    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.title("Planned Trajectories")
    # plt.grid(True)
    # plt.show()
def plot_traj(traj, color,car_length, car_width):
    #for point in traj:
        # print("point: ", point)
    x_vals = [point[0] for point in traj]
    y_vals = [point[1] for point in traj]
    alphas = np.linspace(1, 0.1, len(traj))
    # Plot the trajectory
    # plt.plot(x_vals, y_vals, 'r-')
    # color = "b"
    # Plot the car shapes
    the_color = color
    for i, point in enumerate(traj):
        # alpha = 1
        plot_car(point[0], point[1], point[2], the_color, alphas[i] ,car_length, car_width)
    # print("color history: ",his)
    # assert 1==2, "checking color"



def plot_topo(node_list,edge_list, ax, if_points = False, if_arrow = False):

    # for edge in edge_list:
    #     print("----------------------")
    #     print("----> ", edge)

    # fig,ax = plt.subplots()
    # img = load_img('icat.png')

    for node_id, data in node_list:
        x, y, yaw = data["coord"]
        
        # Differentiate color based on "itsc" property
        color = 'green' if data["itsc"] else 'blue'
        
        # Draw the disk patch (circle)
        circle = Circle((x, y), radius=1, color=color, ec="black")  # ec stands for edgecolor
        ax.add_patch(circle)
        
        # Add node ID inside the circle
        ax.text(x, y, str(node_id), ha='center', va='center', color='white')

    for edge in edge_list:
        points = edge[2]["waypoints"]
        #points = [(10, 10), (30, 40), (55, 5), (5, 45)]

        # Unzip the points to separate x and y coordinates for easy plotting
        x_coords, y_coords, yaw = zip(*points)

        if if_points:
            # Plotting the points
            plt.scatter(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.
        # Drawing lines connecting the points
        plt.plot(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.

    if if_arrow:
        for edge in edge_list:
            points = edge[2]["waypoints"]
            for x, y, yaw in points:
                arrow_length = 0.5  # Adjust as needed
                # plt.scatter(x, y, color='red')
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                
                arrow = Arrow(x, y, dx, dy, width=0.2, color='red')  # Adjust width and color as needed
                ax.add_patch(arrow)

    # Plotting the image
    # plt.imshow(img, origin='lower', extent=[0, 60, 0, 50])  # The extent parameter sets the axis limits and 'origin' is now set to 'lower'.


    # Displaying the plot
    # plt.show()
def plot_car_patch(x, y, yaw, color='blue', car_length=CAR_LENGTH, car_width=CAR_WIDTH):
    """
    Plot a car shape using patches.
    """
    # Define the bottom-left corner of the rectangle
    corner = [
        x - (car_length / 2) * np.cos(yaw) + (car_width / 2) * np.sin(yaw),
        y - (car_length / 2) * np.sin(yaw) - (car_width / 2) * np.cos(yaw)
    ]
    
    # Create a rectangle representing the car
    car = patches.Rectangle(
        corner, car_length, car_width,
        angle=np.degrees(yaw), color=color
    )

    plt.gca().add_patch(car)

def plot_cars(trajs, car_length, car_width):
    num_cars = len(trajs)
    color_list = ['red', 'yellow', 'blue', 'green', 'purple']
    values = np.linspace(0, 1, num_cars)
    color_map = colors.LinearSegmentedColormap.from_list('my_cmap', ['green','yellow', 'red', 'purple','blue'])
    for car_id, traj in enumerate(trajs):
        x0,y0,yaw0,_,_ = traj[0]  
        this_color = color_map(values[car_id])
        # print("this color: ", this_color)
        # assert 1==2, "  check"
        plot_car_patch(x0,y0,yaw0, this_color, car_length, car_width)
        plot_traj(traj,this_color, car_length, car_width)