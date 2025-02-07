import numpy as np
from sklearn.cluster import DBSCAN
from scipy.stats import multivariate_normal
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt

# Load the pose_pts.npy file
data = np.load('/home/tian/icat_ws/src/icat_nav/data/pose_pts.npy')

# Ensure we're only working with the first two columns of data
if data.shape[1] < 2:
    raise ValueError("Input data must contain at least two columns.")

data = data[:, :2]

# Perform clustering using DBSCAN
dbscan = DBSCAN(eps=0.01)  # Specify the threshold (1cm)
labels = dbscan.fit_predict(data)

# Check unique labels to avoid issues with potential noise points
unique_labels = np.unique(labels)
print(f'Unique labels after clustering: {unique_labels}')

# Set a threshold for detecting outliers
threshold = 1e-5

for i in unique_labels:
    if i == -1:  # -1 is the label for noise points in DBSCAN
        continue
    
    # Get cluster points
    cluster_points = data[labels == i]
    
    if len(cluster_points) < 2:  # Need at least two points to fit a distribution
        print(f'Cluster {i} has less than 2 points, skipping.')
        continue

    # Calculate mean and covariance
    mean = np.mean(cluster_points, axis=0)
    cov = np.cov(cluster_points.T)
    
    try:
        # Calculate the eigenvalues and eigenvectors for the covariance
        eigvals, eigvecs = np.linalg.eig(cov)
    except np.linalg.LinAlgError:
        print(f'Could not compute eigenvalues for cluster {i}, skipping.')
        continue
    
    # Calculate standard deviations from the covariance matrix
    std_dev = np.sqrt(np.diag(cov))

    # Plot the cluster points
    plt.scatter(cluster_points[:, 0], cluster_points[:, 1], label=f'Cluster {i}', alpha=0.5)

    # First method: using eigenvalues for ellipses
    if np.all(eigvals > 0):  # Ensure eigenvalues are positive
        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
        ellipse_eigen = Ellipse(xy=mean, width=2 * np.sqrt(eigvals[0]), height=2 * np.sqrt(eigvals[1]),
                                angle=angle, color='lightblue', label='Eigenvalue Based', alpha=0.5)
        plt.gca().add_patch(ellipse_eigen)
    
    # Second method: using standard deviations for ellipses
    ellipse_std = Ellipse(xy=mean, width=2 * std_dev[0], height=2 * std_dev[1], 
                          angle=angle, color='lightgreen', label='Standard Deviation Based', alpha=0.5)
    plt.gca().add_patch(ellipse_std)

    # Plot the mean as a point
    plt.scatter(mean[0], mean[1], color='red', marker='x', s=100, label='Mean')

# Prepare the new_pts array to hold means and unclustered points
new_pts = []

# Iterate over unique labels to compute cluster means
unique_labels = np.unique(labels)
for i in unique_labels:
    if i == -1:  # Skip noise points (unclustered)
        continue

    # Get cluster points
    cluster_points = data[labels == i]
    print("Cluster points shape:", cluster_points.shape)
    
    if len(cluster_points) > 0:  # Ensure there are points in the cluster
        mean = np.mean(cluster_points, axis=0)  # Calculate the mean of the cluster
        new_pts.append(mean)  # Append the mean to new_pts

# Add the unclustered points (noise) to new_pts
unclustered_points = data[labels == -1]
new_pts.extend(unclustered_points)  # Extend new_pts with unclustered points

# Convert new_pts list to a numpy array
new_pts = np.array(new_pts)

print("Original points shape:", data.shape)
print("New points shape:", new_pts.shape)

np.save('/home/tian/icat_ws/src/icat_nav/data/pose_pts_clustered.npy', new_pts)

plt.title("Clustered Points with Ellipses for Variance")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.legend()
plt.show()