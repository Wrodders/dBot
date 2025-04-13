import matplotlib.pyplot as plt

import numpy as np
import cv2


def main():
    with open('track_points.txt', 'r') as file:
        lines = file.readlines() # Read all lines from the file
        # make points numpy array
        points = []
        for line in lines:
            x, y = map(float, line.strip().split(','))
            points.append((x, y))
        points = np.array(points)
        
    # Create a figure and axis
    fig, ax = plt.subplots()
    # Plot the points
    ax.plot(points[:, 0], points[:, 1], marker='o', linestyle='-', color='b')
    # Set labels and title
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Robot Track Points')
    # Set equal aspect ratio
    ax.set_aspect('equal', adjustable='box')
    # Show the grid
    ax.grid(True)
    # Show the plot
    plt.show()
       





if __name__ == "__main__":
    main()