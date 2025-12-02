#!/usr/bin/python3 
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import csv
from scipy.interpolate import CubicSpline

# Load the map image
map_image_path = '/home/hopper/github/thing2/src/F1tenth_car_workspace/base_mapping/map/base_map.pgm'
map_image = Image.open(map_image_path)
map_array = np.array(map_image)

# Map metadata from the YAML file
origin_x, origin_y = -4.4, -1.648101  # Origin of the map
resolution = 0.050000  # Resolution of the map

# Function to convert image coordinates to map coordinates
def image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution):
    x_map = x_img * resolution + origin_x
    y_map = (map_array.shape[0] - y_img) * resolution + origin_y
    return x_map, y_map

# Initialize lists to store clicked points
x_clicked = []
y_clicked = []

# Function to handle mouse click events
def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x_clicked.append(event.xdata)
        y_clicked.append(event.ydata)
        ax.plot(event.xdata, event.ydata, 'ro')  # Mark the clicked point with a red dot
        fig.canvas.draw()  # Update the figure to show the new point

# Function to save the clicked points to a CSV file, adjusted for map coordinates
def save_race_line(filename='race_line.csv'):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for x_img, y_img in zip(x_clicked, y_clicked):
            x_map, y_map = image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution)
            writer.writerow([x_map, y_map, 0.0, 1.0])  # Assigning a constant speed of 1.0 for demonstration
    print(f"Race line saved to {filename}.")

# Function to generate Bezier curve or spline interpolation between clicked points
def generate_smooth_curve(x_points, y_points):
    # Use cubic spline interpolation for smooth curves
    if len(x_points) < 4:
        return np.array([]), np.array([])  # We need at least 4 points for smooth interpolation

    # Interpolate the x and y coordinates independently
    cs_x = CubicSpline(range(len(x_points)), x_points)
    cs_y = CubicSpline(range(len(y_points)), y_points)
    
    # Generate 100 points along the spline
    fine_range = np.linspace(0, len(x_points) - 1, 100)
    smooth_x = cs_x(fine_range)
    smooth_y = cs_y(fine_range)
    
    return smooth_x, smooth_y

# Set up the plot
fig, ax = plt.subplots()
ax.imshow(map_array, cmap='gray')
ax.set_title('Click to draw the race line')
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()

# Save the race line after the plot window is closed
save_race_line('demoline.csv')

# After collecting the clicked points, generate the smooth curve
if len(x_clicked) > 1:
    smooth_x, smooth_y = generate_smooth_curve(x_clicked, y_clicked)
    
    if smooth_x.size > 0:
        # Save the generated smooth points to the CSV file
        with open('demoline.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            for x, y in zip(smooth_x, smooth_y):
                x_map, y_map = image_to_map_coordinates(x, y, origin_x, origin_y, resolution)
                writer.writerow([x_map, y_map, 0.0, 1.0])  # Assigning a constant speed of 1.0 for demonstration

        print("Smooth race line saved to demoline.csv.")

        # Optionally, plot the smooth curve
        fig, ax = plt.subplots()
        ax.imshow(map_array, cmap='gray')
        ax.plot(smooth_x, smooth_y, 'b-', label='Smooth Path')
        ax.legend()
        ax.set_title('Generated Smooth Race Line')
        plt.show()
