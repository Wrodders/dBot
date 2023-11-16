import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

import random

def generate_sensor_data(min_value, max_value):
    """
    Generate random sensor data as a list of floats.

    Parameters:
    - num_samples (int): Number of data points to generate.
    - min_value (float): Minimum value for the generated data.
    - max_value (float): Maximum value for the generated data.

    Returns:
    - List of randomly generated floats.
    """
    sensor_data = random.uniform(min_value, max_value) 
    return sensor_data

# Example usage:

min_value = 0.0
max_value = 100.0





# Parameters
x_len = 200         # Number of points to display
y_range = [40, 110]  # Range of possible Y values to display
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys = [0] * x_len
ax.set_ylim(y_range)
# Initialize communication with TMP102

# Create a blank line. We will update the line in animate
line, = ax.plot(xs, ys)
# Add labels
plt.title('TMP102 Temperature over Time')
plt.xlabel('Samples')
plt.ylabel('Temperature (deg F)')
# This function is called periodically from FuncAnimation
def animate(i, ys):
    # Read temperature (Celsius) from TMP102
    data = generate_sensor_data(min_value, max_value)
    temp_c = round(data,2)
    temp_f=(temp_c*9/5)+32
    # Add y to list
    ys.append(temp_f)
    # Limit y list to set number of items
    ys = ys[-x_len:]
    # Update line with new Y values
    line.set_ydata(ys)
    return line,
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
    animate,
    fargs=(ys,),
    interval=50,
    blit=True)
plt.show()