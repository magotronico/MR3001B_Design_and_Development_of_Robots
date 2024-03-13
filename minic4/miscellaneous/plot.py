import matplotlib.pyplot as plt

# Read the lines from the output.txt file
with open('c:/Users/dilan/Downloads/output2.txt', 'r') as file:
    lines2 = file.readlines()
with open('c:/Users/dilan/Downloads/output.txt', 'r') as file:
    lines = file.readlines()   

# Prepare the x-axis values for output2.txt
x_values1 = [i / 100 for i in range(1001)]

# Read the y-values from the lines of output2.txt
y_values1 = [float(line.strip()) for line in lines2]

# Create a new figure for output2.txt
plt.figure(1)

# Plot the line for output2.txt
plt.plot(x_values1, y_values1)

# Add labels for output2.txt
plt.title('angular_z vs w_output')
plt.xlabel('angular_z')
plt.ylabel('w_output')

# Set the x-axis limits for output2.txt
plt.xlim(0, 10)

# Prepare the x-axis values for output.txt
x_values2 = [i / 100 for i in range(101)]

# Read the y-values from the lines of output.txt
y_values2 = [float(line.strip()) for line in lines]

# Create a new figure for output.txt
plt.figure(2)

# Plot the line for output.txt
plt.plot(x_values2, y_values2)

# Add labels for output.txt
plt.title('linear_x vs v_output')
plt.xlabel('linear_X')
plt.ylabel('v_output')

# Set the x-axis limits for output.txt
plt.xlim(0,1)

# Show both plots in the same window
plt.show()
