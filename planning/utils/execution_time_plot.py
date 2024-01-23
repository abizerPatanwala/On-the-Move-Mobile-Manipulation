import matplotlib.pyplot as plt
import numpy as np

# Assuming you have a file with x, y, and cost columns
# Replace 'your_data_file.txt' with the actual file path


# Extracting x, y, and cost columns
distDiff = [0.125, 0.188, 0.239,  0.345, 0.482 , 0.85, 1.87, 2.12, 2.45, 2.56]
timeImprovement = [14, 14, 14, 13, 12, 11, 8 , 7 , 6 , 6]

# Create a scatter plot with colors representing the cost
plt.plot(distDiff, timeImprovement)


# Add labels and title
plt.xlabel('Path traversal difference b/w static and OTM manipulation')
plt.ylabel('Execution Time Improvement')
plt.title('Distance difference vs Time Improvement')

# Show the plot
plt.show()
