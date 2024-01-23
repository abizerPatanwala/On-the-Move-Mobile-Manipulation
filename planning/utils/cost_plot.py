import matplotlib.pyplot as plt
import numpy as np

# Assuming you have a file with x, y, and cost columns
# Replace 'your_data_file.txt' with the actual file path
data = np.loadtxt('/home/abizer10/MobileUR5e/cost.csv', delimiter=',')

# Extracting x, y, and cost columns
x = data[:, 0]
y = data[:, 1]
cost = data[:, 2]

# Create a scatter plot with colors representing the cost
plt.scatter(x, y, c=cost, cmap='viridis', marker='o')

# Add a colorbar to the plot
plt.colorbar(label='Cost')

# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('2D Plot with Cost as Color')

# Show the plot
plt.show()
