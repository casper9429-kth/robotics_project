import matplotlib.pyplot as plt
import numpy as np

# create a random 2D array of data
data = np.random.rand(10, 10)
print(data.shape)
print(data[1])
# plot the data using imshow
plt.imshow(data, cmap='gray')

# show the plot
plt.show()
