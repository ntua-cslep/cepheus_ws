import matplotlib.pyplot as plt
import numpy as np

x, y = np.loadtxt('vel.txt', delimiter=' ', unpack=True)
plt.plot(x,y)
plt.show()
