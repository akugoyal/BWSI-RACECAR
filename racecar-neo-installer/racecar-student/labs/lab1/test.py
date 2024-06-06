import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import curve_fit

def inv_function(x, a, b, c):
    return a/(x + b) + c

x_data = [1, 2, 3, 4, 5, 10, 15, 20]
y_data = [535, 740, 850, 920, 975, 1090, 1135, 1160]

x = np.linspace(0.9, 25, 3000)

params, covariance = curve_fit(inv_function, x_data, y_data)
print(f"Parameters: {params} || Covariance: {covariance}")
a_fit, b_fit, c_fit = params

print(params[0])
print(params[1])
print(params[2])
y = a_fit/(x + b_fit) + c_fit

plt.plot(x, y)
plt.scatter(x_data, y_data)
plt.show()

print(inv_function(3.4, a_fit, b_fit, c_fit))