import math
import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np

def statCalc(data: list) -> list:
    sum = 0.0
    for i in data:
        sum += i

    avg = sum / len(data)

    variance = 0.0
    for i in data:
        variance += (i - avg)**2
    variance /= len(data)

    std = math.sqrt(variance)
    return [avg, variance, std]

def func(x):
    avg = 4.996639999999999
    variance = 0.008183150400000007
    exp = (-(x - avg)**2) / (2 * variance)
    denom = math.sqrt(2 * math.pi * variance)
    return (1 / denom) * math.exp(exp)

def myFunc(x):
    if x >= 0 and x <= 0.25:
        return -12.8 * x + 3.4
    elif (x > 0.25 and x <= 0.75):
        return 0.2
    elif x > 0.75 and x <= 1:
        return 12.8 * x - 9.4
    else:
        return 0.0

data = [4.546, 4.5, 4.567, 4.508, 4.602, 4.504, 4.666, 4.662, 4.624, 4.518, 4.743, 4.748, 4.775, 4.627, 4.642, 4.747, 4.566, 4.551, 4.536, 4.622, 4.676, 4.557, 4.733, 4.532, 4.667, 4.732, 4.634, 4.7, 4.764, 4.602, 4.644, 4.77, 4.624, 4.717, 4.538, 4.502, 4.798, 4.772, 4.537, 4.586, 4.543, 4.563, 4.675, 4.713, 4.509, 4.678, 4.729, 4.782, 4.722, 4.579]
bias = 0.36
data_biased = []
for x in data:
    data_biased.append(x + bias)
stats = statCalc(data_biased)
print(stats)
# print(len(data))
y = []
# t = []
for x in range(len(data_biased)):
    y.append(func(data_biased[x]))
    # t.append(x * 0.1)

print(integrate.quad(myFunc, 0.95, 1))


plt.scatter(data_biased, y)
# plt.show()