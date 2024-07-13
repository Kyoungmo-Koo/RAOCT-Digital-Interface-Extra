import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

file_length = 5020

def read_csv(file_path):
    with open(file_path, newline='') as csvfile:
        # Create a CSV reader object
        csv_reader = csv.reader(csvfile)
        # Read the CSV data into a 2D list (buffer)
        buffer = [[int(item) for item in row[0:2 * file_length]] for row in csv_reader]
    return buffer

csv_file = "error_data.csv"
data = read_csv(csv_file)

data_x = [[row[i] for i in range(len(row)) if i % 2 == 0] for row in data]
data_y = [[row[i] for i in range(len(row)) if i % 2 != 0] for row in data]

import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 10, 5020) 

# Plot the buffers
plt.figure(figsize=(10, 6))
plt.plot(x, data_x[2], label='Buffer 1')
plt.plot(x, data_y[2], label='Buffer 2')
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Two One-Dimensional Buffers')
plt.legend()
plt.grid(True)
plt.show()

# count = 0
# for k in range(48):
#     for i in range(5020):
#         count = count + (data[k][i] == data[k + 1][i])
#     print(count)
#     count = 0

# count = 0
# for k in range(49):
#     for i in range(5020):
#         count = count + (data[k][i] == data[k + 1][i])
# print(count)

# count = 0
# for k in range(49):
#     for i in range(5020):
#         count = count + (data[k][i] < 1000)
# print(count)

k = 31
list = []
for i in range(5020):
    if(data_y[k][i] == data_y[k+1][i]):
        list.append(i)

# Divide 5020 by 50 -> 10. It is because it is 50kHz. (Time domain is 10 ms)
x = np.linspace(0, 10, 5020) 


# Plot the buffers
plt.figure(figsize=(10, 6))
plt.plot(x, data_y[k], label='Buffer 1')
plt.plot(x, data_y[k+1], label='Buffer 2')
plt.scatter([x[i] for i in list], [data_y[k][i] for i in list], color='red', label='Points where Buffer 1 = Buffer 2')
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Two One-Dimensional Buffers')
plt.legend()
plt.grid(True)
plt.show()

