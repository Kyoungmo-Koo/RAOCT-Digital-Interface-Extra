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

def read_txt(file_path):
    with open(file_path, 'r') as file:
        integers = []
        for line in file:
            # Remove newline and other potential trailing characters
            clean_line = line.strip()
            # Split the line into two parts and convert each to an integer
            first_part = int(clean_line[0:4], 16)
            second_part = int(clean_line[4:8], 16)
            # Add the converted integers to the list
            integers.extend([first_part, second_part])
        return integers

def subtract_lists(Actual_data, Target_data):
    result = []
    for row in Actual_data :
        # Subtract the corresponding element in list_1d from each element in the row
        result_row = [row_element - Target_data[i] for i, row_element in enumerate(row)]
        result.append(result_row)
    return result

def calculate_rms(buffer):
    rms_values = []
    for row in buffer:
        # Calculate the mean of the squares of the elements in the row
        mean_of_squares = sum(x**2 for x in row) / len(row)
        # Take the square root of the mean of squares to get the RMS
        rms = math.sqrt(mean_of_squares)
        rms_values.append(rms)
    return rms_values

def calculate_mean_abs(buffer):
    mean_abs_values = []
    for row in buffer:
        # Calculate the mean of the absolute values of the elements in the row
        mean_abs = sum(abs(x) for x in row) / len(row)
        mean_abs_values.append(mean_abs)
    return mean_abs_values

def calculate_max_abs(buffer):
    max_abs_values = []
    for row in buffer:
        # Find the maximum absolute value in the row
        max_abs = max(abs(x) for x in row)
        max_abs_values.append(max_abs)
    return max_abs_values


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

count = 0
for k in range(48):
    for i in range(5020):
        count = count + (data[k][i] == data[k + 1][i])
    print(count)
    count = 0

count = 0
for k in range(49):
    for i in range(5020):
        count = count + (data[k][i] == data[k + 1][i])
print(count)

count = 0
for k in range(49):
    for i in range(5020):
        count = count + (data[k][i] < 1000)
print(count)

k = 31
list = []
for i in range(5020):
    if(data_y[k][i] == data_y[k+1][i]):
        list.append(i)

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

