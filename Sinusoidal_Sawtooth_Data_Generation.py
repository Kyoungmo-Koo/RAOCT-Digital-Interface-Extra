import math

MAX_POS = 65535;
ARRAY_SIZE = 5000;

def pack_position_data(x_pos, y_pos):
    packed_data = ((x_pos & 0xFFFF) << 16) | (y_pos & 0xFFFF)
    return format(packed_data, '08X')

def generate_sine_position_data(array_size):
    position_data_array = []
    for i in range(array_size):
        x = int(MAX_POS * math.sin(1.0 * math.pi * i / array_size)) & 0xFFFF
        y = int(MAX_POS * math.sin(1.0 * math.pi * i / array_size)) & 0xFFFF
        position_data_array.append(pack_position_data(x, y))
    return position_data_array

def generate_step_position_data(step, array_size):
    position_data_array = []
    for i in range(1, array_size + 1):
        x = step * i
        y = step * i
        position_data_array.append(pack_position_data(x, y))
    return position_data_array

sine_position_data = generate_sine_position_data(ARRAY_SIZE)
step_position_data = generate_step_position_data(6, ARRAY_SIZE)

with open('sine_position_data.txt', 'w') as file:
    sine_position_data_hex = generate_sine_position_data(ARRAY_SIZE)
    for position_data in sine_position_data_hex:
        file.write(position_data + '\n')

# Generate step position data and write to file
with open('step_position_data.txt', 'w') as file:
    step_position_data_hex = generate_step_position_data(6, ARRAY_SIZE)
    for position_data in step_position_data_hex:
        file.write(position_data + '\n')
