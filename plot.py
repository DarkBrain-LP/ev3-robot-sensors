import pandas as pd
import matplotlib.pyplot as plt

base_adress = 'C:/Users/ManU_LP/Downloads/YOVO_Yawo/1ev3-robot-sensors'

def plot_and_save(filename):
    # Initialize empty lists to store x and y coordinates inside the function
    x_values = []
    y_values = []
    
    # Open the file
    with open(f'{base_adress}/{filename}.csv', 'r') as file:
        # Skip the first line
        next(file)
        # Iterate through each line in the file
        for line in file:
            # Split the line by comma and convert to float
            parts = line.strip().split(',')
            x = float(parts[0])
            y = float(parts[1])
            # Append x and y coordinates to the lists
            x_values.append(x)
            y_values.append(y)
    
    print(f"First 10 x values for {filename}: {x_values[:10]}")
    print(f"First 10 y values for {filename}: {y_values[:10]}")

    # Add x and y of x_values and y_values to a new csv file
    with open(f'{filename}_output.csv', 'w') as file:
        # Write the header
        file.write('x,y\n')
        # Write the x and y coordinates
        for x, y in zip(x_values, y_values):
            file.write(f'{x},{y}\n')
    
    # Plot the x and y coordinates
    plt.figure(figsize=(8, 6))
    plt.plot(x_values, y_values, marker='o', linestyle='-')
    plt.title(f'Plot of x and y coordinates for {filename}')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)

# Call the function for each file
plot_and_save('_1pid_log')
plot_and_save('_1klm_kalman_log')
plot_and_save('_1klm_gyro_logs')

# Show all plots
plt.show()
