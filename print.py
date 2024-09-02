import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure the serial port (adjust COM port and baud rate as necessary)
ser = serial.Serial('COM7', 115200)

# Initialize lists to store data
x_data = []
y_data = []
z_data = []
time_data = []

# Initialize the plot
fig, ax = plt.subplots()
ax.set_title("Real-time QMC5883L Sensor Data")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Sensor Values")
line_x, = ax.plot([], [], label='X')
line_y, = ax.plot([], [], label='Y')
line_z, = ax.plot([], [], label='Z')
ax.legend()

start_time = time.time()

def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(-100000, 100000)  # Adjust y-axis limits as needed
    return line_x, line_y, line_z

def update(frame):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                # Assuming the format "x=64754.000000,y=57814.000000,z=63711.000000,temp=4.200000"
                parts = line.split(',')
                x = float(parts[0].split('=')[1])
                y = float(parts[1].split('=')[1])
                z = float(parts[2].split('=')[1])
                current_time = time.time() - start_time

                x_data.append(x)
                y_data.append(y)
                z_data.append(z)
                time_data.append(current_time)

                # Update plot limits if necessary
                if current_time > ax.get_xlim()[1]:
                    ax.set_xlim(0, current_time + 5)

                line_x.set_data(time_data, x_data)
                line_y.set_data(time_data, y_data)
                line_z.set_data(time_data, z_data)
            except Exception as e:
                print(f"Error parsing line: {line}")
                print(e)

    return line_x, line_y, line_z

ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=10)

try:
    plt.show()
except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
