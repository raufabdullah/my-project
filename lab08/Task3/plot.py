import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyACM0', 115200)
plt.ion()

tempValues = []
x_dps_values = []
y_dps_values = []
z_dps_values = []
time_s = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    
    # Create 2x2 subplot layout
    plt.subplot(2, 2, 1)
    plt.title('Temperature')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.plot(time_s, tempValues, 'r.-', label='Temp')
    plt.legend(loc='upper left')
    
    plt.subplot(2, 2, 2)
    plt.title('X-Axis Gyroscope')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (dps)')
    plt.plot(time_s, x_dps_values, 'b.-', label='X-axis')
    plt.legend(loc='upper left')
    
    plt.subplot(2, 2, 3)
    plt.title('Y-Axis Gyroscope')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (dps)')
    plt.plot(time_s, y_dps_values, 'g.-', label='Y-axis')
    plt.legend(loc='upper left')
    
    plt.subplot(2, 2, 4)
    plt.title('Z-Axis Gyroscope')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (dps)')
    plt.plot(time_s, z_dps_values, 'm.-', label='Z-axis')
    plt.legend(loc='upper left')
    
    plt.tight_layout()

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass

    try:
        line = sinWaveData.readline().decode().strip()
        
        if line:
            # Parse CSV format: <temp>,<x_dps>,<y_dps>,<z_dps>
            values = line.split(',')
            if len(values) == 4:
                temp = float(values[0])
                x_dps = float(values[1])
                y_dps = float(values[2])
                z_dps = float(values[3])
                
                tempValues.append(temp)
                x_dps_values.append(x_dps)
                y_dps_values.append(y_dps)
                z_dps_values.append(z_dps)
                time_s.append(cnt * 0.1)
                cnt += 1

                drawnow(makeFig)
                plt.pause(0.0001)

                # Keep only last 100 samples
                if len(tempValues) > 100:
                    tempValues.pop(0)
                    x_dps_values.pop(0)
                    y_dps_values.pop(0)
                    z_dps_values.pop(0)
                    time_s.pop(0)

    except Exception as e:
        print("Error:", e)
