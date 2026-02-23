import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyACM0', 115200)
plt.ion()

tempValues = []
time_s = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Live Temperature Data')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature')
    plt.ylim(0, 24)
    plt.plot(time_s, tempValues, 'r.-', label='Temperature')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass

    try:
        line = sinWaveData.readline().decode().strip()
        
        if line:
            temp = float(line)
            
            tempValues.append(temp)
            time_s.append(cnt * 0.1)
            cnt += 1

            drawnow(makeFig)
            plt.pause(0.0001)

            # Keep only last 100 samples
            if len(tempValues) > 100:
                tempValues.pop(0)
                time_s.pop(0)

    except Exception as e:
        print("Error:", e)
