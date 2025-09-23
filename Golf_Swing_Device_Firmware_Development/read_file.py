import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Read file from ESP32 ---
esp32 = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=2)
esp32.write(b"READ_FILE\r\n")

file_content = ""
while True:
    line = esp32.readline().decode("utf-8", errors="ignore").strip()
    if line == "EOF":
        break
    # Skip ESP-IDF logs if any
    if line.startswith("I (") or line.startswith("E ("):
        continue
    file_content += line + "\n"

esp32.close()

# --- Parse CSV ---
lines = file_content.strip().split("\n")
readings = lines[1:]  # skip header

ax, ay, az, gx, gy, gz, vib = [], [], [], [], [], [], []

for r in readings:
    r = r.strip().split(",")
    if len(r) == 7:
        ax.append(float(r[0]))
        ay.append(float(r[1]))
        az.append(float(r[2]))
        gx.append(float(r[3]))
        gy.append(float(r[4]))
        gz.append(float(r[5]))
        vib.append(float(r[6]))
        
        
print(ax, ay, az, gx, gy, gz, vib)

# --- Plotting ---
fig = plt.figure(figsize=(15, 10))

# Accelerometer subplot
ax1 = fig.add_subplot(131, projection='3d')
ax1.plot(ax, ay, az, label="Accelerometer")
ax1.scatter(ax[0], ay[0], az[0], color="green", s=50, label="Start")
ax1.scatter(ax[-1], ay[-1], az[-1], color="red", s=50, label="End")
ax1.set_xlabel("ax")
ax1.set_ylabel("ay")
ax1.set_zlabel("az")
ax1.set_title("Accelerometer (3D)")
ax1.legend()

# Gyroscope subplot
ax2 = fig.add_subplot(132, projection='3d')
ax2.plot(gx, gy, gz, label="Gyroscope", color="orange")
ax2.scatter(gx[0], gy[0], gz[0], color="green", s=50, label="Start")
ax2.scatter(gx[-1], gy[-1], gz[-1], color="red", s=50, label="End")
ax2.set_xlabel("gx")
ax2.set_ylabel("gy")
ax2.set_zlabel("gz")
ax2.set_title("Gyroscope (3D)")
ax2.legend()

# Vibration subplot
ax3 = fig.add_subplot(133, projection='3d')
samples = range(len(vib))
ax3.plot(samples, [0]*len(vib), vib, label="Vibration", color="green")
ax3.scatter(samples[0], 0, vib[0], color="green", s=50, label="Start")
ax3.scatter(samples[-1], 0, vib[-1], color="red", s=50, label="End")
ax3.set_xlabel("Sample")
ax3.set_ylabel("0 (dummy axis)")
ax3.set_zlabel("Vibration")
ax3.set_title("Vibration (3D)")
ax3.legend()

plt.tight_layout()
plt.show()
