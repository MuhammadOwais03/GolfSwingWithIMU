


import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, integrate
from scipy.spatial.transform import Rotation as R
import serial
import time
import pandas as pd
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

class GolfIMU:
    def __init__(self):
        self.PORT = '/dev/ttyUSB0'  # Change to your serial port
        self.BAUD = 115200
        self.TIMEOUT = 2
        self.CALIBRATION_SAMPLES = 20
        self.ACCEL_IN_MG = True
        self.GRAVITY = 9.81
        self.ser = None
        self.offsets_acc = np.zeros(3)
        self.offsets_gyro = np.zeros(3)

    def connect(self):
        try:
            self.ser = serial.Serial(self.PORT, self.BAUD, timeout=self.TIMEOUT)
            print(f"Connected to IMU at {self.PORT} ({self.BAUD} baud)")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            raise

    def read_data(self):
        print("1: Sending READ_FILE command")
        self.ser.write(b"READ_FILE\r\n")
        file_content = ""
        count = 0
        while True:
            
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            print(f"{count}: ", line)
            count += 1
            if line == "EOF":
                break
            if line.startswith("I (") or line.startswith("E ("):
                continue
            file_content += line + "\n"

        
        lines = file_content.strip().split("\n")
        
        readings = lines[1:]  # Skip header
        

        accel_list = []
        gyro_list = []
        vib = []

        for r in readings:
            r = r.strip().split(",")
            
            if len(r) == 7:
                try:
                    ax = float(r[1]) * -1
                    ay = float(r[0]) * -1
                    az = float(r[2])
                    gx = float(r[4]) * -1
                    gy = float(r[3]) * -1
                    gz = float(r[5])
                    v = float(r[6])
                    accel_list.append([ax, ay, az])
                    gyro_list.append([gx, gy, gz])
                    vib.append(v)
                except ValueError as e:
                    print(f"Error parsing line {r}: {e}")
                    continue

        # print("Accel list:", accel_list)
        # print("Gyro list:", gyro_list)
        # print("Vibration list:", vib)

        if not accel_list:
            print("Warning: No valid accelerometer data collected.")
            return np.array([]), np.array([])

        accel = np.array(accel_list) - self.offsets_acc
        gyro = np.array(gyro_list) - self.offsets_gyro
        return accel, gyro

    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial connection closed")

def animate_accel(csv_file="accelfilt.csv", out_file="accelfilt.gif"):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Ensure data is saved correctly.")
        return

    t = df["time"].values
    ax = df["Ax_filt"].values
    ay = df["Ay_filt"].values
    az = df["Az_filt"].values

    az = (az - 9.8) * -1
    ay *= -1

    fig = plt.figure(figsize=(8, 6))
    ax3d = fig.add_subplot(111, projection="3d")

    line, = ax3d.plot([], [], [], "r-", lw=2, label="Filtered Accel")
    point, = ax3d.plot([], [], [], "ko")

    ax3d.set_xlim([min(ax), max(ax)])
    ax3d.set_ylim([min(ay), max(ay)])
    ax3d.set_zlim([min(az), max(az)])
    ax3d.set_xlabel("Ax (m/s²)")
    ax3d.set_ylabel("Ay (m/s²)")
    ax3d.set_zlabel("Az (m/s²)")
    ax3d.set_title("3D Filtered Acceleration Trajectory")
    ax3d.legend()

    def update(i):
        line.set_data(ax[:i], ay[:i])
        line.set_3d_properties(az[:i])
        point.set_data([ax[i]], [ay[i]])
        point.set_3d_properties([az[i]])
        return line, point

    ani = FuncAnimation(fig, update, frames=len(t), interval=30, blit=True)
    ani.save(out_file, writer="pillow")
    print(f"Animation saved as {out_file}")
    plt.show()

has_run = False
if __name__ == "__main__":
    if has_run:
        print("Script already running, exiting to prevent re-entry.")
        exit(0)
    has_run = True
    imu = GolfIMU()
    imu.connect()

    fs = 250  # Hz
    print("Collecting IMU data... Perform the golf swing.")
    data = imu.read_data()
    if data and len(data[0]) > 12:  # Ensure enough samples for filtering
        accel, gyro = data
        t = np.linspace(0, len(accel)/fs, len(accel))
    else:
        print("Error: Insufficient or no data collected from IMU.")
        imu.close()
        exit(1)

    imu.close()
    print(f"Collected {len(accel)} samples.")

    # Convert to numpy arrays directly, avoiding extra dimension
    accel_arr = np.array(accel)
    gyro_arr = np.array(gyro)
    t_arr = np.array(t)
    print("accel_arr shape:", accel_arr.shape)  # Should be (16, 3)
    print("gyro_arr shape:", gyro_arr.shape)    # Should be (16, 3)
    print("t_arr shape:", t_arr.shape)          # Should be (16,)

    if len(accel_arr) <= 12:
        print("Error: Not enough samples for filtering.")
        exit(1)

    dt = np.mean(np.diff(t_arr))

    # Low-pass filter
    b, a = signal.butter(3, 20 / (fs / 2), btype='low')
    ax_filt = signal.filtfilt(b, a, accel_arr[:, 0], padlen=min(3, len(accel_arr)-1))
    ay_filt = signal.filtfilt(b, a, accel_arr[:, 1], padlen=min(3, len(accel_arr)-1))
    az_filt = signal.filtfilt(b, a, accel_arr[:, 2], padlen=min(3, len(accel_arr)-1))
    gx_filt = signal.filtfilt(b, a, gyro_arr[:, 0], padlen=min(3, len(gyro_arr)-1))
    gy_filt = signal.filtfilt(b, a, gyro_arr[:, 1], padlen=min(3, len(gyro_arr)-1))
    gz_filt = signal.filtfilt(b, a, gyro_arr[:, 2], padlen=min(3, len(gyro_arr)-1))

    # Quaternion integration
    q = np.zeros((len(t_arr), 4))
    q[0] = [1, 0, 0, 0]
    for i in range(1, len(t_arr)):
        omega = [gx_filt[i], gy_filt[i], gz_filt[i]]
        q_dot = 0.5 * np.array([0.0, *omega])
        q[i] = q[i - 1] + q_dot * dt
        q[i] /= np.linalg.norm(q[i])

    # Rotation matrices & Euler angles
    euler_deg = np.zeros((len(t_arr), 3))
    R_mats = np.zeros((len(t_arr), 3, 3))
    for i in range(len(t_arr)):
        w, x, y, z = q[i]
        quat_scipy = np.array([x, y, z, w])
        rot = R.from_quat(quat_scipy)
        R_mats[i] = rot.as_matrix()
        euler_deg[i] = rot.as_euler("zyx", degrees=True)

    # Rotate accel into global frame
    accel_global = np.zeros_like(accel_arr)
    for i in range(len(t_arr)):
        rot = R.from_matrix(R_mats[i])
        accel_global[i] = rot.apply([ax_filt[i], ay_filt[i], az_filt[i]])
    accel_global[:, 2] -= imu.GRAVITY

    # Stationary detection
    acc_mag = np.linalg.norm(accel_arr, axis=1)
    b, a = signal.butter(1, 0.001 / (fs / 2), "high")
    acc_mag_filt = signal.filtfilt(b, a, acc_mag, padlen=min(3, len(acc_mag)-1))
    acc_mag_filt = np.abs(acc_mag_filt)
    b, a = signal.butter(1, 5 / (fs / 2), "low")
    acc_mag_filt = signal.filtfilt(b, a, acc_mag_filt, padlen=min(3, len(acc_mag_filt)-1))
    stationary = acc_mag_filt < 0.06

    # Velocity with ZUPT
    vel_global = np.zeros_like(accel_global)
    for i in range(1, len(t_arr)):
        vel_global[i] = vel_global[i - 1] + accel_global[i] * dt
        if stationary[i]:
            vel_global[i] = [0, 0, 0]

    # Drift correction
    vel_drift = np.zeros_like(vel_global)
    s_start = np.where(np.diff(stationary.astype(int)) == -1)[0]
    s_end = np.where(np.diff(stationary.astype(int)) == 1)[0]
    for j in range(min(len(s_start), len(s_end))):
        s_idx, e_idx = s_start[j], s_end[j]
        if e_idx - s_idx <= 0:
            continue
        drift_rate = vel_global[e_idx - 1] / float(e_idx - s_idx)
        ramp = np.arange(e_idx - s_idx).reshape(-1, 1) * drift_rate
        vel_drift[s_idx:e_idx] = ramp
    vel_global -= vel_drift

    # Position
    pos_global = integrate.cumulative_trapezoid(vel_global, t_arr, axis=0, initial=0)

    # Save CSVs
    df_raw = pd.DataFrame({
        "time": t_arr,
        "Ax": accel_arr[:, 0], "Ay": accel_arr[:, 1], "Az": accel_arr[:, 2],
        "Gx": gyro_arr[:, 0], "Gy": gyro_arr[:, 1], "Gz": gyro_arr[:, 2]
    })
    df_raw.to_csv("raw.csv", index=False)

    df_pos = pd.DataFrame({
        "time": t_arr,
        "velX": vel_global[:, 0], "velY": vel_global[:, 1], "velZ": vel_global[:, 2],
        "posX": pos_global[:, 0], "posY": pos_global[:, 1], "posZ": pos_global[:, 2],
        "yaw_deg": euler_deg[:, 0], "pitch_deg": euler_deg[:, 1], "roll_deg": euler_deg[:, 2]
    })
    df_pos.to_csv("pos.csv", index=False)

    df_accel_filt = pd.DataFrame({
        "time": t_arr,
        "Ax_filt": ax_filt,
        "Ay_filt": ay_filt,
        "Az_filt": az_filt
    })
    df_accel_filt.to_csv("accelfilt.csv", index=False)

    # Plots
    fig = plt.figure(figsize=(16, 12))

    # 1: Raw Acceleration
    ax1 = fig.add_subplot(331)
    ax1.plot(t_arr, accel_arr[:, 0], label="Ax")
    ax1.plot(t_arr, accel_arr[:, 1], label="Ay")
    ax1.plot(t_arr, accel_arr[:, 2], label="Az")
    ax1.set_title("Raw Acceleration (m/s²)")
    ax1.legend()
    ax1.grid(True)

    # 2: Raw Gyroscope
    ax2 = fig.add_subplot(332)
    ax2.plot(t_arr, gyro_arr[:, 0], label="Gx")
    ax2.plot(t_arr, gyro_arr[:, 1], label="Gy")
    ax2.plot(t_arr, gyro_arr[:, 2], label="Gz")
    ax2.set_title("Raw Gyroscope (rad/s)")
    ax2.legend()
    ax2.grid(True)

    # 3: Filtered Acceleration
    ax3 = fig.add_subplot(333)
    ax3.plot(t_arr, ax_filt, label="Ax")
    ax3.plot(t_arr, ay_filt, label="Ay")
    ax3.plot(t_arr, az_filt, label="Az")
    ax3.set_title("Filtered Acceleration (m/s²)")
    ax3.legend()
    ax3.grid(True)

    # 4: Filtered Gyroscope
    ax4 = fig.add_subplot(334)
    ax4.plot(t_arr, gx_filt, label="Gx")
    ax4.plot(t_arr, gy_filt, label="Gy")
    ax4.plot(t_arr, gz_filt, label="Gz")
    ax4.set_title("Filtered Gyroscope (rad/s)")
    ax4.legend()
    ax4.grid(True)

    # 5: Velocity vs Time
    ax5 = fig.add_subplot(335)
    ax5.plot(t_arr, vel_global[:, 0], label="Vx")
    ax5.plot(t_arr, vel_global[:, 1], label="Vy")
    ax5.plot(t_arr, vel_global[:, 2], label="Vz")
    ax5.set_title("Velocity vs Time")
    ax5.legend()
    ax5.grid(True)

    # 6: Position vs Time
    ax6 = fig.add_subplot(336)
    ax6.plot(t_arr, pos_global[:, 0], label="X")
    ax6.plot(t_arr, pos_global[:, 1], label="Y")
    ax6.plot(t_arr, pos_global[:, 2], label="Z")
    ax6.set_title("Position vs Time")
    ax6.legend()
    ax6.grid(True)

    # 7: Euler Angles
    ax7 = fig.add_subplot(337)
    ax7.plot(t_arr, euler_deg[:, 0], label="Yaw")
    ax7.plot(t_arr, euler_deg[:, 1], label="Pitch")
    ax7.plot(t_arr, euler_deg[:, 2], label="Roll")
    ax7.set_title("Euler Angles vs Time")
    ax7.legend()
    ax7.grid(True)

    # 8: 3D Trajectory
    ax8 = fig.add_subplot(338, projection="3d")
    ax8.plot(pos_global[:, 0], pos_global[:, 1], pos_global[:, 2], "b-")
    ax8.set_title("3D Displacement")
    ax8.set_xlabel("X (m)")
    ax8.set_ylabel("Y (m)")
    ax8.set_zlabel("Z (m)")

    # Extra subplot: 3D Acceleration (filtered)
    fig2 = plt.figure(figsize=(8, 6))
    ax9 = fig2.add_subplot(111, projection="3d")
    ax9.plot(ax_filt, -1*ay_filt, -1*(az_filt-9.8), "r-")
    ax9.set_title("3D Acceleration (Filtered)")
    ax9.set_xlabel("Ax (m/s²)")
    ax9.set_ylabel("Ay (m/s²)")
    ax9.set_zlabel("Az (m/s²)")
    ax9.grid(True)

    plt.tight_layout()
    plt.show()
    animate_accel("accelfilt.csv", "accelfilt.gif")