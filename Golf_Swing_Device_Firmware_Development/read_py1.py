
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, integrate
from scipy.spatial.transform import Rotation as R
import serial
import time
import pandas as pd
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import os 

# GolfIMU class remains unchanged
class GolfIMU:
    def __init__(self):
        self.PORT = 'COM3'  # Change to your serial port
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
        vib_list = []

        for r in readings:
            r = r.strip().split(",")
            if len(r) == 7:
                try:
                    ax = float(r[1]) * -1.0
                    ay = float(r[0]) * -1.0
                    az = (float(r[2])  - 20.0)*-1.0
                    gx = float(r[3]) 
                    gy = float(r[4]) 
                    gz = float(r[5]) 
                    v = float(r[6])
                  
                    accel_list.append([ax, ay, az])
                    gyro_list.append([gx, gy, gz])
                    vib_list.append(v)
                except ValueError as e:
                    print(f"Error parsing line {r}: {e}")
                    continue

        if not accel_list:
            print("Warning: No valid accelerometer data collected.")
            return np.array([]), np.array([])

        accel = np.array(accel_list) - self.offsets_acc
        gyro = np.array(gyro_list) - self.offsets_gyro
        vib = np.array(vib_list) 
        return accel, gyro, vib

    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial connection closed")

def load_raw_csv(file_path="raw.csv"):
    if not os.path.exists(file_path):
        return None, None, None

    df = pd.read_csv(file_path)
    accel_arr = df[["Ax", "Ay", "Az"]].values
    gyro_arr = df[["Gx", "Gy", "Gz"]].values
    vib_arr = df["Vib"].values if "Vib" in df.columns else np.zeros(len(accel_arr))
    t_arr = df["time"].values
    return accel_arr, gyro_arr, vib_arr, t_arr

def animate_accel(csv_file="accelfilt.csv", out_file="accelfilt.gif"):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Ensure data is saved correctly.")
        return

    # === Extract filtered acceleration ===
    t = df["time"].values
    ax = df["Ax_filt"].values
    ay = df["Ay_filt"].values
    az = df["Az_filt"].values

    # === Optional: speed & angle ===
    speed = np.sqrt(ax**2 + ay**2 + az**2)
    angle = np.degrees(np.arctan2(ay, ax))  # horizontal angle

    # === Shift values to positive quadrant ===
    ax_shift = ax - min(ax) if min(ax) < 0 else ax
    ay_shift = ay - min(ay) if min(ay) < 0 else ay
    az_shift = az - min(az) if min(az) < 0 else az

    # === Create 4 subplots (Top, Rear, Front, Full 3D) ===
    fig = plt.figure(figsize=(20, 6))
    ax_top   = fig.add_subplot(141, projection='3d')
    ax_rear  = fig.add_subplot(142, projection='3d')
    ax_front = fig.add_subplot(143, projection='3d')
    ax_full  = fig.add_subplot(144, projection='3d')

    # Common axis limits
    xlim = [0, max(ax_shift) + 0.1]
    ylim = [0, max(ay_shift) + 0.1]
    zlim = [0, max(az_shift) + 0.1]

    for ax_view in [ax_top, ax_rear, ax_front, ax_full]:
        ax_view.set_xlim(xlim)
        ax_view.set_ylim(ylim)
        ax_view.set_zlim(zlim)
        ax_view.set_xlabel("Ax")
        ax_view.set_ylabel("Ay")
        ax_view.set_zlabel("Az")

    # Different view angles
    ax_top.view_init(elev=90, azim=-90)   # Top
    ax_rear.view_init(elev=0, azim=180)   # Rear
    ax_front.view_init(elev=0, azim=0)    # Front
    ax_full.view_init(elev=30, azim=-60)  # Full 3D (angled)

    # Titles
    ax_top.set_title("Top View")
    ax_rear.set_title("Rear View")
    ax_front.set_title("Front View")
    ax_full.set_title("3D Full Trajectory")

    # Trajectory line and point for each view
    lines, points = [], []
    for ax_view in [ax_top, ax_rear, ax_front, ax_full]:
        line, = ax_view.plot([], [], [], "r-", lw=2, label="Filtered Accel")
        point, = ax_view.plot([], [], [], "ko")
        lines.append(line)
        points.append(point)

    ax_full.legend()

    # === Text box for Speed & Angle ===
    text_box = fig.text(0.5, 0.02, "", fontsize=12, ha='center', va='center')

    # === Animation Update Function ===
    def update(i):
        for line, point in zip(lines, points):
            line.set_data(ax_shift[:i], ay_shift[:i])
            line.set_3d_properties(az_shift[:i])
            point.set_data([ax_shift[i]], [ay_shift[i]])
            point.set_3d_properties([az_shift[i]])

        text_box.set_text(
            f"Time: {t[i]:.2f}s  |  Speed: {speed[i]:.2f} m/s  |  Angle: {angle[i]:.1f}°"
        )
        return lines + points

    ani = FuncAnimation(fig, update, frames=len(t), interval=30, blit=True)
    ani.save(out_file, writer="pillow")
    print(f"[OK] Animation saved as {out_file}")
    plt.show()

if __name__ == "__main__":
    has_run = False
    if has_run:
        print("Script already running, exiting to prevent re-entry.")
        exit(0)
    has_run = True

    imu = GolfIMU()
    accel_arr = None
    gyro_arr = None
    t_arr = None
    fs = 250  # Hz

    # Try serial connection
    try:
        imu.connect()
        print("Collecting IMU data... Perform the golf swing.")
        data = imu.read_data()
        if data and len(data[0]) > 12:
            accel, gyro, vib = data
            t = np.linspace(0, len(accel) / fs, len(accel))
            accel_arr = np.array(accel)
            gyro_arr = np.array(gyro)
            vib_arr = np.array(vib)
            t_arr = np.array(t)
        else:
            print("Error: Insufficient or no data collected from IMU.")
            accel_arr, gyro_arr, vib_arr, t_arr = load_raw_csv()
    except Exception as e:
        print(f"Falling back to CSV because serial connection failed: {e}")
        accel_arr, gyro_arr, vib_arr, t_arr = load_raw_csv()

    imu.close()

    if accel_arr is None or len(accel_arr) <= 12:
        print("Error: Not enough samples to continue.")
        exit(1)

    print(f"Collected {len(accel_arr)} samples.")
    print("accel_arr shape:", accel_arr.shape)
    print("gyro_arr shape:", gyro_arr.shape)
    print("vib_arr shape:", vib_arr.shape)
    print("t_arr shape:", t_arr.shape)

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

    # Shift position to positive quadrant
    pos_shift = np.zeros_like(pos_global)
    pos_shift[:, 0] = pos_global[:, 0] - min(pos_global[:, 0]) if min(pos_global[:, 0]) < 0 else pos_global[:, 0]
    pos_shift[:, 1] = pos_global[:, 1] - min(pos_global[:, 1]) if min(pos_global[:, 1]) < 0 else pos_global[:, 1]
    pos_shift[:, 2] = pos_global[:, 2] - min(pos_global[:, 2]) if min(pos_global[:, 2]) < 0 else pos_global[:, 2]

    # Save CSVs
    df_raw = pd.DataFrame({
        "time": t_arr,
        "Ax": accel_arr[:, 0], "Ay": accel_arr[:, 1], "Az": accel_arr[:, 2],
        "Gx": gyro_arr[:, 0], "Gy": gyro_arr[:, 1], "Gz": gyro_arr[:, 2],"Vib": vib_arr
    })
    df_raw.to_csv("raw.csv", index=False)

    df_pos = pd.DataFrame({
        "time": t_arr,
        "velX": vel_global[:, 0], "velY": vel_global[:, 1], "velZ": vel_global[:, 2],
        "posX": pos_shift[:, 0], "posY": pos_shift[:, 1], "posZ": pos_shift[:, 2],
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

    #6 Plot Vibration vs Time
    ax6 = fig.add_subplot(336)
    ax6.plot(t_arr, vib_arr, label="Vibration", color="purple")
    ax6.set_title("Vibration vs Time")
    ax6.legend()
    ax6.grid(True)

    # 7: Position vs Time
    ax7 = fig.add_subplot(337)
    ax7.plot(t_arr, pos_shift[:, 0], label="X")
    ax7.plot(t_arr, pos_shift[:, 1], label="Y")
    ax7.plot(t_arr, pos_shift[:, 2], label="Z")
    ax7.set_title("Position vs Time")
    ax7.legend()
    ax7.grid(True)
    # 8: Euler Angles
    ax8 = fig.add_subplot(338)
    ax8.plot(t_arr, euler_deg[:, 0], label="Yaw")
    ax8.plot(t_arr, euler_deg[:, 1], label="Pitch")
    ax8.plot(t_arr, euler_deg[:, 2], label="Roll")
    ax8.set_title("Euler Angles vs Time")
    ax8.legend()
    ax8.grid(True)

    # 9: 3D Trajectory (Positive Axes)
    ax9 = fig.add_subplot(339, projection="3d")
    ax9.plot(pos_shift[:, 0], pos_shift[:, 1], pos_shift[:, 2], "b-")
    ax9.set_xlim([0, max(pos_shift[:, 0]) + 0.1])
    ax9.set_ylim([0, max(pos_shift[:, 1]) + 0.1])
    ax9.set_zlim([0, max(pos_shift[:, 2]) + 0.1])
    ax9.set_title("3D Displacement (Positive Axes)")
    ax9.set_xlabel("X (m)")
    ax9.set_ylabel("Y (m)")
    ax9.set_zlabel("Z (m)")

    # Extra subplot: 3D Acceleration (Filtered, Positive Axes)
    fig2 = plt.figure(figsize=(8, 6))
    ax9 = fig2.add_subplot(111, projection="3d")
    ax_filt_shift = ax_filt - min(ax_filt) if min(ax_filt) < 0 else ax_filt
    ay_filt_shift = ay_filt - min(ay_filt) if min(ay_filt) < 0 else ay_filt
    az_filt_shift = az_filt - min(az_filt) if min(az_filt) < 0 else az_filt
    ax9.plot(ax_filt_shift, ay_filt_shift, az_filt_shift, "r-")
    ax9.set_xlim([0, max(ax_filt_shift) + 0.1])
    ax9.set_ylim([0, max(ay_filt_shift) + 0.1])
    ax9.set_zlim([0, max(az_filt_shift) + 0.1])
    ax9.set_title("3D Acceleration (Filtered, Positive Axes)")
    ax9.set_xlabel("Ax (m/s²)")
    ax9.set_ylabel("Ay (m/s²)")
    ax9.set_zlabel("Az (m/s²)")
    ax9.grid(True)

    plt.tight_layout()
    plt.show()
    animate_accel("accelfilt.csv", "accelfilt.gif")
