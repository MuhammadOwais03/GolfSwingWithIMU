
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
        self.imu_low_accel_IN_MG = True
        self.GRAVITY = 9.81
        self.ser = None
        self.offsets_acc = np.zeros(3)
        self.offsets_imu_gyro = np.zeros(3)

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

        imu_low_accel = []   # [lax, lay, laz]
        imu_high_accel = []  # [hax, hay, haz]
        imu_gyro = []        # [gx, gy, gz]
        imu_euler = []       # [pitch, roll, yaw]
        imu_vibration = []   # [imu_vibration]

        for r in readings:
            r = r.strip().split(",")
            if len(r) == 13:
                try:
                    lax, lay, laz = float(r[1]), (float(r[0])+1) * -1, float(r[2]) * -1
                    hax, hay, haz = -float(r[3]), -float(r[4]), -float(r[5])
                    gx, gy, gz = float(r[6]), float(r[7]), float(r[8])
                    pitch, roll, yaw = float(r[9]), float(r[10]), float(r[11])
                    vib = float(r[12])

                    imu_low_accel.append([lax, lay, laz])
                    imu_high_accel.append([hax, hay, haz])
                    imu_gyro.append([gx, gy, gz])
                    imu_euler.append([pitch, roll, yaw])
                    imu_vibration.append(vib)

                except ValueError as e:
                    print(f"Error parsing line {r}: {e}")
                    continue

        # --- Convert to numpy arrays for easy processing ---
        imu_low_accel = np.array(imu_low_accel)
        imu_high_accel = np.array(imu_high_accel)
        imu_gyro = np.array(imu_gyro)
        imu_euler = np.array(imu_euler)
        imu_vibration = np.array(imu_vibration)

        if imu_low_accel.size == 0:
            print("Warning: No valid IMU data collected.")
            return np.array([]), np.array([]), np.array([]), np.array([]), np.array([])

        # Optionally subtract offsets if you have them
        imu_low_accel -= self.offsets_acc
        imu_gyro -= self.offsets_imu_gyro

        return imu_low_accel, imu_high_accel, imu_gyro, imu_euler, imu_vibration

    
    
    def close(self):
        if self.ser:
            self.ser.close()
            print("Serial connection closed")

def load_raw_csv(file_path="raw.csv"):
    if not os.path.exists(file_path):
        return None, None, None

    df = pd.read_csv(file_path)
    imu_low_accel_arr = df[["low_ax", "low_ay", "low_az"]].values
    imu_high_accel_arr = df[["high_ax", "high_ay", "high_az"]].values
    imu_euler = df[["pitch", "roll", "yaw"]].values
    imu_gyro = df[["Gx", "Gy", "Gz"]].values
    vel = df[["velX", "velY", "velZ"]].values  # shape: (N, 3)
    roll = df[["pitch", "roll", "yaw"]].values  # shape: (N, 3)
    imu_vibration_arr = df["imu_vibration"].values if "imu_vibration" in df.columns else np.zeros(len(imu_low_accel_arr))
    t_arr = df["time"].values
    return imu_low_accel_arr, imu_high_accel_arr, imu_gyro, imu_euler, imu_vibration_arr, t_arr, vel, roll

def get_Impact_time_face_angle_velocity(file_path="raw.csv"):
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found.")
        return None, None, None
    
    df = pd.read_csv(file_path)
    imu_high_accel_arr = df[["high_ax", "high_ay", "high_az"]].values
    t_arr = df["time"].values  # Define t_arr before use
    
    # Find index of maximum acceleration value
    max_index = np.unravel_index(np.argmax(imu_high_accel_arr), imu_high_accel_arr.shape)
    row_idx = max_index[0]
    col_idx = max_index[1]
    
    # Get details
    max_value = imu_high_accel_arr[row_idx, col_idx]
    axis_name = ['X', 'Y', 'Z'][col_idx]
    time_at_max = t_arr[row_idx]

    # Get velocity, face angle (yaw), and acceleration at time of impact
    vel = df[["velX", "velY", "velZ"]].values  # shape: (N, 3)
    roll = df[["pitch", "roll", "yaw"]].values
    accel = df[["low_ax", "low_ay", "low_az"]].values
    target_time = time_at_max
    idx = (np.abs(t_arr - target_time)).argmin()
    velocity_at_time = vel[idx]
    roll_at_time = roll[idx]
    accel_at_time = accel[idx]

    # Calculate resultant velocity and acceleration magnitude
    resultant_velocity = np.linalg.norm(velocity_at_time)
    accel_magnitude = np.linalg.norm(accel_at_time)

    # Print face angle (yaw), velocity, and acceleration at impact
    print(f"Impact Time: {t_arr[idx]:.3f}s")
    print(f"Face Angle (Roll): {roll_at_time[1]:.2f} deg")
    print(f"Resultant Velocity: {resultant_velocity:.6f} m/s")
    print(f"Acceleration Magnitude: {accel_magnitude:.6f} m/s²")

    return time_at_max, roll_at_time[1], resultant_velocity

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_imu_low_accel_trajectory(csv_file="raw.csv"):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return

    # === Extract filtered imu_low_acceleration ===
    t = df["time"].values
    ax = df["Ax_filt"].values
    ay = df["Ay_filt"].values
    az = df["Az_filt"].values

    # === Shift to positive quadrant (optional) ===
    ax_shift = ax - min(ax) if min(ax) < 0 else ax
    ay_shift = ay - min(ay) if min(ay) < 0 else ay
    az_shift = az - min(az) if min(az) < 0 else az

    # === Identify impact point using vibration (high frequency accelerations) ===
    df['vibe_mag'] = np.sqrt(df['high_ax']**2 + df['high_ay']**2 + df['high_az']**2)
    impact_idx = df['vibe_mag'].argmax()

    # === Identify top of backswing: last sign change in Gz (yaw rate) before impact ===
    pre_impact_df = df.iloc[:impact_idx + 1]
    gz_sign = np.sign(pre_impact_df['Gz'].values)
    sign_diff = np.diff(gz_sign)
    sign_change_idxs = np.where(sign_diff != 0)[0]
    if len(sign_change_idxs) > 0:
        top_idx = sign_change_idxs[-1] + 1
    else:
        top_idx = 0  # Default to start if no sign change

    # === Split data into phases ===
    # Backswing: start to top
    back_ax = ax_shift[:top_idx + 1]
    back_ay = ay_shift[:top_idx + 1]
    back_az = az_shift[:top_idx + 1]

    # Downswing: top to impact
    down_ax = ax_shift[top_idx:impact_idx + 1]
    down_ay = ay_shift[top_idx:impact_idx + 1]
    down_az = az_shift[top_idx:impact_idx + 1]

    # Follow-through: after impact (optional, in blue)
    follow_ax = ax_shift[impact_idx:]
    follow_ay = ay_shift[impact_idx:]
    follow_az = az_shift[impact_idx:]

    # === Create figure and subplots ===
    fig = plt.figure(figsize=(20, 6))
    ax_top   = fig.add_subplot(141, projection='3d')
    ax_rear  = fig.add_subplot(142, projection='3d')
    ax_front = fig.add_subplot(143, projection='3d')
    ax_full  = fig.add_subplot(144, projection='3d')

    # Common limits
    xlim = [0, max(ax_shift) + 0.1]
    ylim = [0, max(ay_shift) + 0.1]
    zlim = [0, max(az_shift) + 0.1]
    for view in [ax_top, ax_rear, ax_front, ax_full]:
        view.set_xlim(xlim)
        view.set_ylim(ylim)
        view.set_zlim(zlim)
        view.set_xlabel("Ax (m/s²)")
        view.set_ylabel("Ay (m/s²)")
        view.set_zlabel("Az (m/s²)")

    # View angles
    ax_top.view_init(90, -90)    # top
    ax_rear.view_init(-90, 90)    # rear
    ax_front.view_init(-30, 60)    # front
    ax_full.view_init(30, -60)   # full trajectory

    # Titles
    ax_top.set_title("Side View")
    ax_rear.set_title("Rear View")
    ax_front.set_title("Front View")
    ax_full.set_title("3D Full Trajectory")

    # === Plot phases with colors ===
    for view in [ax_top, ax_rear, ax_front, ax_full]:
        # Backswing in red
        view.plot(back_ax, back_ay, back_az, "r-", lw=2, label="Backswing")
        
        # Downswing in green
        view.plot(down_ax, down_ay, down_az, "g-", lw=2, label="Downswing")
        
        # Follow-through in blue (optional)
        if follow_ax.size > 0:
            view.plot(follow_ax, follow_ay, follow_az, "g-", lw=2, label="Follow-through")
        
        # Final point
        view.plot([ax_shift[-1]], [ay_shift[-1]], [az_shift[-1]], "ko")

    # === Optional info box ===
    impact_time, face_angle, imapact_speed = get_Impact_time_face_angle_velocity()
    text_str = f"Final Time: {impact_time:.9f}s  |  Final Speed: {imapact_speed:.3f} m/s  |  Angle: {face_angle:.1f}°"
    fig.text(0.5, 0.02, text_str, fontsize=12, ha='center', va='center')

    # Add legend to the full view (or adjust as needed)
    ax_full.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

def animate_imu_low_accel(csv_file="raw.csv", out_file="imu_low_accelfilt.gif"):
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Ensure data is saved correctly.")
        return

    # === Extract filtered imu_low_acceleration ===
    t = df["time"].values
    ax = df["Ax_filt"].values
    ay = df["Ay_filt"].values
    az = df["Az_filt"].values
    # === Identify impact point using vibration (high frequency accelerations) ===
    df['vibe_mag'] = np.sqrt(df['high_ax']**2 + df['high_ay']**2 + df['high_az']**2)
    impact_idx = df['vibe_mag'].argmax()

    # === Shift to positive quadrant (optional) ===
    ax_shift = ax - min(ax) if min(ax) < 0 else ax
    ay_shift = ay - min(ay) if min(ay) < 0 else ay
    az_shift = az - min(az) if min(az) < 0 else az

    # === Identify top of backswing: last sign change in Gz (yaw rate) before impact ===
    pre_impact_df = df.iloc[:impact_idx + 1]
    gz_sign = np.sign(pre_impact_df['Gz'].values)
    sign_diff = np.diff(gz_sign)
    sign_change_idxs = np.where(sign_diff != 0)[0]
    if len(sign_change_idxs) > 0:
        top_idx = sign_change_idxs[-1] + 1
    else:
        top_idx = 0  # Default to start if no sign change

    # === Identify top of backswing: last sign change in Gz (yaw rate) before impact ===
    pre_impact_df = df.iloc[:impact_idx + 1]
    gz_sign = np.sign(pre_impact_df['Gz'].values)
    sign_diff = np.diff(gz_sign)
    sign_change_idxs = np.where(sign_diff != 0)[0]
    if len(sign_change_idxs) > 0:
        top_idx = sign_change_idxs[-1] + 1
    else:
        top_idx = 0  # Default to start if no sign change

    # === Split data into phases ===
    # Backswing: start to top
    back_ax = ax_shift[:top_idx + 1]
    back_ay = ay_shift[:top_idx + 1]
    back_az = az_shift[:top_idx + 1]

    # Downswing: top to impact
    down_ax = ax_shift[top_idx:impact_idx + 1]
    down_ay = ay_shift[top_idx:impact_idx + 1]
    down_az = az_shift[top_idx:impact_idx + 1]

    # Follow-through: after impact (optional, in blue)
    follow_ax = ax_shift[impact_idx:]
    follow_ay = ay_shift[impact_idx:]
    follow_az = az_shift[impact_idx:]
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
    # View angles
    ax_top.view_init(90, -90)    # top
    ax_rear.view_init(-90, 90)    # rear
    ax_front.view_init(-30, 60)    # front
    ax_full.view_init(30, -60)   # full trajectory

    # Titles
    ax_top.set_title("Side View")
    ax_rear.set_title("Rear View")
    ax_front.set_title("Front View")
    ax_full.set_title("3D Full Trajectory")

    # Trajectory line and point for each view
    lines, points = [], []
    for ax_view in [ax_top, ax_rear, ax_front, ax_full]:
        line_1, = ax_view.plot(back_ax, back_ay, back_az, "r-", lw=2, label="Backswing")
        line_2, = ax_view.plot(down_ax, down_ay, down_az, "g-", lw=2, label="Downswing")
        line_3, = ax_view.plot(follow_ax, follow_ay, follow_az, "g-", lw=2, label="Follow-through")
        point, = ax_view.plot([], [], [], "ko")
        lines.append(line_1)
        lines.append(line_2)
        lines.append(line_3)
        points.append(point)

    ax_full.legend()

    # === Text box for Speed & Angle ===
    text_box = fig.text(0.5, 0.02, "", fontsize=12, ha='center', va='center')

    def update(i):
        # segment indexes
        for v in range(4):  # for each subplot view
            # 3 lines per view (backswing, downswing, follow)
            back_line = lines[v*3 + 0]
            down_line = lines[v*3 + 1]
            follow_line = lines[v*3 + 2]
            point = points[v]

            # Backswing segment
            idx_back = min(i, top_idx)
            back_line.set_data(ax_shift[:idx_back], ay_shift[:idx_back])
            back_line.set_3d_properties(az_shift[:idx_back])

            # Downswing segment
            if i > top_idx:
                idx_down = min(i, impact_idx)
                down_line.set_data(ax_shift[top_idx:idx_down], ay_shift[top_idx:idx_down])
                down_line.set_3d_properties(az_shift[top_idx:idx_down])
            else:
                down_line.set_data([], [])
                down_line.set_3d_properties([])

            # Follow-through segment
            if i > impact_idx:
                idx_follow = min(i, len(ax_shift)-1)
                follow_line.set_data(ax_shift[impact_idx:idx_follow], ay_shift[impact_idx:idx_follow])
                follow_line.set_3d_properties(az_shift[impact_idx:idx_follow])
            else:
                follow_line.set_data([], [])
                follow_line.set_3d_properties([])

            # Current point marker
            point.set_data([ax_shift[i]], [ay_shift[i]])
            point.set_3d_properties([az_shift[i]])

        # Update text box
        impact_time, face_angle, impact_speed = get_Impact_time_face_angle_velocity()
        text_box.set_text(
            f"Time: {impact_time:.6f}s  |  Speed: {impact_speed:.3f} m/s  |  Angle: {face_angle:.3f}°"
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
    imu_low_accel_arr = None
    imu_gyro = None
    t_arr = None
    vel = None
    roll = None
    target_time = None
    resultant_velocity = None
    fs = 800  # Hz

    # Try serial connection
    try:
        imu.connect()
        print("Collecting IMU data... Perform the golf swing.")
        data = imu.read_data()
        print(data)
        if data and len(data[0]) > 12:
            imu_low_accel_arr, imu_high_accel_arr, imu_gyro, imu_euler, imu_vibration_arr = data
            t = np.linspace(0, len(imu_low_accel_arr) / fs, len(imu_low_accel_arr))
            t_arr = np.array(t)
        else:
            print("Error: Insufficient or no data collected from IMU.")
            imu_low_accel_arr, imu_high_accel_arr, imu_gyro, imu_euler, imu_vibration_arr, t_arr, vel,roll = load_raw_csv()
    except Exception as e:
        print(f"Falling back to CSV because serial connection failed: {e}")
        imu_low_accel_arr, imu_high_accel_arr, imu_gyro, imu_euler, imu_vibration_arr, t_arr, ve, roll = load_raw_csv()

    imu.close()

    if imu_low_accel_arr is None or len(imu_low_accel_arr) <= 12:
        print("Error: Not enough samples to continue.")
        exit(1)

    print(f"Collected {len(imu_low_accel_arr)} samples.")
    print("imu_low_accel_arr shape:", imu_low_accel_arr.shape)
    print("imu_gyro shape:", imu_gyro.shape)
    print("imu_vibration_arr shape:", imu_vibration_arr.shape)
    print("t_arr shape:", t_arr.shape)
    print("vel shape:", vel.shape if vel is not None else "None (velocity data not available)")
    print("Roll shape:", roll.shape if roll is not None else "None (roll data not available)") 

    if len(imu_low_accel_arr) <= 12:
        print("Error: Not enough samples for filtering.")
        exit(1)

    dt = np.mean(np.diff(t_arr))

    # Low-pass filter
    b, a = signal.butter(3, 6 / (fs / 2), btype='low')
    ax_filt = signal.filtfilt(b, a, imu_low_accel_arr[:, 0], padlen=min(3, len(imu_low_accel_arr)-1))
    ay_filt = signal.filtfilt(b, a, imu_low_accel_arr[:, 1], padlen=min(3, len(imu_low_accel_arr)-1))
    az_filt = signal.filtfilt(b, a, imu_low_accel_arr[:, 2], padlen=min(3, len(imu_low_accel_arr)-1))
    gx_filt = signal.filtfilt(b, a, imu_gyro[:, 0], padlen=min(3, len(imu_gyro)-1))
    gy_filt = signal.filtfilt(b, a, imu_gyro[:, 1], padlen=min(3, len(imu_gyro)-1))
    gz_filt = signal.filtfilt(b, a, imu_gyro[:, 2], padlen=min(3, len(imu_gyro)-1))

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

    # Rotate imu_low_accel into global frame
    imu_low_accel_global = np.zeros_like(imu_low_accel_arr)
    for i in range(len(t_arr)):
        rot = R.from_matrix(R_mats[i])
        imu_low_accel_global[i] = rot.apply([ax_filt[i], ay_filt[i], az_filt[i]])
    imu_low_accel_global[:, 2] -= imu.GRAVITY

    # Stationary detection
    acc_mag = np.linalg.norm(imu_low_accel_arr, axis=1)
    b, a = signal.butter(1, 0.001 / (fs / 2), "high")
    acc_mag_filt = signal.filtfilt(b, a, acc_mag, padlen=min(3, len(acc_mag)-1))
    acc_mag_filt = np.abs(acc_mag_filt)
    b, a = signal.butter(1, 5 / (fs / 2), "low")
    acc_mag_filt = signal.filtfilt(b, a, acc_mag_filt, padlen=min(3, len(acc_mag_filt)-1))
    stationary = acc_mag_filt < 0.06

    # Velocity with ZUPT
    vel_global = np.zeros_like(imu_low_accel_global)
    for i in range(1, len(t_arr)):
        vel_global[i] = vel_global[i - 1] + imu_low_accel_global[i] * dt
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

    # # Save CSVs
    df_raw = pd.DataFrame({
        "time": t_arr,
        "low_ax": imu_low_accel_arr[:, 0], "low_ay": imu_low_accel_arr[:, 1], "low_az": imu_low_accel_arr[:, 2],
        "high_ax": imu_high_accel_arr[:, 0], "high_ay": imu_high_accel_arr[:, 1], "high_az": imu_high_accel_arr[:, 2],
        "pitch": imu_euler[:, 0], "roll": imu_euler[:, 1], "yaw": imu_euler[:, 2],
        "Gx": imu_gyro[:, 0], "Gy": imu_gyro[:, 1], "Gz": imu_gyro[:, 2],"imu_vibration": imu_vibration_arr,
        "velX": vel_global[:, 0], "velY": vel_global[:, 1], "velZ": vel_global[:, 2],
        "posX": pos_shift[:, 0], "posY": pos_shift[:, 1], "posZ": pos_shift[:, 2],
        "yaw_deg": euler_deg[:, 0], "pitch_deg": euler_deg[:, 1], "roll_deg": euler_deg[:, 2],
        "Ax_filt": ax_filt, "Ay_filt": ay_filt, "Az_filt": az_filt
    })
    df_raw.to_csv("raw.csv", index=False)

    # === Improved Plot Layout ===
    fig, axs = plt.subplots(3, 3, figsize=(20, 14))  # larger figure
    plt.subplots_adjust(wspace=0.3, hspace=0.5)      # spacing between plots

    # 1: Raw imu_low_acceleration
    axs[0, 0].plot(t_arr, imu_low_accel_arr[:, 0], label="Ax", linewidth=1)
    axs[0, 0].plot(t_arr, imu_low_accel_arr[:, 1], label="Ay", linewidth=1)
    axs[0, 0].plot(t_arr, imu_low_accel_arr[:, 2], label="Az", linewidth=1)
    axs[0, 0].set_title("Raw imu_low_acceleration (m/s²)")
    axs[0, 0].legend(loc='upper right', fontsize=8)
    axs[0, 0].grid(True)

    # 2: Raw imu_gyroscope
    axs[0, 1].plot(t_arr, imu_gyro[:, 0], label="Gx")
    axs[0, 1].plot(t_arr, imu_gyro[:, 1], label="Gy")
    axs[0, 1].plot(t_arr, imu_gyro[:, 2], label="Gz")
    axs[0, 1].set_title("Raw imu_gyroscope (rad/s)")
    axs[0, 1].legend(loc='upper right', fontsize=8)
    axs[0, 1].grid(True)

    # 3: Filtered imu_low_acceleration
    axs[0, 2].plot(t_arr, ax_filt, label="Ax")
    axs[0, 2].plot(t_arr, ay_filt, label="Ay")
    axs[0, 2].plot(t_arr, az_filt, label="Az")
    axs[0, 2].set_title("Filtered imu_low_acceleration (m/s²)")
    axs[0, 2].legend(loc='upper right', fontsize=8)
    axs[0, 2].grid(True)

    # 4: Filtered imu_gyroscope
    axs[1, 0].plot(t_arr, gx_filt, label="Gx")
    axs[1, 0].plot(t_arr, gy_filt, label="Gy")
    axs[1, 0].plot(t_arr, gz_filt, label="Gz")
    axs[1, 0].set_title("Filtered imu_gyroscope (rad/s)")
    axs[1, 0].legend(loc='upper right', fontsize=8)
    axs[1, 0].grid(True)

    # 5: Velocity vs Time
    axs[1, 1].plot(t_arr, vel_global[:, 0], label="Vx")
    axs[1, 1].plot(t_arr, vel_global[:, 1], label="Vy")
    axs[1, 1].plot(t_arr, vel_global[:, 2], label="Vz")
    axs[1, 1].set_title("Velocity vs Time")
    axs[1, 1].legend(loc='upper right', fontsize=8)
    axs[1, 1].grid(True)

    # 6: Vibration vs Time
    axs[1, 2].plot(t_arr, imu_high_accel_arr[:, 0], label="Hax")
    axs[1, 2].plot(t_arr, imu_high_accel_arr[:, 1], label="Hay")
    axs[1, 2].plot(t_arr, imu_high_accel_arr[:, 2], label="Haz")
    axs[1, 2].set_title("IMU Vibration vs Time")
    axs[1, 2].legend(loc='upper right', fontsize=8)
    axs[1, 2].grid(True)

    # 7: Position vs Time
    axs[2, 0].plot(t_arr, pos_shift[:, 0], label="X")
    axs[2, 0].plot(t_arr, pos_shift[:, 1], label="Y")
    axs[2, 0].plot(t_arr, pos_shift[:, 2], label="Z")
    axs[2, 0].set_title("Position vs Time")
    axs[2, 0].legend(loc='upper right', fontsize=8)
    axs[2, 0].grid(True)

    # 8: Euler Angles (from euler_deg)
    axs[2, 1].plot(t_arr, euler_deg[:, 0], label="Yaw")
    axs[2, 1].plot(t_arr, euler_deg[:, 1], label="Pitch")
    axs[2, 1].plot(t_arr, euler_deg[:, 2], label="Roll")
    axs[2, 1].set_title("Euler Angles vs Time (deg)")
    axs[2, 1].legend(loc='upper right', fontsize=8)
    axs[2, 1].grid(True)

    # 9: Euler Angles (from imu_euler)
    axs[2, 2].plot(t_arr, imu_euler[:, 0], label="Pitch")
    axs[2, 2].plot(t_arr, imu_euler[:, 1], label="Roll")
    axs[2, 2].plot(t_arr, imu_euler[:, 2], label="Yaw")
    axs[2, 2].set_title("IMU Euler Angles vs Time (rad)")
    axs[2, 2].legend(loc='upper right', fontsize=8)
    axs[2, 2].grid(True)

    # === 3D Plot in separate figure ===
    fig3d = plt.figure(figsize=(8, 6))
    ax3d = fig3d.add_subplot(111, projection="3d")
    ax_filt_shift = ax_filt - min(ax_filt) if min(ax_filt) < 0 else ax_filt
    ay_filt_shift = ay_filt - min(ay_filt) if min(ay_filt) < 0 else ay_filt
    az_filt_shift = az_filt - min(az_filt) if min(az_filt) < 0 else az_filt
    ax3d.plot(ax_filt_shift, ay_filt_shift, az_filt_shift, "r-")
    ax3d.set_xlim([0, max(ax_filt_shift) + 0.1])
    ax3d.set_ylim([0, max(ay_filt_shift) + 0.1])
    ax3d.set_zlim([0, max(az_filt_shift) + 0.1])
    ax3d.set_title("3D IMU Low Acceleration (Filtered)")
    ax3d.set_xlabel("Ax (m/s²)")
    ax3d.set_ylabel("Ay (m/s²)")
    ax3d.set_zlabel("Az (m/s²)")
    ax3d.grid(True)

    plt.show()

    plt.tight_layout()
    plt.show()
    animate_imu_low_accel("raw.csv", "imu_low_accelfilt.gif")
    plot_imu_low_accel_trajectory()






