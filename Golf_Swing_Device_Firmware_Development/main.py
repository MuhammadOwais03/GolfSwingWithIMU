from python_scripts.read_py1 import GolfIMU
from python_scripts.helper import load_raw_csv, plot_graphs, animate_imu_low_accel, plot_imu_low_accel_trajectory, get_impact_index
import numpy as np
import pandas as pd
from scipy import signal, integrate
from scipy.spatial.transform import Rotation as R
import os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import argparse
import sys
import time
import matplotlib.animation as animation
from PIL import Image
from multiprocessing import Process



BASE_PATH = "python_scripts/data"


def display_animated_gif(gif_path="imu_low_accel.gif"):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    """Load and display GIF as animated plot."""
    # Load all frames from GIF
    frames = []
    try:
        with Image.open(gif_path) as img:
            for frame_idx in range(img.n_frames):
                img.seek(frame_idx)
                # Convert to RGB if needed (GIFs can be RGBA)
                frame = np.array(img.convert('RGB'))
                frames.append(frame)
        print(f"[INFO] Loaded {len(frames)} frames from {gif_path}")
    except Exception as e:
        print(f"[ERROR] Could not load GIF: {e}")
        return

    # Create figure and animation
    fig, ax = plt.subplots(figsize=(12, 4))  # Adjust size to fit your GIF
    def animate(frame_idx):
        ax.clear()
        ax.imshow(frames[frame_idx])
        ax.axis('off')  # Hide axes for clean view
        ax.set_title(f"IMU Low Accel Animation - Frame {frame_idx + 1}/{len(frames)}")

    # Animate at ~15 FPS (match your GIF's FPS)
    ani = animation.FuncAnimation(fig, animate, frames=len(frames), interval=66, repeat=True, blit=False)
    plt.tight_layout()
    plt.show()


def clear_screen():
    # Works on both Linux/Mac and Windows
    os.system('cls' if os.name == 'nt' else 'clear')


            

def main_menu(t_arr, imu_low_accel_arr, imu_gyro, ax_filt, ay_filt, az_filt, gx_filt, gy_filt, gz_filt, 
              vel_global, imu_high_accel_arr, pos_shift, euler_deg, imu_euler):
    csv_path = os.path.join(BASE_PATH, "raw.csv")

    while True:
        clear_screen()
        print("========================================")
        print("        🏌️  Golf Swing Data CLI        ")
        print("========================================")
        print("1️⃣  Show all graphs")
        print("2️⃣  Animate IMU (low accel)")
        print("3️⃣  Plot IMU low accel trajectory")
        print("4️⃣  Exit")
        print("========================================")

        choice = input("👉 Enter your choice (1–4): ").strip()

        clear_screen()

        if choice == "1":
            print("📈 Displaying graphs...\n")
            plot_graphs(
                t_arr, imu_low_accel_arr, imu_gyro,
                ax_filt, ay_filt, az_filt,
                gx_filt, gy_filt, gz_filt,
                vel_global, imu_high_accel_arr, pos_shift,
                euler_deg, imu_euler
            )
            input("\n✅ Press Enter to return to the menu...")

        elif choice == "2":
            print("🎬 Generating low-accel animation...")
            print("⏳ This may take around 1 second...\n")
            time.sleep(1)  # Small delay for clarity
            animate_imu_low_accel(csv_path, "imu_low_accel.gif")
            print("\n✅ Animation complete! Saved as: imu_low_accel.gif")
             # Start GIF viewer in separate process to avoid Tk threading conflicts
            p = Process(target=display_animated_gif, args=("imu_low_accel.gif",))
            p.start()
            p.join()  
            input("\nPress Enter to return to the menu...")

        elif choice == "3":
            print("📊 Plotting IMU low accel trajectory...\n")
            plot_imu_low_accel_trajectory()
            input("\n✅ Press Enter to return to the menu...")

        elif choice == "4":
            print("👋 Exiting. Goodbye!\n")
            break

        else:
            print("❌ Invalid choice. Please enter a number between 1 and 4.")
            time.sleep(1)
            
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
        imu_low_accel_arr, imu_high_accel_arr, imu_gyro, imu_euler, imu_vibration_arr, t_arr, vel, roll = load_raw_csv()

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
    
    
    # row_idx, col_idx = get_impact_index(imu_high_accel_arr)
    # print(f"Impact detected at index {row_idx}, axis {['X','Y','Z'][col_idx]} with value {imu_high_accel_arr[row_idx, col_idx]} m/s²")
    # imu_low_accel_arr = imu_low_accel_arr[0:row_idx]
    # imu_high_accel_arr = imu_high_accel_arr[0:row_idx]
    # imu_gyro = imu_gyro[0:row_idx]
    # imu_vibration_arr = imu_vibration_arr[0:row_idx]
    # imu_euler = imu_euler[0:row_idx]
    # vel = vel[0:row_idx] if vel is not None else None
    # roll = roll[0:row_idx] if roll is not None else None
    # t_arr = t_arr[0:row_idx]
    
    # print(f"Data truncated to {len(imu_low_accel_arr)} samples up to impact. Shape {imu_low_accel_arr.shape}")
    # print(f"Data truncated to {len(imu_gyro)} samples up to impact. Shape {imu_gyro.shape}")
    # print(f"Data truncated to {len(imu_vibration_arr)} samples up to impact. Shape {imu_vibration_arr.shape}")
    # print(f"Data truncated to {len(imu_euler)} samples up to impact. Shape {imu_euler.shape}")
    # print(f"Data truncated to {len(t_arr)} samples up to impact. Shape {t_arr.shape}")
    # print("Press Enter to continue with processing...")
    # input()
    
    

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
    
    # Magnitude of High Accel IMU for vibration
    
    
    print("Shapes before DataFrame creation:")
    print("t_arr:", len(t_arr))
    print("imu_low_accel_arr:", imu_low_accel_arr.shape)
    print("imu_high_accel_arr:", imu_high_accel_arr.shape)
    print("imu_gyro:", imu_gyro.shape)
    print("imu_vibration_arr:", imu_vibration_arr.shape)
    print("imu_euler:", imu_euler.shape)
    print("vel_global:", vel_global.shape)
    print("pos_shift:", pos_shift.shape)
    print("euler_deg:", euler_deg.shape)
    print("ax_filt:", len(ax_filt), "ay_filt:", len(ay_filt), "az_filt:", len(az_filt))


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
    df_raw.to_csv(f"{BASE_PATH}/raw.csv", index=False)

    
    
    main_menu(t_arr, imu_low_accel_arr, imu_gyro, ax_filt, ay_filt, az_filt, gx_filt, gz_filt, gy_filt, vel_global, imu_high_accel_arr, pos_shift, euler_deg, imu_euler)
