import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from multiprocessing.dummy import Pool
import imageio.v3 as iio  
from pathlib import Path
import os 



BASE_PATH = "python_scripts/data"



def load_raw_csv(file_path=f"{BASE_PATH}/raw.csv"):
    print(f"Loading data from {file_path}...")
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

def get_impact_index(imu_high_accel_arr):
    # Find index of maximum acceleration value
    max_index = np.unravel_index(np.argmax(imu_high_accel_arr), imu_high_accel_arr.shape) 
    print(max_index)
    row_idx = max_index[0]
    col_idx = max_index[1]
    
    # Get details
    max_value = imu_high_accel_arr[row_idx, col_idx]
    axis_name = ['X', 'Y', 'Z'][col_idx]
    print(f"Max High Accel: {max_value} m/s² on axis {axis_name} at index {row_idx}")
    return row_idx, col_idx

def get_Impact_time_face_angle_velocity(file_path=f"{BASE_PATH}/raw.csv"):
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found.")
        return None, None, None
    
    df = pd.read_csv(file_path)
    imu_high_accel_arr = df[["high_ax", "high_ay", "high_az"]].values
    t_arr = df["time"].values  # Define t_arr before use
    
    # Find index of maximum acceleration value
    max_index = np.unravel_index(np.argmax(imu_high_accel_arr), imu_high_accel_arr.shape) 
    print(max_index)
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


def plot_imu_low_accel_trajectory(csv_file=f"{BASE_PATH}/raw.csv"):
    
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    
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
    ax_top.view_init(90,-90)    # top
    ax_rear.view_init(-90, 90)    # rear
    ax_front.view_init(-90, 90)    # front
    ax_full.view_init(30, -60)   # full trajectory

    # Titles
    ax_top.set_title("Side View")
    ax_rear.set_title("Rear View")
    ax_front.set_title("Front View")
    ax_full.set_title("3D Full Trajectory")
    
    # == side view ==
    ax_top.plot(back_ax, back_ay, back_az, "r-", lw=2, label="Backswing")
    ax_top.plot(down_ax, down_ay, down_az, "g-", lw=2, label="Downswing")
    if follow_ax.size > 0:
            ax_top.plot(follow_ax, follow_ay, follow_az, "g-", lw=2, label="Follow-through")
    ax_top.plot([ax_shift[-1]], [ay_shift[-1]], [az_shift[-1]], "ko")
    
    # == rear view ==  x=z and z=x
    ax_rear.plot(-back_az+1, back_ay, back_ax+1, "r-", lw=2, label="Backswing")
    ax_rear.plot(-down_az+1, down_ay, down_ax+1, "g-", lw=2, label="Downswing")
    if follow_az.size > 0:
            ax_rear.plot(-follow_az+1, follow_ay, follow_ax+1, "g-", lw=2, label="Follow-through")
    ax_rear.plot([-az_shift[-1] + 1], [ay_shift[-1]], [ax_shift[-1] + 1], "ko")
    
     # == front view ==  x=-z and z=x
    ax_front.plot(back_az + 1, back_ay, back_ax + 1, "r-", lw=2, label="Backswing")
    ax_front.plot(down_az + 1, down_ay, down_ax + 1, "g-", lw=2, label="Downswing")
    if follow_az.size > 0:
            ax_front.plot(follow_az + 1, follow_ay, follow_ax + 1, "g-", lw=2, label="Follow-through")
    ax_front.plot([az_shift[-1] + 1] , [ay_shift[-1] ], [ax_shift[-1] + 1] , "ko")
    
    # == 3d view ==
    ax_full.plot(back_ax, back_ay, back_az, "r-", lw=2, label="Backswing")
    ax_full.plot(down_ax, down_ay, down_az, "g-", lw=2, label="Downswing")
    if follow_ax.size > 0:
            ax_full.plot(follow_ax, follow_ay, follow_az, "g-", lw=2, label="Follow-through")
    ax_full.plot([ax_shift[-1]], [ay_shift[-1]], [az_shift[-1]], "ko")
    
    # === Optional info box ===
    impact_time, face_angle, imapact_speed = get_Impact_time_face_angle_velocity()
    text_str = f"Final Time: {impact_time:.9f}s  |  Final Speed: {imapact_speed:.3f} m/s  |  Angle: {face_angle:.1f}°"
    fig.text(0.5, 0.02, text_str, fontsize=12, ha='center', va='center')

    # Add legend to the full view (or adjust as needed)
    ax_full.legend(loc='upper right')

    plt.tight_layout()
    plt.show()



def render_frame(args):
    
    i, back_ax, back_ay, back_az, down_ax, down_ay, down_az, follow_ax, follow_ay, follow_az, \
    ax_shift, ay_shift, az_shift, top_idx, impact_idx, text_str, temp_dir = args
    
    # Create figure for this frame
    fig = plt.figure(figsize=(20, 6), dpi=100)
    ax_top   = fig.add_subplot(141, projection='3d')
    ax_rear  = fig.add_subplot(142, projection='3d')
    ax_front = fig.add_subplot(143, projection='3d')
    ax_full  = fig.add_subplot(144, projection='3d')

    # Limits (precomputed for consistency)
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

    ax_top.view_init(90, -90)
    ax_rear.view_init(-90, 90)
    ax_front.view_init(-90, -90)
    ax_full.view_init(30, -60)
    

    ax_top.set_title("Side View")
    ax_rear.set_title("Rear View")
    ax_front.set_title("Front View")
    ax_full.set_title("3D Full Trajectory")
    

    fig.text(0.5, 0.02, text_str, fontsize=12, ha='center', va='center')
    

    # Cumulative indices
    idx_back = min(i + 1, len(back_ax))
    idx_down = min(i - top_idx + 1, len(down_ax)) if i >= top_idx else 0
    idx_follow = min(i - impact_idx + 1, len(follow_ax)) if i >= impact_idx else 0
    

    # Plot Top (Side)
    ax_top.plot(back_ax[:idx_back], back_ay[:idx_back], back_az[:idx_back], "r-", lw=1)
    ax_top.plot(down_ax[:idx_down], down_ay[:idx_down], down_az[:idx_down], "g-", lw=1)
    ax_top.plot(follow_ax[:idx_follow], follow_ay[:idx_follow], follow_az[:idx_follow], "b-", lw=1)
    ax_top.plot([ax_shift[i]], [ay_shift[i]], [az_shift[i]], "ko")
    

    # Plot Rear
    ax_rear.plot(-back_az[:idx_back] + 1, back_ay[:idx_back], back_ax[:idx_back] + 1, "r-", lw=1)
    ax_rear.plot(-down_az[:idx_down] + 1, down_ay[:idx_down], down_ax[:idx_down] + 1, "g-", lw=1)
    ax_rear.plot(-follow_az[:idx_follow] + 1, follow_ay[:idx_follow], follow_ax[:idx_follow] + 1, "b-", lw=1)
    ax_rear.plot([-az_shift[i] + 1], [ay_shift[i]], [ax_shift[i] + 1], "ko")
    

    # Plot Front
    ax_front.plot(back_az[:idx_back] + 1, back_ay[:idx_back], back_ax[:idx_back] + 1, "r-", lw=1)
    ax_front.plot(down_az[:idx_down] + 1, down_ay[:idx_down], down_ax[:idx_down] + 1, "g-", lw=1)
    ax_front.plot(follow_az[:idx_follow] + 1, follow_ay[:idx_follow], follow_ax[:idx_follow] + 1, "b-", lw=1)
    ax_front.plot([az_shift[i] + 1], [ay_shift[i]], [ax_shift[i] + 1], "ko")
    

    # Plot Full with labels to avoid legend warning
    ax_full.plot(back_ax[:idx_back], back_ay[:idx_back], back_az[:idx_back], "r-", lw=1, label="Backswing")
    ax_full.plot(down_ax[:idx_down], down_ay[:idx_down], down_az[:idx_down], "g-", lw=1, label="Downswing")
    ax_full.plot(follow_ax[:idx_follow], follow_ay[:idx_follow], follow_az[:idx_follow], "b-", lw=1, label="Follow-through")
    ax_full.plot([ax_shift[i]], [ay_shift[i]], [az_shift[i]], "ko")
    ax_full.legend(loc='upper right')


    # Save frame
    frame_path = temp_dir / f"frame_{i:04d}.png"
    fig.savefig(frame_path, bbox_inches='tight', dpi=100)
    plt.close(fig)

    return str(frame_path)


def animate_imu_low_accel(csv_file=f"{BASE_PATH}/raw.csv", out_file="imu_low_accelfilt.gif", downsample_factor=6, num_threads=3):
    
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend (no Tkinter)
    import matplotlib.pyplot as plt
    
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found. Ensure data is saved correctly.")
        return

    # Downsample data
    step = downsample_factor
    df = df.iloc[::step].reset_index(drop=True)
    num_frames = len(df)
    print(f"[INFO] Downsampled to {num_frames} frames (factor={step}) for speed.")

    # Extract and process data (using your updated logic for indices)
    t = df["time"].values
    ax = df["Ax_filt"].values
    ay = df["Ay_filt"].values
    az = df["Az_filt"].values

    df['vibe_mag'] = np.sqrt(df['high_ax']**2 + df['high_ay']**2 + df['high_az']**2)
    impact_idx = df['vibe_mag'].argmax()

    ax_shift = ax - np.min(ax) if np.min(ax) < 0 else ax
    ay_shift = ay - np.min(ay) if np.min(ay) < 0 else ay
    az_shift = az - np.min(az) if np.min(az) < 0 else az

    pre_impact_df = df.iloc[:impact_idx + 1]
    gz_sign = np.sign(pre_impact_df['Gz'].values)
    sign_diff = np.diff(gz_sign)
    sign_change_idxs = np.where(sign_diff != 0)[0]
    top_idx = sign_change_idxs[-1] + 1 if len(sign_change_idxs) > 0 else 0

    back_ax, back_ay, back_az = ax_shift[:top_idx + 1].copy(), ay_shift[:top_idx + 1].copy(), az_shift[:top_idx + 1].copy()
    down_ax, down_ay, down_az = ax_shift[top_idx:impact_idx + 1].copy(), ay_shift[top_idx:impact_idx + 1].copy(), az_shift[top_idx:impact_idx + 1].copy()
    follow_ax, follow_ay, follow_az = ax_shift[impact_idx:].copy(), ay_shift[impact_idx:].copy(), az_shift[impact_idx:].copy()

    # Cache impact info
    impact_time, face_angle, impact_speed = get_Impact_time_face_angle_velocity()
    if impact_time is None:
        return
    text_str = f"Time: {impact_time:.6f}s  |  Speed: {impact_speed:.3f} m/s  |  Angle: {face_angle:.1f}°"

    # Temp dir for frames
    temp_dir = Path("temp_frames")
    temp_dir.mkdir(exist_ok=True)

    # Prepare args for each frame
    render_args = []
    for i in range(num_frames):
        args = (i, back_ax, back_ay, back_az, down_ax, down_ay, down_az,
                follow_ax, follow_ay, follow_az, ax_shift, ay_shift, az_shift,
                top_idx, impact_idx, text_str, temp_dir)
        render_args.append(args)

    # Parallel rendering
    print(f"[INFO] Rendering {num_frames} frames using {num_threads} threads...")
    with Pool(num_threads) as pool:
        frame_paths = pool.map(render_frame, render_args)
    pool.close()
    pool.join()

    # Compile GIF using imwrite
    print("[INFO] Compiling GIF...")
    fps = 15  # Balanced for smoothness
    images = [iio.imread(p) for p in sorted(frame_paths)]
    iio.imwrite(out_file, images, fps=fps)
    print(f"[OK] Animation saved as {out_file} ({num_frames} frames at {fps} FPS)")

    # Cleanup
    for path in frame_paths:
        os.remove(path)
    temp_dir.rmdir()
    
    
    
    
    
def plot_graphs(t_arr, imu_low_accel_arr, imu_gyro, ax_filt,ay_filt,az_filt, gx_filt,gy_filt,gz_filt, vel_global, imu_high_accel_arr, pos_shift, euler_deg, imu_euler):
    
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    
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



