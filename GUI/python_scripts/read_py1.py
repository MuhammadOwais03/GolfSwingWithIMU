
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy import signal, integrate
from scipy.spatial.transform import Rotation as R
import serial
import time
import pandas as pd
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing.dummy import Pool
import imageio.v3 as iio  # pip install imageio[ffmpeg] if needed
from pathlib import Path
import os 
from PIL import Image




BASE_PATH = "python_scripts/data"


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
        print(self.ser.readline().decode())
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

# python_scripts/data/raw.csv






