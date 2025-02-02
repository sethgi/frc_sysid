import csv
import os
from typing import Dict, Callable
import mmap
from datetime import datetime
import numpy as np

from wpilib.logreader import DataLogReader

def load_data(log_path: str, time_limit: float = None):
    
    def collate(fields):
        fields = [sorted(f) for f in fields]
        result = []
        
        for entry_idx in range(min(len(f) for f in fields)):
            all_times = [f[entry_idx][0] for f in fields]
            all_data = [f[entry_idx][1] for f in fields]
            data_time = np.mean(all_times)
            if time_limit is not None and data_time > time_limit:
                break
            result.append((data_time, all_data))
        return (np.array([r[0] for r in result]), np.array([r[1] for r in result]))

    # TODO: make this less hardocded
    accel_x, accel_y, accel_z = [], [], []
    ang_vel_x, ang_vel_y, ang_vel_z = [], [], []
    
    target_x, target_y, target_z, target_roll, target_pitch, target_yaw = [],[],[],[],[],[]
    
    enc_FL, enc_FR, enc_BL, enc_BR = [],[],[],[]
    pos_FL, pos_FR, pos_BL, pos_BR = [],[],[],[]
    
    botposes = []

    with open(log_path, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            raise ValueError("not a log file")

        entries = {}
        for record in reader:
            timestamp = record.timestamp / 1_000_000
            
            if record.isStart():
                try:
                    data = record.getStartData()
                    entries[data.entry] = data
                except TypeError:
                    print("Start(INVALID)")
                    
            elif record.isFinish():
                try:
                    entry = record.getFinishEntry()
                    if entry not in entries:
                        print("...ID not found")
                    else:
                        del entries[entry]
                except TypeError:
                    print("Finish(INVALID)")
                    
            elif record.isSetMetadata():
                try:
                    data = record.getSetMetadataData()
                    if data.entry not in entries:
                        print("...ID not found")
                except TypeError:
                    print("SetMetadata(INVALID)")
            elif record.isControl():
                print("Unrecognized control record")
            else:
                entry = entries.get(record.entry)
                if entry is None:
                    print("<ID not found>")
                    continue
                
                try:
                    # handle systemTime specially
                    if entry.name == "systemTime" and entry.type == "int64":
                        dt = datetime.fromtimestamp(record.getInteger() / 1000000)
                        continue

                    prev_len = len(pos_FL)
                    if entry.type == "double":
                        data = record.getDouble()
                    elif entry.type == "int64":
                        data = record.getInteger()
                    elif entry.type in ("string", "json"):
                        data = record.getString()
                    elif entry.type == "msgpack":
                        data = record.getMsgPack()
                    elif entry.type == "boolean":
                        data = record.getBoolean()
                    elif entry.type == "boolean[]":
                        data = record.getBooleanArray()
                    elif entry.type == "double[]":
                        data = record.getDoubleArray()
                    elif entry.type == "float[]":
                        data = record.getFloatArray()
                    elif entry.type == "int64[]":
                        data = record.getIntegerArray()
                    elif entry.type == "string[]":
                        data = record.getStringArray()

                    # Populate the appropriate array
                    if entry.name == 'Pidgeon Acceleration X':
                        accel_x.append((timestamp, data))
                    elif entry.name == 'Pidgeon Acceleration Y':
                        accel_y.append((timestamp, data))
                    elif entry.name == 'Pidgeon Acceleration Z':
                        accel_z.append((timestamp, data))
                    elif entry.name == 'Pidgeon Angular Velocity X':
                        ang_vel_x.append((timestamp, data))
                    elif entry.name == 'Pidgeon Angular Velocity Y':
                        ang_vel_y.append((timestamp, data))
                    elif entry.name == 'Pidgeon Angular Velocity Z':
                        ang_vel_z.append((timestamp, data))
                    elif entry.name == 'Target X':
                        target_x.append((timestamp, data))
                    elif entry.name == 'Target Y':
                        target_y.append((timestamp, data))
                    elif entry.name == 'Target Z':
                        target_z.append((timestamp, data))
                    elif entry.name == 'Target Roll':
                        target_roll.append((timestamp, data))
                    elif entry.name == 'Target Pitch':
                        target_pitch.append((timestamp, data))
                    elif entry.name == 'Target Yaw':
                        target_yaw.append((timestamp, data))
                    elif entry.name == 'FL encoder position':
                        enc_FL.append((timestamp, data))
                    elif entry.name == 'FR encoder position':
                        enc_FR.append((timestamp, data))
                    elif entry.name == 'BL encoder position':
                        enc_BL.append((timestamp, data))
                    elif entry.name == 'BR encoder position':
                        enc_BR.append((timestamp, data))
                    elif entry.name == 'FL position':
                        pos_FL.append((timestamp, data))
                    elif entry.name == 'FR position':
                        pos_FR.append((timestamp, data))
                    elif entry.name == 'BL position':
                        pos_BL.append((timestamp, data))
                    elif entry.name == 'BR position':
                        pos_BR.append((timestamp, data))
                        
                    elif entry.name == 'NT:/limelight/botpose':
                        botposes.append((timestamp, data))
                        
                    if prev_len != len(pos_FL) and entry.name != "FL position":
                        breakpoint()
                        print(len(pos_FL))
                    
                    prev_len = len(pos_FL)
                        
                except TypeError:
                    print("  invalid")
    IMU = collate((accel_x, accel_y, accel_z, ang_vel_x, ang_vel_y, ang_vel_z))
    encoders = collate((enc_FL, enc_FR, enc_BL, enc_BR, pos_FL, pos_FR, pos_BL, pos_BR))
    tags = collate((target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
    
    botposes = [b for b in botposes if b[0] < time_limit]
    
    return IMU, encoders, tags, botposes
    

def dump_data_to_csv(log_file, out_dir, time_limit: float = None):
    IMU, encoders, tags, botposes = load_data(log_file, time_limit)

    
    os.makedirs(out_dir, exist_ok=True)
    
    # Dump IMU data to CSV (columns: time, accel_x, accel_y, accel_z, ang_vel_x, ang_vel_y, ang_vel_z)
    with open(f"{out_dir}/imu.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "accel_x", "accel_y", "accel_z", "ang_vel_x", "ang_vel_y", "ang_vel_z"])
        for t, data in zip(IMU[0], IMU[1]):
            writer.writerow([t] + list(data))
    
    # Dump encoder data to CSV (columns: time, enc_FL, enc_FR, enc_BL, enc_BR, pos_FL, pos_FR, pos_BL, pos_BR)
    with open(f"{out_dir}/encoders.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "enc_FL", "enc_FR", "enc_BL", "enc_BR", "pos_FL", "pos_FR", "pos_BL", "pos_BR"])
        for t, data in zip(encoders[0], encoders[1]):
            writer.writerow([t] + list(data))
    
    # Dump tag data to CSV (columns: time, target_x, target_y, target_z, target_roll, target_pitch, target_yaw)
    with open(f"{out_dir}/tags.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "target_x", "target_y", "target_z", "target_roll", "target_pitch", "target_yaw"])
        for t, data in zip(tags[0], tags[1]):
            if np.sum(np.abs(data)) > 1e-3:
                writer.writerow([t] + list(data))
                
    with open(f"{out_dir}/botposes.csv", "w", newline="") as f:
        writer = csv.writer(f)
        header=["time", "x", "y", "z", "roll", "pitch", "yaw", "latency", "tag_count", "tag_span", "avg_dist", "avg_area"]
        writer.writerow(header)
        for t, data in botposes:
            if np.sum(np.abs(data)) > 1e-3:
                writer.writerow([t] + list(data[:len(header)-1]))
                
    
    print("CSV files dumped: imu.csv, encoders.csv, botposes.csv, and tags.csv")