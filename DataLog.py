from typing import Dict, Callable
import mmap
from datetime import datetime
import numpy as np

from wpilib.logreader import DataLogReader

def load_data(log_path: str):
    
    def collate(fields):
        result = []
        for entry_idx in range(len(fields[0])):
            all_times = [f[entry_idx][0] for f in fields]
            all_data = [f[entry_idx][1] for f in fields]
            data_time = np.mean(all_times)
            result.append((data_time, all_data))
        return (np.array([r[0] for r in result]), np.array([r[1] for r in result]))

    # TODO: make this less hardocded
    accel_x, accel_y, accel_z = [], [], []
    ang_vel_x, ang_vel_y, ang_vel_z = [], [], []
    
    target_x, target_y, target_z, target_roll, target_pitch, target_yaw = [[]]*6
    
    enc_FL, enc_FR, enc_BL, enc_BR = [[]]*4
    pos_FL, pos_FR, pos_BL, pos_BR = [[]]*4
    
    with open(log_path, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            raise ValueError("not a log file")

        entries = {}
        for record in reader:
            timestamp = record.timestamp / 1000000
            
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
                    elif entry.name == 'BR encoder':
                        enc_BR.append((timestamp, data))
                    elif entry.name == 'FL position':
                        pos_FL.append((timestamp, data))
                    elif entry.name == 'FR position':
                        pos_FR.append((timestamp, data))
                    elif entry.name == 'BL position':
                        pos_BL.append((timestamp, data))
                    elif entry.name == 'BR position':
                        pos_BR.append((timestamp, data))
                        
                except TypeError:
                    print("  invalid")
    IMU = collate((accel_x, accel_y, accel_z, ang_vel_x, ang_vel_y, ang_vel_z))
    encoders = collate((enc_FL, enc_FR, enc_BL, enc_BR, pos_FL, pos_FR, pos_BL, pos_BR))
    tags = collate((target_x, target_y, target_z, target_roll, target_pitch, target_yaw))
    
    return IMU, encoders, tags
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file")
    args = parser.parse_args()
    
    
    IMU, encoders, tags = load_data(args.log_file)