#!/usr/bin/env python3
"""Regenerate the derived metric tables in data/metrics/ from the recorded runs.

The analysis pipeline that produced the report wrote most of its tables as CSV.
Two quantities the report quotes (the EKF RMSE over samples with a valid Vicon
fix, and the final distance to the target pose) exist only inside the MAT
struct, so this script exports them to CSV to keep every published number
machine-checkable.

    python tools/export_metric_tables.py
"""
from __future__ import annotations

import csv
import os

import numpy as np
import scipy.io as sio

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")


def export_task3():
    src = os.path.join(DATA, "real", "task3", "real_task3_from_full.mat")
    d = sio.loadmat(src, squeeze_me=True, struct_as_record=False)["Dreal_task3"]
    m = d.metrics
    rows = [
        ("ekf_rmse_valid_m", float(m.ekf_rmse_valid_m), "m", "EKF RMSE over samples with a valid Vicon fix"),
        ("ekf_rmse_lost_m", float(m.ekf_rmse_lost_m), "m", "EKF RMSE over samples with no Vicon fix"),
        ("ekf_rmse_m", float(m.ekf_rmse_m), "m", "EKF RMSE over the whole run"),
        ("ekf_max_m", float(m.ekf_max_m), "m", "maximum EKF position error"),
        ("imu_rmse_m", float(m.imu_rmse_m), "m", "dead-reckoning (IMU/odometry) RMSE"),
        ("target_final_m", float(m.target_final_m), "m", "final distance to the target pose"),
        ("target_rmse_m", float(m.target_rmse_m), "m", "RMSE against the target trajectory"),
        ("vicon_loss_percent", float(m.vicon_loss_percent), "%", "share of samples without a Vicon fix"),
        ("valid_vicon_percent", float(m.valid_vicon_percent), "%", "share of samples with a Vicon fix"),
        ("max_wheel_cmd_rad_s", float(m.max_wheel_cmd_rad_s), "rad/s", "peak commanded wheel speed"),
        ("duration_s", float(m.duration_s), "s", "run duration"),
    ]
    out = os.path.join(DATA, "metrics", "task3_filter_metrics.csv")
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["Quantity", "Value", "Unit", "Note"])
        for name, val, unit, note in rows:
            w.writerow([name, f"{val:.17g}", unit, note])
    print(f"wrote {out} ({len(rows)} rows)")


if __name__ == "__main__":
    export_task3()
