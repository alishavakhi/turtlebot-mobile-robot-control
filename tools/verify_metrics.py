#!/usr/bin/env python3
"""Recompute the reported metrics of the Final Activity from the recorded data.

MATLAB and Simulink are not available where this repository was prepared, so
nothing here runs a model.  What it does instead is independent of MATLAB:
every metric below is recomputed in NumPy from the exported logs that the
analysis pipeline produced, and compared against the value stored in the
metric table under ``data/metrics/``.

If a number in the README, in the report or in ``data/metrics/`` does not come
out of the data, this script fails — that is the point.

Usage
-----
    pip install -r requirements.txt
    python tools/verify_metrics.py            # human-readable report
    python tools/verify_metrics.py --quiet    # exit status only

Exit status: 0 if every check passes, 1 otherwise.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import os
import subprocess
import sys
import zipfile

import numpy as np
import scipy.io as sio

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
METRICS = os.path.join(DATA, "metrics")
SRC = os.path.join(ROOT, "src")

GOAL_T1 = np.array([-1.60, -1.10, 0.0])
GOAL_T2 = np.array([1.00, -0.30, 0.0])
WHEEL_LIMIT = 10.0          # rad/s, lab actuator limit
RED_X = (-0.60, 0.20)       # noisy-Vicon region (from the activity map)
RED_Y = (-1.50, 0.10)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def load_struct(path, key):
    """Load a plain MAT struct (what the analysis pipeline exported)."""
    d = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    return d[key]


def read_csv(name):
    with io.open(os.path.join(METRICS, name), encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class Report:
    def __init__(self):
        self.rows = []

    def check(self, name, computed, expected, tol=1e-6, unit=""):
        ok = bool(abs(computed - expected) <= tol * max(1.0, abs(expected)))
        self.rows.append((name, computed, expected, unit, ok))
        return ok

    def check_true(self, name, condition, detail=""):
        self.rows.append((name, detail, "expected true", "", bool(condition)))
        return bool(condition)

    @property
    def failures(self):
        return [r for r in self.rows if not r[4]]

    def print(self):
        print(f"{'check':<58}{'computed':>16}{'stored':>16}   unit  ok")
        print("-" * 106)
        for name, comp, exp, unit, ok in self.rows:
            c = f"{comp:.6f}" if isinstance(comp, float) else str(comp)
            e = f"{exp:.6f}" if isinstance(exp, float) else str(exp)
            print(f"{name:<58}{c:>16}{e:>16}   {unit:<5} {'PASS' if ok else 'FAIL'}")
        print("-" * 106)
        print(f"{len(self.rows) - len(self.failures)}/{len(self.rows)} checks passed")


def transposed(v):
    """Some exports store state arrays as (3, N); return (N, 3)."""
    v = np.asarray(v, dtype=float)
    return v.T if v.shape[0] == 3 and v.shape[1] > 3 else v


# --------------------------------------------------------------------------
# Task 1 — posture regulation, three real runs
# --------------------------------------------------------------------------
def task1(rep: Report):
    csv_rows = {r["Run"]: r for r in read_csv("task1_three_real_runs.csv")}
    for run in ("1", "2", "3"):
        path = os.path.join(DATA, "real", "task1", f"test{run}_task1_separated.mat")
        d = load_struct(path, "D_task1")
        qf = np.asarray(d.qf_vicon_task1, dtype=float)
        ep = float(np.hypot(GOAL_T1[0] - qf[0], GOAL_T1[1] - qf[1]))
        eth = float(abs(wrap(GOAL_T1[2] - qf[2])))
        rep.check(f"task1 run {run}: final Vicon position error", ep,
                  float(csv_rows[run]["final_position_error_m"]), unit="m")
        rep.check(f"task1 run {run}: final Vicon heading error", eth,
                  float(csv_rows[run]["final_orientation_error_rad"]), unit="rad")

    # motor limit on the representative run
    d3 = load_struct(os.path.join(DATA, "real", "task1", "test3_task1_separated.mat"), "D_task1")
    w = np.asarray(d3.omega_wheels, dtype=float)
    wmax = float(np.max(np.abs(w)))
    rep.check_true("task1 run 3: commanded wheel speed inside the 10 rad/s limit",
                   wmax <= WHEEL_LIMIT + 1e-9, f"max |omega_wheel| = {wmax:.4f} rad/s")

    # the report's summary table must agree with the run-3 recomputation
    summary = read_csv("task1_summary.csv")[0]
    qf = np.asarray(d3.qf_vicon_task1, dtype=float)
    rep.check("task1 summary table: final position error",
              float(np.hypot(GOAL_T1[0] - qf[0], GOAL_T1[1] - qf[1])),
              float(summary["ViconFinalPosErr_m"]), unit="m")
    rep.check("task1 summary table: final heading error",
              float(abs(wrap(GOAL_T1[2] - qf[2]))),
              float(summary["ViconFinalThetaErr_rad"]), unit="rad")

    # all three runs reached the 0.05 m position threshold, none the heading one
    eps = [float(r["final_position_error_m"]) for r in csv_rows.values()]
    eths = [float(r["final_orientation_error_rad"]) for r in csv_rows.values()]
    rep.check_true("task1: every real run met the 0.05 m position threshold",
                   all(e < 0.05 for e in eps), f"max = {max(eps):.4f} m")
    rep.check_true("task1: no real run met the heading threshold (as the report states)",
                   all(t > 0.05 for t in eths), f"min = {min(eths):.4f} rad")


# --------------------------------------------------------------------------
# Task 2 — navigation through the noisy-Vicon region
# --------------------------------------------------------------------------
def task2(rep: Report):
    stored = read_csv("task2_report_numbers.csv")[0]
    d = load_struct(os.path.join(DATA, "real", "task2", "test3_task2_full_outputs.mat"), "D_task2")

    q_est = transposed(d.q_est)
    ref = np.asarray(d.Y_viscon_ground_trith, dtype=float)
    err = np.hypot(q_est[:, 0] - ref[:, 0], q_est[:, 1] - ref[:, 1])

    rep.check("task2: localization RMSE (whole run)", float(np.sqrt(np.mean(err ** 2))),
              float(stored["RealLocalizationRMSE_m"]), unit="m")
    rep.check("task2: maximum localization error", float(np.max(err)),
              float(stored["RealLocalizationMaxError_m"]), unit="m")

    red = ((ref[:, 0] >= RED_X[0]) & (ref[:, 0] <= RED_X[1])
           & (ref[:, 1] >= RED_Y[0]) & (ref[:, 1] <= RED_Y[1]))
    rep.check("task2: localization RMSE inside the noisy (red) region",
              float(np.sqrt(np.mean(err[red] ** 2))),
              float(stored["RealRedAreaRMSE_m"]), unit="m")

    qf = ref[np.isfinite(ref[:, 0])][-1]
    rep.check("task2: final position error at the goal",
              float(np.hypot(GOAL_T2[0] - qf[0], GOAL_T2[1] - qf[1])),
              float(stored["RealFinalPositionError_m"]), unit="m")
    rep.check("task2: final heading error at the goal",
              float(abs(wrap(GOAL_T2[2] - qf[2]))),
              float(stored["RealFinalThetaError_rad"]), unit="rad")

    w = np.asarray(d.omega_wheels, dtype=float)
    rep.check("task2: peak commanded wheel speed", float(np.max(np.abs(w))),
              float(stored["MaxRealCommandedWheel_rad_s"]), unit="rad/s")
    rep.check_true("task2: measurement loss in this run was 0/509 samples (as reported)",
                   int(float(stored["ViconLossSamples"])) == 0,
                   f"samples = {int(float(stored['ViconTotalSamples']))}")


# --------------------------------------------------------------------------
# Task 3 — circular tracking in the 90 % Vicon-loss region
# --------------------------------------------------------------------------
def task3(rep: Report):
    stored = read_csv("task3_real_vs_sim.csv")[0]
    d = load_struct(os.path.join(DATA, "real", "task3", "real_task3_from_full.mat"), "Dreal_task3")

    q_est = transposed(d.q_est)
    gt = np.asarray(d.Y_vicon_ground_truth, dtype=float)
    valid = np.asarray(d.vicon_valid).astype(bool) & np.isfinite(gt[:, 0])
    err = np.hypot(q_est[:, 0] - gt[:, 0], q_est[:, 1] - gt[:, 1])

    rep.check("task3: EKF localization RMSE over the samples with a valid Vicon fix",
              float(np.sqrt(np.mean(err[valid] ** 2))),
              float(d.metrics.ekf_rmse_valid_m), unit="m")
    rep.check("task3: share of the run without a Vicon fix",
              100.0 * (1.0 - valid.mean()),
              float(stored["RealViconLoss_percent"]), tol=5e-3, unit="%")

    qf = gt[valid][-1]
    target = np.asarray(d.q_target, dtype=float)
    target = transposed(target)[-1] if target.shape[0] == 3 and target.size > 3 else target
    tgt = np.asarray(d.q_target, dtype=float)
    tgt_last = tgt[-1] if tgt.ndim == 2 and tgt.shape[1] == 3 else tgt[:, -1]
    rep.check("task3: final distance to the target pose",
              float(np.hypot(tgt_last[0] - qf[0], tgt_last[1] - qf[1])),
              float(d.metrics.target_final_m), unit="m")

    # the stored metric field must also equal the reported one
    rep.check("task3: stored EKF RMSE equals the value printed in the report",
              float(d.metrics.ekf_rmse_m), float(stored["RealEKFRMSE_m"]), unit="m")
    rep.check("task3: peak commanded wheel speed",
              float(np.max(np.abs(transposed(d.omega_wheels)))),
              float(stored["RealMaxWheelCmd_rad_s"]), unit="rad/s")


# --------------------------------------------------------------------------
# Task 4 — LiDAR EKF without motion capture (not recomputable here)
# --------------------------------------------------------------------------
def task4(rep: Report):
    files = sorted(f for f in os.listdir(os.path.join(DATA, "simulation"))
                   if f.startswith("task4_") and f.endswith(".mat"))
    rep.check_true("task4: all five simulation exports are present", len(files) == 5,
                   ", ".join(files))
    # Their payload is a Simulink.SimulationOutput object stored as an opaque
    # MCOS value; SciPy cannot read it.  This is stated instead of faked.
    opaque = 0
    for f in files:
        d = sio.loadmat(os.path.join(DATA, "simulation", f), squeeze_me=True,
                        struct_as_record=False)
        if type(d.get("out")).__name__ == "MatlabOpaque":
            opaque += 1
    rep.check_true("task4: exports are opaque SimulationOutput objects (not recomputed)",
                   opaque == len(files), f"{opaque}/{len(files)} opaque")


# --------------------------------------------------------------------------
# integrity — the published model files and the extracted code
# --------------------------------------------------------------------------
def integrity(rep: Report):
    intake_manifest = os.path.join(ROOT, "docs", "PROVENANCE.sha256")
    if os.path.exists(intake_manifest):
        bad = []
        for line in io.open(intake_manifest, encoding="utf-8"):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            digest, path = line.split("  ", 1)
            full = os.path.join(ROOT, path)
            if not os.path.exists(full) or sha256(full) != digest:
                bad.append(path)
        rep.check_true("integrity: every file matches docs/PROVENANCE.sha256",
                       not bad, f"{len(bad)} mismatching file(s)")

    # re-extracting the embedded MATLAB code must reproduce src/model_code/ byte for byte
    ok, detail = True, "all models reproduce the committed extraction"
    for model, folder in (("task1_simulation.slx", "task1"),
                          ("task2_simulation.slx", "task2"),
                          ("task3_simulation.slx", "task3"),
                          ("task4_simulation.slx", "task4"),
                          ("task4_simulation_analytic_jacobian.slx", "task4_analytic"),
                          ("online_robot_2024b.slx", "online")):
        slx = os.path.join(SRC, "simulink", model)
        tmp = os.path.join("/tmp", f"rl_verify_{folder}")
        subprocess.run([sys.executable, os.path.join(ROOT, "tools", "extract_model_code.py"),
                        slx, tmp, "--name", folder],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        for fn in sorted(os.listdir(tmp)):
            a = os.path.join(tmp, fn)
            b = os.path.join(SRC, "model_code", folder, fn)
            if not os.path.exists(b) or sha256(a) != sha256(b):
                ok, detail = False, f"mismatch in {folder}/{fn}"
                break
    rep.check_true("integrity: model code re-extracts identically from the .slx files",
                   ok, detail)

    # every Simulink model is a readable zip with a block diagram
    ok_models = True
    for fn in sorted(os.listdir(os.path.join(SRC, "simulink"))):
        path = os.path.join(SRC, "simulink", fn)
        try:
            with zipfile.ZipFile(path) as z:
                ok_models &= any(n.startswith("simulink/") for n in z.namelist())
        except zipfile.BadZipFile:
            ok_models = False
    rep.check_true("integrity: every .slx file is a readable Simulink package", ok_models)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    rep = Report()
    task1(rep)
    task2(rep)
    task3(rep)
    task4(rep)
    integrity(rep)

    if not args.quiet:
        rep.print()
    if rep.failures:
        print(f"\n{len(rep.failures)} CHECK(S) FAILED", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
