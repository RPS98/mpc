#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Plot results from the MPC + MAV Simulator integrated example CSV log.

Reference step transitions (waypoint changes) are detected automatically from
the CSV: either from the waypoint_index column or from sudden jumps in the
reference position. Rise time and settling time are computed independently
for each segment.

Metrics can be pre-computed with compute_metrics.py and passed via -m/-m2 to
skip recomputation. When a log CSV is also provided alongside the metrics CSV,
raw signals (acceleration, jerk) are recomputed for the smoothness figure.

Usage (single file, compute metrics on-the-fly):
    python3 plot_results.py -f mpc_log.csv

Usage (pre-computed metrics, save PNGs):
    python3 plot_results.py -f mpc_log.csv -m mpc_log_metrics.csv --save

Usage (comparison with pre-computed metrics):
    python3 plot_results.py \\
        -f  mpc_log.csv  -m  mpc_log_metrics.csv  --label1 MPC \\
        -f2 pid_log.csv  -m2 pid_log_metrics.csv  --label2 PID \\
        --save
"""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import csv
import dataclasses
import os
import sys
from collections import defaultdict
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Metrics configuration — edit these defaults to tune metric computation
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class MetricsConfig:
    """Tuneable parameters for control performance metrics."""

    # Settling time threshold (absolute, metres).
    # Settling time = elapsed time until 3D error first drops below this value.
    # Set to match the waypoint acceptance radius of the simulation (0.1 m by
    # default) so that every segment receives an annotation. Tighten for a
    # more demanding criterion on the final hover phase.
    settling_threshold_m: float = 0.1

    # Rise time: time for 3D error to drop from upper to lower % of d0.
    rise_time_upper_pct: float = 90.0
    rise_time_lower_pct: float = 10.0

    # Steady-state error window: fraction of the settled window (from t_settle
    # to segment end) used for RMSE. 1.0 = full window; 0.5 = latter half.
    steady_state_hover_fraction: float = 1.0

    # Maximum speed for constraint-violation metric (m/s).
    # Auto-read from the CSV 'max_speed' column when not provided here.
    # None disables the metric.
    max_speed: Optional[float] = None

    # Velocity smoothing window (samples) applied before computing
    # acceleration and jerk. Larger values reduce differentiation spikes.
    smoothing_window: int = 11

    # Minimum reference displacement (m) to count as a waypoint transition.
    min_step_distance: float = 0.05


# Global instance — change here to adjust defaults without touching the logic.
METRICS_CFG = MetricsConfig()

# ---------------------------------------------------------------------------
# CSV I/O
# ---------------------------------------------------------------------------

PRINT_ERROR = True


def read_csv(file_path: str) -> dict:
    """Read a CSV log and return a dict mapping column name → list[float]."""
    data: dict = defaultdict(list)
    with open(file_path, mode='r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                if isinstance(value, str) and value.strip():
                    data[key].append(float(value))

    if not data:
        return data

    time_len = len(data['time'])
    for key in data:
        if len(data[key]) != time_len:
            print(f'ERROR: key {key} has length {len(data[key])}, expected {time_len}')

    return data


# ---------------------------------------------------------------------------
# Array helpers
# ---------------------------------------------------------------------------


def _np(series) -> np.ndarray:
    return np.asarray(series, dtype=float)


def get_series(data: dict, key: str, fallback_keys=None):
    """Return a data series using fallback keys when the primary is absent."""
    if fallback_keys is None:
        fallback_keys = []
    for candidate in [key] + fallback_keys:
        if candidate in data and len(data[candidate]) > 0:
            return data[candidate], candidate
    return [], key


def compute_mean_error(value_gt, value_ref) -> float:
    return float(np.mean(np.abs(_np(value_gt) - _np(value_ref))))


def _smooth(arr: np.ndarray, window: int) -> np.ndarray:
    """Symmetric box-car moving average (handles edges with 'same' mode)."""
    if window <= 1:
        return arr.copy()
    kernel = np.ones(window) / window
    return np.convolve(arr, kernel, mode='same')


# ---------------------------------------------------------------------------
# Segment detection
# ---------------------------------------------------------------------------


def detect_segments(data: dict, min_step_dist: float) -> list:
    """Detect reference step transitions and return a list of segment dicts.

    Each segment covers the period during which the drone tracks a single
    reference position (one waypoint). Segments are delimited by the instants
    when the reference position jumps to the next waypoint.

    Detection priority:
      1. waypoint_index column transitions (preferred — exact).
      2. Sudden jumps in (x_ref, y_ref, z_ref) > min_step_dist (fallback).

    Returns:
        List of dicts with keys: start_idx, end_idx, ref_pos (ndarray[3]).
    """
    x_ref = _np(data['x_ref'])
    y_ref = _np(data['y_ref'])
    z_ref = _np(data['z_ref'])
    n = len(x_ref)

    # Always detect segments from actual reference jumps in the CSV.
    # This is more reliable than waypoint_index because:
    #   - waypoint_index changes one outer-loop step before the reference
    #     actually updates (due to zero-order hold), creating spurious segments.
    #   - Reference jumps directly capture when the controller target changed.
    ref = np.column_stack([x_ref, y_ref, z_ref])
    ref_jump = np.linalg.norm(np.diff(ref, axis=0), axis=1)
    jump_idx = np.where(ref_jump > min_step_dist)[0] + 1
    boundaries = [0] + jump_idx.tolist() + [n]

    segments = []
    for i in range(len(boundaries) - 1):
        s, e = boundaries[i], boundaries[i + 1]
        if e - s < 2:
            continue
        segments.append({
            'start_idx': s,
            'end_idx': e,
            'ref_pos': np.array([x_ref[s], y_ref[s], z_ref[s]]),
        })
    return segments


# ---------------------------------------------------------------------------
# Per-segment metrics
# ---------------------------------------------------------------------------


def _compute_segment_metrics(seg: dict, t: np.ndarray, err3d: np.ndarray,
                              cfg: MetricsConfig) -> Optional[dict]:
    """Compute rise time, settling time, SSE and overshoot for one segment."""
    s, e = seg['start_idx'], seg['end_idx']
    seg_t = t[s:e]
    seg_err = err3d[s:e]

    if len(seg_t) < 3:
        return None

    d0 = max(float(seg_err[0]), 1e-6)
    settle_thr = cfg.settling_threshold_m   # absolute threshold (metres)
    upper_thr = (cfg.rise_time_upper_pct / 100.0) * d0
    lower_thr = (cfg.rise_time_lower_pct / 100.0) * d0
    t0 = float(seg_t[0])

    # Rise time: from first crossing of upper_thr down to first crossing of lower_thr
    below_upper = np.where(seg_err <= upper_thr)[0]
    below_lower = np.where(seg_err <= lower_thr)[0]
    if len(below_upper) > 0 and len(below_lower) > 0:
        rise_time = max(0.0, float(seg_t[below_lower[0]] - seg_t[below_upper[0]]))
    else:
        rise_time = None

    # Settling time: elapsed from t0 until 3D error first drops below settle_thr.
    # Using an absolute threshold (matching the waypoint acceptance radius) ensures
    # every segment receives an annotation, including intermediate approaches where
    # the drone transitions away before the error settles to a tight percentage.
    below_settle = np.where(seg_err <= settle_thr)[0]
    if len(below_settle) > 0:
        settling_time = float(seg_t[below_settle[0]] - t0)
        t_settle = float(seg_t[below_settle[0]])
    else:
        settling_time = None  # never reached threshold within this segment
        t_settle = None

    # Steady-state error: RMSE over the settled part of the segment.
    # Only meaningful when the error actually settled; otherwise None.
    # steady_state_hover_fraction controls how much of the settled window is used:
    # 1.0 = from t_settle to t_end; 0.5 = last half of that window.
    if settling_time is not None:
        settled_t = seg_t[seg_t >= t_settle]
        settled_e = seg_err[seg_t >= t_settle]
        n_settled = len(settled_t)
        skip = int(n_settled * (1.0 - cfg.steady_state_hover_fraction))
        used = settled_e[skip:]
        sse = float(np.sqrt(np.mean(used**2))) if len(used) > 0 else None
    else:
        sse = None  # drone did not settle within this segment

    # Overshoot: max rebound in 3D error after the minimum is reached
    min_idx = int(np.argmin(seg_err))
    overshoot = (float(np.max(seg_err[min_idx:]) - seg_err[min_idx])
                 if min_idx < len(seg_err) - 1 else 0.0)

    return {
        't_start': t0,
        't_end': float(seg_t[-1]),
        't_settle': t_settle,
        'rise_time': rise_time,
        'settling_time': settling_time,
        'steady_state_error': sse,
        'overshoot': overshoot,
        'd0': d0,
        'settle_thr': settle_thr,
        'ref_pos': seg['ref_pos'],
    }


# ---------------------------------------------------------------------------
# Aggregate metrics computation
# ---------------------------------------------------------------------------


def _compute_3d_error(data: dict) -> np.ndarray:
    pos = np.column_stack([_np(data['x']), _np(data['y']), _np(data['z'])])
    ref = np.column_stack([_np(data['x_ref']), _np(data['y_ref']), _np(data['z_ref'])])
    return np.linalg.norm(pos - ref, axis=1)


def compute_metrics(data: dict, cfg: MetricsConfig) -> dict:
    """Compute all control performance metrics from a CSV data dict.

    Segments are detected automatically from reference steps. All per-step
    metrics (rise time, settling time, SSE, overshoot) are computed per
    segment and then aggregated.

    Returns a dict with keys:
      compute_time_mean_us, compute_time_std_us,
      settling_time_s (mean), settling_time_max_s,
      steady_state_error_m,
      overshoot_m,
      rise_time_mean_s, rise_time_min_s, rise_time_max_s,
      jerk_energy, accel_energy,
      speed_violation_pct,
      seg_metrics   (list of per-segment result dicts, for plotting),
      _t, _err3d, _accel_t, _accel_norm, _jerk_t, _jerk_norm, _speed
    """
    t = _np(data['time'])
    err3d = _compute_3d_error(data)

    # --- 1. Compute time ---
    if 'controller_solve_time_us' in data:
        solve = _np(data['controller_solve_time_us'])
        valid = solve[solve > 0.0]
        compute_time_mean = float(np.mean(valid)) if len(valid) > 0 else None
        compute_time_std = float(np.std(valid)) if len(valid) > 0 else None
    else:
        compute_time_mean = compute_time_std = None

    # --- Segment-based metrics ---
    segments = detect_segments(data, cfg.min_step_distance)
    seg_metrics = [m for seg in segments
                   for m in [_compute_segment_metrics(seg, t, err3d, cfg)]
                   if m is not None]

    def _agg(key, agg='mean'):
        vals = [s[key] for s in seg_metrics if s.get(key) is not None]
        if not vals:
            return None
        return float({'mean': np.mean, 'max': np.max, 'min': np.min}[agg](vals))

    # --- 6. Jerk / accel (smooth velocity first to reduce differentiation noise) ---
    vx = _smooth(_np(data['vx']), cfg.smoothing_window)
    vy = _smooth(_np(data['vy']), cfg.smoothing_window)
    vz = _smooth(_np(data['vz']), cfg.smoothing_window)
    dt = np.diff(t)
    dt = np.where(dt > 0, dt, 1e-6)

    ax_s = np.diff(vx) / dt
    ay_s = np.diff(vy) / dt
    az_s = np.diff(vz) / dt
    accel_norm = np.sqrt(ax_s**2 + ay_s**2 + az_s**2)
    accel_energy = float(np.sum(accel_norm**2 * dt))

    jx = np.diff(ax_s) / dt[:-1]
    jy = np.diff(ay_s) / dt[:-1]
    jz = np.diff(az_s) / dt[:-1]
    jerk_norm = np.sqrt(jx**2 + jy**2 + jz**2)
    jerk_energy = float(np.sum(jerk_norm**2 * dt[:-1]))

    # --- 7. Speed constraint violation ---
    # max_speed priority: cfg override → CSV column → None (metric disabled)
    effective_max_speed = cfg.max_speed
    if effective_max_speed is None and 'max_speed' in data:
        csv_ms = _np(data['max_speed'])
        pos = csv_ms[csv_ms > 0.0]
        if len(pos) > 0:
            effective_max_speed = float(pos[0])

    speed = np.sqrt(_np(data['vx'])**2 + _np(data['vy'])**2 + _np(data['vz'])**2)
    speed_violation_pct = (
        float(100.0 * np.sum(speed > effective_max_speed) / len(speed))
        if effective_max_speed is not None else None
    )

    return {
        'compute_time_mean_us': compute_time_mean,
        'compute_time_std_us': compute_time_std,
        'settling_time_s': _agg('settling_time', 'mean'),
        'settling_time_max_s': _agg('settling_time', 'max'),
        'steady_state_error_m': _agg('steady_state_error', 'mean'),
        'overshoot_m': _agg('overshoot', 'max'),
        'rise_time_mean_s': _agg('rise_time', 'mean'),
        'rise_time_min_s': _agg('rise_time', 'min'),
        'rise_time_max_s': _agg('rise_time', 'max'),
        'jerk_energy': jerk_energy,
        'accel_energy': accel_energy,
        'speed_violation_pct': speed_violation_pct,
        '_effective_max_speed': effective_max_speed,
        '_settle_thr_m': cfg.settling_threshold_m,
        # Per-segment details for annotations
        'seg_metrics': seg_metrics,
        # Raw signals for smoothness figure
        '_t': t,
        '_err3d': err3d,
        '_accel_t': t[1:],
        '_accel_norm': accel_norm,
        '_jerk_t': t[2:],
        '_jerk_norm': jerk_norm,
        '_speed': speed,
    }


# ---------------------------------------------------------------------------
# Print helpers
# ---------------------------------------------------------------------------


def _fmt(value, fmt='.4f', unit='') -> str:
    if value is None:
        return 'N/A'
    return f'{value:{fmt}}{unit}'


def print_metrics(metrics: dict, label: str = '',
                  cfg: MetricsConfig = METRICS_CFG) -> None:
    """Print a per-segment table and aggregate metrics to stdout."""
    title = f'=== Performance Metrics [{label}] ===' if label else '=== Performance Metrics ==='
    print(f'\n{title}')

    # Per-segment table
    segs = metrics.get('seg_metrics', [])
    if segs:
        thr_m = cfg.settling_threshold_m
        up = cfg.rise_time_upper_pct
        lo = cfg.rise_time_lower_pct
        header = (f"  {'Seg':>3}  {'t_start':>8}  {'d0(m)':>7}  "
                  f"{'Rise(s)':>8}  {'Settle(s)':>9}  "
                  f"{'SSE(m)':>7}  {'Overshoot(m)':>12}")
        print(f'\n  Per-segment metrics  '
              f'[rise: {lo:.0f}%→{up:.0f}% of d0 | settle: err < {thr_m:.3f} m]')
        print(header)
        print('  ' + '-' * (len(header) - 2))
        for i, sm in enumerate(segs):
            rt = _fmt(sm['rise_time'], '.3f')
            st = _fmt(sm['settling_time'], '.3f')
            sse = _fmt(sm['steady_state_error'], '.4f')
            ov = _fmt(sm['overshoot'], '.4f')
            ref = sm['ref_pos']
            print(f"  {i:>3}  {sm['t_start']:>8.3f}  {sm['d0']:>7.3f}  "
                  f"{rt:>8}  {st:>9}  {sse:>7}  {ov:>12}  "
                  f"→[{ref[0]:.2f},{ref[1]:.2f},{ref[2]:.2f}]")

    # Aggregates
    print('\n  Aggregate:')
    ct_mean = metrics['compute_time_mean_us']
    ct_std = metrics['compute_time_std_us']
    if ct_mean is not None:
        print(f'  Compute time       : {ct_mean:.2f} ± {ct_std:.2f} µs')
    else:
        print('  Compute time       : N/A')

    st_mean = metrics['settling_time_s']
    st_max = metrics['settling_time_max_s']
    thr_m = cfg.settling_threshold_m
    print(f'  Settling time (<{thr_m:.3f}m): {_fmt(st_mean, ".3f", " s")} (mean), '
          f'{_fmt(st_max, ".3f", " s")} (max)')
    print(f'  Steady-state error : {_fmt(metrics["steady_state_error_m"], ".4f", " m")} (RMSE mean)')
    print(f'  Overshoot          : {_fmt(metrics["overshoot_m"], ".4f", " m")} (max 3D)')

    rt_mean = metrics['rise_time_mean_s']
    rt_min = metrics['rise_time_min_s']
    rt_max = metrics['rise_time_max_s']
    upper = cfg.rise_time_upper_pct
    lower = cfg.rise_time_lower_pct
    if rt_mean is not None:
        print(f'  Rise time          : {rt_mean:.3f} s (mean), '
              f'{rt_min:.3f}–{rt_max:.3f} s (range)  '
              f'[{lower:.0f}%→{upper:.0f}% of d0]')
    else:
        print(f'  Rise time          : N/A')

    print(f'  Jerk energy        : {_fmt(metrics["jerk_energy"], ".3f")} (m/s³)²·s')
    print(f'  Accel energy       : {_fmt(metrics["accel_energy"], ".3f")} (m/s²)²·s')

    sv = metrics['speed_violation_pct']
    ems = metrics.get('_effective_max_speed')
    if sv is not None and ems is not None:
        print(f'  Speed violation    : {sv:.2f} %  (||v|| > {ems:.2f} m/s)')
    elif sv is not None:
        print(f'  Speed violation    : {sv:.2f} %')
    else:
        print('  Speed violation    : N/A  (pass --max_speed to enable)')


def print_metrics_comparison(m1: dict, m2: dict,
                              label1: str = 'A', label2: str = 'B',
                              cfg: MetricsConfig = METRICS_CFG) -> None:
    """Print a side-by-side comparison table."""
    print(f'\n=== Metrics Comparison: [{label1}] vs [{label2}] ===')

    rows = [
        ('Compute time (µs)', 'compute_time_mean_us', '.2f'),
        ('Settling time mean (s)', 'settling_time_s', '.3f'),
        ('Settling time max (s)', 'settling_time_max_s', '.3f'),
        ('Steady-state err (m)', 'steady_state_error_m', '.4f'),
        ('Overshoot (m)', 'overshoot_m', '.4f'),
        ('Rise time (s)', 'rise_time_mean_s', '.3f'),
        ('Jerk energy', 'jerk_energy', '.3f'),
        ('Accel energy', 'accel_energy', '.3f'),
        ('Speed violation (%)', 'speed_violation_pct', '.2f'),
    ]

    col_w = max(len(label1), len(label2), 10)
    header = (f"  {'Metric':<28} {label1:>{col_w}}  "
              f"{label2:>{col_w}}  {'Δ (A-B)':>{col_w+2}}")
    print(header)
    print('  ' + '-' * (len(header) - 2))
    for name, key, fmt in rows:
        v1, v2 = m1.get(key), m2.get(key)
        s1 = _fmt(v1, fmt) if v1 is not None else 'N/A'
        s2 = _fmt(v2, fmt) if v2 is not None else 'N/A'
        s_delta = f'{v1-v2:{fmt}}' if (v1 is not None and v2 is not None) else 'N/A'
        print(f'  {name:<28} {s1:>{col_w}}  {s2:>{col_w}}  {s_delta:>{col_w+2}}')


# ---------------------------------------------------------------------------
# Plot annotation helpers
# ---------------------------------------------------------------------------


def annotate_segment_steps(ax, seg_metrics: list,
                           settle_label: str = 't_settle') -> None:
    """Draw reference step and settling time markers for all segments.

    - Gray dotted vertical line at each t_start (reference jump).
    - Purple dashed vertical line at each t_settle (first crossing of
      settling_threshold_m), present for every segment that reaches it.
    """
    first_step = True
    first_settle = True
    for sm in seg_metrics:
        lbl = 'ref step' if first_step else None
        ax.axvline(sm['t_start'], color='gray', linestyle=':', linewidth=0.8,
                   alpha=0.6, label=lbl)
        first_step = False

        if sm['t_settle'] is not None:
            lbl = settle_label if first_settle else None
            ax.axvline(sm['t_settle'], color='purple', linestyle='--',
                       linewidth=1.0, alpha=0.85, label=lbl)
            first_settle = False


# ---------------------------------------------------------------------------
# Core plot functions
# ---------------------------------------------------------------------------


def plot_values(data: dict, values: list, title: str, axs,
                metrics: Optional[dict] = None,
                color: Optional[str] = None,
                label_suffix: str = '') -> None:
    """Plot values vs time with reference overlay and segment annotations."""
    pos_keys = {'x', 'y', 'z'}

    for i, value in enumerate(values):
        series_gt, label_gt = get_series(data, value)
        if not series_gt:
            print(f"Warn: no data found for '{value}'")
            continue

        kw: dict = {'linestyle': 'solid'}
        if color:
            kw['color'] = color
        lbl = f'{label_gt}{label_suffix}' if label_suffix else label_gt
        axs[i].plot(data['time'], series_gt, label=lbl, **kw)

        fallback_ref = ([value + '_cmd', value + '_ref']
                        if value in ['thrust', 'wx', 'wy', 'wz'] else [])
        series_ref, label_ref = get_series(data, value + '_ref', fallback_ref)
        if series_ref:
            kw_ref: dict = {'linestyle': 'dotted'}
            if color:
                kw_ref['color'] = color
            lbl_ref = f'{label_ref}{label_suffix}' if label_suffix else label_ref
            axs[i].plot(data['time'], series_ref, label=lbl_ref, **kw_ref)
            if PRINT_ERROR and not label_suffix:
                print(f'  Mean error {value}: {compute_mean_error(series_gt, series_ref):.6f}')

        # Segment annotations for position channels (primary dataset only)
        if metrics is not None and not label_suffix and value in pos_keys:
            seg_ms = metrics.get('seg_metrics', [])
            thr_m = metrics.get('_settle_thr_m')
            lbl_settle = (f't_settle (<{thr_m:.3f}m)' if thr_m is not None
                          else 't_settle')
            if seg_ms:
                annotate_segment_steps(axs[i], seg_ms, settle_label=lbl_settle)

        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel(value)
        axs[i].set_title(f'{title} - {value}')
        axs[i].legend(fontsize=7)
        axs[i].grid(True, alpha=0.3)


def plot_speed_magnitude(data: dict, ax,
                         metrics: Optional[dict] = None,
                         color: Optional[str] = None,
                         label_suffix: str = '') -> None:
    """Plot speed magnitude |v| vs time with optional max_speed line."""
    vx, _ = get_series(data, 'vx')
    vy, _ = get_series(data, 'vy')
    vz, _ = get_series(data, 'vz')
    if not vx or not vy or not vz:
        return

    speed = np.sqrt(_np(vx)**2 + _np(vy)**2 + _np(vz)**2)
    lbl = f'|v|{label_suffix}' if label_suffix else '|v|'
    kw: dict = {'linestyle': 'solid'}
    if color:
        kw['color'] = color
    ax.plot(data['time'], speed, label=lbl, **kw)

    if metrics is not None and not label_suffix:
        ms = metrics.get('_effective_max_speed')
        if ms is not None:
            ax.axhline(ms, color='red', linestyle='--', linewidth=1.0,
                       label=f'max_speed={ms:.2f}')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('m/s')
    ax.set_title('Speed Magnitude')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)


def plot_drone_3d(position: np.ndarray, orientation: np.ndarray, axs) -> None:
    """Render a 3D drone frame at the given pose."""
    arm_len, rotor_len = 0.1, 0.05
    x, y, z = position
    qw, qx, qy, qz = orientation
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
    ])
    pos = np.array([x, y, z])
    heading = pos + R @ np.array([rotor_len, 0, 0])
    q1 = pos + R @ np.array([arm_len, arm_len, 0])
    q2 = pos + R @ np.array([-arm_len, -arm_len, 0])
    q3 = pos + R @ np.array([arm_len, -arm_len, 0])
    q4 = pos + R @ np.array([-arm_len, arm_len, 0])
    rotors = [q + R @ np.array([0, 0, rotor_len]) for q in [q1, q2, q3, q4]]
    axs.plot3D([q1[0], q2[0]], [q1[1], q2[1]], [q1[2], q2[2]], 'k')
    axs.plot3D([q3[0], q4[0]], [q3[1], q4[1]], [q3[2], q4[2]], 'k')
    for q, r in zip([q1, q2, q3, q4], rotors):
        axs.plot3D([q[0], r[0]], [q[1], r[1]], [q[2], r[2]], 'r')
    axs.plot3D([x, heading[0]], [y, heading[1]], [z, heading[2]], '-', color='orange')


def plot_trajectory_3d(data: dict, axs,
                       color: Optional[str] = None,
                       label_suffix: str = '') -> None:
    """Plot 3D trajectory with drone pose visualisations."""
    x_vals, y_vals, z_vals = data['x'], data['y'], data['z']
    kw: dict = {}
    if color:
        kw['color'] = color
    lbl = f'trajectory{label_suffix}' if label_suffix else 'trajectory'
    axs.plot(x_vals, y_vals, z_vals, linestyle='solid', label=lbl, **kw)

    x_ref, _ = get_series(data, 'x_ref')
    y_ref, _ = get_series(data, 'y_ref')
    z_ref, _ = get_series(data, 'z_ref')
    if x_ref and y_ref and z_ref:
        lbl_ref = f'reference{label_suffix}' if label_suffix else 'reference'
        axs.plot(x_ref, y_ref, z_ref, linestyle='dashed', label=lbl_ref, **kw)

    if not label_suffix:
        interval = max(1, len(x_vals) // 20)
        for step in range(0, len(x_vals), interval):
            plot_drone_3d(
                np.array([x_vals[step], y_vals[step], z_vals[step]]),
                np.array([data['qw'][step], data['qx'][step],
                          data['qy'][step], data['qz'][step]]),
                axs)

    axs.set_xlabel('x (m)')
    axs.set_ylabel('y (m)')
    axs.set_zlabel('z (m)')
    axs.set_title('3D Trajectory')
    axs.legend(fontsize=7)
    all_vals = np.concatenate([x_vals, y_vals, z_vals])
    max_range = max(abs(float(np.min(all_vals))), abs(float(np.max(all_vals))), 0.5)
    axs.set_xlim(-max_range, max_range)
    axs.set_ylim(-max_range, max_range)
    axs.set_zlim(0.0, max_range)


# ---------------------------------------------------------------------------
# Smoothness figure (Fig 3)
# ---------------------------------------------------------------------------


def _plot_signal_clipped(ax, t, signal, linewidth=0.8, color=None,
                         xlabel='Time (s)', ylabel='', title='',
                         clip_pct: float = 99.5) -> None:
    """Plot a 1-D signal, clipping the y-axis at clip_pct to suppress spikes."""
    kw: dict = {'linewidth': linewidth}
    if color:
        kw['color'] = color
    ax.plot(t, signal, **kw)
    if len(signal) > 0:
        p_clip = np.percentile(signal, clip_pct)
        ax.set_ylim(bottom=0.0, top=p_clip * 1.15)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)


def plot_smoothness_figure(data: dict, metrics: dict, label: str = '') -> plt.Figure:
    """Figure 3: solve-time histogram + acceleration + jerk magnitude."""
    fig, axs = plt.subplots(1, 3, figsize=(16, 4))
    suf = f' [{label}]' if label else ''
    fig.suptitle(f'Compute Time & Actuation Smoothness{suf}')

    # -- Solve time histogram --
    if 'controller_solve_time_us' in data:
        solve = _np(data['controller_solve_time_us'])
        valid = solve[solve > 0.0]
        if len(valid) > 0:
            axs[0].hist(valid, bins=50, edgecolor='black', linewidth=0.4)
            mean_v = float(np.mean(valid))
            axs[0].axvline(mean_v, color='red', linestyle='--',
                           label=f'mean={mean_v:.2f} µs')
            axs[0].set_xlabel('Solve time (µs)')
            axs[0].set_ylabel('Count')
            axs[0].set_title('Controller Solve Time Distribution')
            axs[0].legend(fontsize=7)
            axs[0].grid(True, alpha=0.3)
        else:
            axs[0].text(0.5, 0.5, 'No data', ha='center', va='center',
                        transform=axs[0].transAxes)
    else:
        axs[0].text(0.5, 0.5, 'controller_solve_time_us\nnot in CSV',
                    ha='center', va='center', transform=axs[0].transAxes)

    # -- Acceleration magnitude --
    accel_t = metrics.get('_accel_t', np.array([]))
    accel_norm = metrics.get('_accel_norm', np.array([]))
    ae = metrics.get('accel_energy')
    title_a = 'Linear Acceleration'
    if ae is not None:
        title_a += f'\nEnergy = {ae:.3f} (m/s²)²·s'
    if len(accel_t) > 0:
        _plot_signal_clipped(axs[1], accel_t, accel_norm,
                             ylabel='||a|| (m/s²)', title=title_a)

    # -- Jerk magnitude --
    jerk_t = metrics.get('_jerk_t', np.array([]))
    jerk_norm = metrics.get('_jerk_norm', np.array([]))
    je = metrics.get('jerk_energy')
    title_j = 'Jerk'
    if je is not None:
        title_j += f'\nEnergy = {je:.3f} (m/s³)²·s'
    if len(jerk_t) > 0:
        _plot_signal_clipped(axs[2], jerk_t, jerk_norm, color='darkorange',
                             ylabel='||j|| (m/s³)', title=title_j)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Metrics bar chart (Fig 4, comparison mode)
# ---------------------------------------------------------------------------


def plot_comparison_bars(m1: dict, m2: dict,
                         label1: str = 'A', label2: str = 'B') -> plt.Figure:
    """Bar chart comparison of scalar metrics."""
    metric_defs = [
        ('Compute\ntime (µs)', 'compute_time_mean_us'),
        ('Settling\ntime (s)', 'settling_time_s'),
        ('Steady-state\nerr (m)', 'steady_state_error_m'),
        ('Overshoot\n(m)', 'overshoot_m'),
        ('Rise\ntime (s)', 'rise_time_mean_s'),
        ('Jerk\nenergy', 'jerk_energy'),
        ('Accel\nenergy', 'accel_energy'),
        ('Speed\nviolation (%)', 'speed_violation_pct'),
    ]
    available = [(name, key) for name, key in metric_defs
                 if m1.get(key) is not None or m2.get(key) is not None]
    n = len(available)
    fig, axs = plt.subplots(1, n, figsize=(max(12, 2 * n), 5))
    if n == 1:
        axs = [axs]
    fig.suptitle(f'Metrics Comparison: {label1} vs {label2}')
    colors = ['steelblue', 'darkorange']
    for ax, (name, key) in zip(axs, available):
        v1 = m1.get(key)
        v2 = m2.get(key)
        vals = [v1 if v1 is not None else 0.0, v2 if v2 is not None else 0.0]
        bars = ax.bar([label1, label2], vals, color=colors,
                      edgecolor='black', linewidth=0.5)
        for bar, val in zip(bars, [v1, v2]):
            if val is not None:
                ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height(),
                        f'{val:.3g}', ha='center', va='bottom', fontsize=8)
        ax.set_title(name, fontsize=9)
        ax.grid(True, axis='y', alpha=0.3)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure builders — single dataset
# ---------------------------------------------------------------------------


def make_figures(data: dict, metrics: dict,
                 label: str = '',
                 color: Optional[str] = None) -> list:
    """Create all figures for a single dataset."""
    suffix = f' [{label}]' if label else ''
    figs = []

    # Fig 0: 3D trajectory
    fig0 = plt.figure(figsize=(8, 6))
    ax0 = fig0.add_subplot(projection='3d')
    plot_trajectory_3d(data, ax0, color=color)
    fig0.suptitle(f'3D Trajectory{suffix}')
    figs.append(fig0)

    # Fig 1: State tracking
    fig1, axs1 = plt.subplots(3, 3, figsize=(14, 10))
    fig1.suptitle(f'State Tracking{suffix}')
    if PRINT_ERROR:
        print(f'Position errors{suffix}:')
    plot_values(data, ['x', 'y', 'z'], 'Position', axs1[0, :],
                metrics=metrics, color=color)
    if PRINT_ERROR:
        print(f'Orientation errors{suffix}:')
    plot_values(data, ['roll', 'pitch', 'yaw'], 'Orientation', axs1[1, :],
                color=color)
    if PRINT_ERROR:
        print(f'Velocity{suffix}:')
    plot_values(data, ['vx', 'vy', 'vz'], 'Velocity', axs1[2, :], color=color)
    fig1.tight_layout()
    figs.append(fig1)

    # Fig 2: Control & angular velocity
    fig2, axs2 = plt.subplots(2, 3, figsize=(14, 7))
    fig2.suptitle(f'Control & Angular Velocity{suffix}')
    if PRINT_ERROR:
        print(f'Angular velocity{suffix}:')
    plot_values(data, ['wx', 'wy', 'wz'], 'Angular Velocity', axs2[0, :],
                color=color)

    thrust_data, _ = get_series(data, 'thrust')
    if thrust_data:
        kw: dict = {}
        if color:
            kw['color'] = color
        axs2[1, 0].plot(data['time'], thrust_data, label='thrust', **kw)
        axs2[1, 0].set_xlabel('Time (s)')
        axs2[1, 0].set_ylabel('N')
        axs2[1, 0].set_title('Thrust')
        axs2[1, 0].legend(fontsize=7)
        axs2[1, 0].grid(True, alpha=0.3)

    plot_speed_magnitude(data, axs2[1, 1], metrics=metrics, color=color)

    motor_keys = ['motor_w0', 'motor_w1', 'motor_w2', 'motor_w3']
    for mk in motor_keys:
        series, lbl = get_series(data, mk)
        if series:
            kw2: dict = {}
            if color:
                kw2['color'] = color
            axs2[1, 2].plot(data['time'], series, label=lbl, **kw2)
    axs2[1, 2].set_xlabel('Time (s)')
    axs2[1, 2].set_ylabel('rad/s')
    axs2[1, 2].set_title('Motor Angular Velocities')
    axs2[1, 2].legend(fontsize=7)
    axs2[1, 2].grid(True, alpha=0.3)

    fig2.tight_layout()
    figs.append(fig2)

    # Fig 3: Smoothness
    figs.append(plot_smoothness_figure(data, metrics, label=label))

    return figs


# ---------------------------------------------------------------------------
# Figure builders — comparison mode (two datasets)
# ---------------------------------------------------------------------------


def make_comparison_figures(data1: dict, data2: dict,
                             m1: dict, m2: dict,
                             label1: str, label2: str) -> list:
    """Create overlaid figures for two datasets plus a bar comparison figure."""
    figs = []
    colors = ['steelblue', 'darkorange']

    # Fig 0: 3D trajectory overlay
    fig0 = plt.figure(figsize=(8, 6))
    ax0 = fig0.add_subplot(projection='3d')
    plot_trajectory_3d(data1, ax0, color=colors[0], label_suffix=f' [{label1}]')
    plot_trajectory_3d(data2, ax0, color=colors[1], label_suffix=f' [{label2}]')
    ax0.set_title(f'3D Trajectory: {label1} vs {label2}')
    ax0.legend(fontsize=7)
    all_vals = np.concatenate([
        data1['x'], data1['y'], data1['z'],
        data2['x'], data2['y'], data2['z'],
    ])
    max_range = max(abs(float(np.min(all_vals))), abs(float(np.max(all_vals))), 0.5)
    ax0.set_xlim(-max_range, max_range)
    ax0.set_ylim(-max_range, max_range)
    ax0.set_zlim(0.0, max_range)
    figs.append(fig0)

    # Fig 1: State tracking overlay
    fig1, axs1 = plt.subplots(3, 3, figsize=(14, 10))
    fig1.suptitle(f'State Tracking: {label1} vs {label2}')
    for state_vars, row, title in [
        (['x', 'y', 'z'], 0, 'Position'),
        (['roll', 'pitch', 'yaw'], 1, 'Orientation'),
        (['vx', 'vy', 'vz'], 2, 'Velocity'),
    ]:
        plot_values(data1, state_vars, title, axs1[row, :],
                    metrics=m1, color=colors[0],
                    label_suffix=f' [{label1}]')
        plot_values(data2, state_vars, title, axs1[row, :],
                    color=colors[1], label_suffix=f' [{label2}]')
    fig1.tight_layout()
    figs.append(fig1)

    # Fig 2: Control overlay
    fig2, axs2 = plt.subplots(2, 3, figsize=(14, 7))
    fig2.suptitle(f'Control & Angular Velocity: {label1} vs {label2}')
    plot_values(data1, ['wx', 'wy', 'wz'], 'Angular Velocity', axs2[0, :],
                color=colors[0], label_suffix=f' [{label1}]')
    plot_values(data2, ['wx', 'wy', 'wz'], 'Angular Velocity', axs2[0, :],
                color=colors[1], label_suffix=f' [{label2}]')

    for d, color, lbl in [(data1, colors[0], label1), (data2, colors[1], label2)]:
        thrust_data, _ = get_series(d, 'thrust')
        if thrust_data:
            axs2[1, 0].plot(d['time'], thrust_data, color=color,
                            label=f'thrust [{lbl}]')
    axs2[1, 0].set_xlabel('Time (s)')
    axs2[1, 0].set_ylabel('N')
    axs2[1, 0].set_title('Thrust')
    axs2[1, 0].legend(fontsize=7)
    axs2[1, 0].grid(True, alpha=0.3)

    plot_speed_magnitude(data1, axs2[1, 1], metrics=m1,
                         color=colors[0], label_suffix=f' [{label1}]')
    plot_speed_magnitude(data2, axs2[1, 1],
                         color=colors[1], label_suffix=f' [{label2}]')
    axs2[1, 1].legend(fontsize=7)

    motor_keys = ['motor_w0', 'motor_w1', 'motor_w2', 'motor_w3']
    for d, color, lbl in [(data1, colors[0], label1), (data2, colors[1], label2)]:
        for mk in motor_keys:
            series, series_lbl = get_series(d, mk)
            if series:
                axs2[1, 2].plot(d['time'], series, color=color,
                                label=f'{series_lbl} [{lbl}]')
    axs2[1, 2].set_xlabel('Time (s)')
    axs2[1, 2].set_ylabel('rad/s')
    axs2[1, 2].set_title('Motor Angular Velocities')
    axs2[1, 2].legend(fontsize=7)
    axs2[1, 2].grid(True, alpha=0.3)
    fig2.tight_layout()
    figs.append(fig2)

    # Fig 3: Smoothness side-by-side
    fig3, axs3 = plt.subplots(2, 3, figsize=(16, 8))
    fig3.suptitle(f'Actuation Smoothness: {label1} vs {label2}')
    for row, (d, m, lbl, color) in enumerate([
            (data1, m1, label1, colors[0]),
            (data2, m2, label2, colors[1])]):
        if 'controller_solve_time_us' in d:
            solve = _np(d['controller_solve_time_us'])
            valid = solve[solve > 0.0]
            if len(valid) > 0:
                axs3[row, 0].hist(valid, bins=40, color=color,
                                  edgecolor='black', linewidth=0.4)
                mv = float(np.mean(valid))
                axs3[row, 0].axvline(mv, color='red', linestyle='--',
                                     label=f'mean={mv:.2f} µs')
                axs3[row, 0].set_title(f'Solve Time [{lbl}]')
                axs3[row, 0].set_xlabel('µs')
                axs3[row, 0].legend(fontsize=7)
                axs3[row, 0].grid(True, alpha=0.3)

        ae = m.get('accel_energy')
        accel_t = m.get('_accel_t', np.array([]))
        accel_n = m.get('_accel_norm', np.array([]))
        t_a = f'Accel [{lbl}]' + (f'\nEnergy={ae:.3f}' if ae else '')
        if len(accel_t) > 0:
            _plot_signal_clipped(axs3[row, 1], accel_t, accel_n, color=color,
                                 ylabel='||a|| (m/s²)', title=t_a)

        je = m.get('jerk_energy')
        jerk_t = m.get('_jerk_t', np.array([]))
        jerk_n = m.get('_jerk_norm', np.array([]))
        t_j = f'Jerk [{lbl}]' + (f'\nEnergy={je:.3f}' if je else '')
        if len(jerk_t) > 0:
            _plot_signal_clipped(axs3[row, 2], jerk_t, jerk_n, color=color,
                                 ylabel='||j|| (m/s³)', title=t_j)

    fig3.tight_layout()
    figs.append(fig3)

    # Fig 4: Metric comparison bars
    figs.append(plot_comparison_bars(m1, m2, label1, label2))

    return figs


# ---------------------------------------------------------------------------
# Metrics CSV I/O (read pre-computed metrics from compute_metrics.py output)
# ---------------------------------------------------------------------------

_RAW_SIGNAL_KEYS = ('_t', '_err3d', '_accel_t', '_accel_norm',
                    '_jerk_t', '_jerk_norm', '_speed')


def _derive_segments_path(metrics_path: str) -> str:
    """Derive the segments CSV path from a metrics CSV path.

    Examples:
        mpc_log_metrics.csv  →  mpc_log_segments.csv
        my_metrics.csv       →  my_segments.csv
    """
    base = os.path.splitext(metrics_path)[0]
    if base.endswith('_metrics'):
        base = base[:-len('_metrics')]
    return base + '_segments.csv'


def _parse_nullable_float(val: str) -> Optional[float]:
    v = val.strip() if isinstance(val, str) else ''
    return float(v) if v else None


def read_metrics_csv(path: str) -> dict:
    """Read an aggregate metrics CSV (written by compute_metrics.py) into a
    metrics dict compatible with the plotting functions.

    Raw signal arrays (_t, _accel_t, etc.) are initialised to empty arrays;
    they are populated by calling compute_metrics() on the log CSV when
    available (see main()).
    """
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        row = next(reader, None)
    if row is None:
        return {}

    m: dict = {
        '_settle_thr_m': _parse_nullable_float(row.get('settling_threshold_m', '')),
        '_effective_max_speed': _parse_nullable_float(row.get('effective_max_speed', '')),
        'compute_time_mean_us': _parse_nullable_float(row.get('compute_time_mean_us', '')),
        'compute_time_std_us': _parse_nullable_float(row.get('compute_time_std_us', '')),
        'settling_time_s': _parse_nullable_float(row.get('settling_time_s', '')),
        'settling_time_max_s': _parse_nullable_float(row.get('settling_time_max_s', '')),
        'steady_state_error_m': _parse_nullable_float(row.get('steady_state_error_m', '')),
        'overshoot_m': _parse_nullable_float(row.get('overshoot_m', '')),
        'rise_time_mean_s': _parse_nullable_float(row.get('rise_time_mean_s', '')),
        'rise_time_min_s': _parse_nullable_float(row.get('rise_time_min_s', '')),
        'rise_time_max_s': _parse_nullable_float(row.get('rise_time_max_s', '')),
        'jerk_energy': _parse_nullable_float(row.get('jerk_energy', '')),
        'accel_energy': _parse_nullable_float(row.get('accel_energy', '')),
        'speed_violation_pct': _parse_nullable_float(row.get('speed_violation_pct', '')),
        'seg_metrics': [],
    }
    # Initialise raw signal slots — filled later if a log CSV is provided.
    for key in _RAW_SIGNAL_KEYS:
        m[key] = np.array([])
    return m


def read_segments_csv(path: str) -> list:
    """Read a segments CSV (written by compute_metrics.py) into a list of
    per-segment metric dicts compatible with the plotting functions.
    """
    segs = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            segs.append({
                't_start': _parse_nullable_float(row.get('t_start', '')),
                't_end': _parse_nullable_float(row.get('t_end', '')),
                't_settle': _parse_nullable_float(row.get('t_settle', '')),
                'rise_time': _parse_nullable_float(row.get('rise_time', '')),
                'settling_time': _parse_nullable_float(row.get('settling_time', '')),
                'steady_state_error': _parse_nullable_float(row.get('steady_state_error', '')),
                'overshoot': _parse_nullable_float(row.get('overshoot', '')),
                'd0': _parse_nullable_float(row.get('d0', '')) or 1e-6,
                'settle_thr': _parse_nullable_float(row.get('settle_thr', '')) or 0.1,
                'ref_pos': np.array([
                    float(row.get('ref_x', 0) or 0),
                    float(row.get('ref_y', 0) or 0),
                    float(row.get('ref_z', 0) or 0),
                ]),
            })
    return segs


def save_figures(figs: list, outdir: str, names: list) -> None:
    """Save figures as PNG files in outdir."""
    os.makedirs(outdir, exist_ok=True)
    for fig, name in zip(figs, names):
        path = os.path.join(outdir, name)
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'  Saved: {path}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Plot position controller results with performance metrics')
    # Log CSV inputs
    parser.add_argument('-f', '--file_name', type=str, default=None,
                        help='Primary log CSV (optional when -m is given)')
    parser.add_argument('-f2', '--file_name2', type=str, default=None,
                        help='Second log CSV for comparison mode')
    # Pre-computed metrics CSV inputs
    parser.add_argument('-m', '--metrics', type=str, default=None,
                        help='Pre-computed metrics CSV for primary dataset')
    parser.add_argument('-s', '--segments', type=str, default=None,
                        help='Pre-computed segments CSV for primary dataset '
                             '(derived from -m if omitted)')
    parser.add_argument('-m2', '--metrics2', type=str, default=None,
                        help='Pre-computed metrics CSV for second dataset')
    parser.add_argument('-s2', '--segments2', type=str, default=None,
                        help='Pre-computed segments CSV for second dataset '
                             '(derived from -m2 if omitted)')
    # Metric tuning
    parser.add_argument('--max_speed', type=float, default=None,
                        help='Velocity constraint upper bound (m/s) for violation metric')
    # Labels
    parser.add_argument('--label1', type=str, default='',
                        help='Label for the primary dataset')
    parser.add_argument('--label2', type=str, default='',
                        help='Label for the second dataset (comparison mode)')
    # Display / save
    parser.add_argument('--no-show', action='store_true',
                        help='Do not display figures (implies --save)')
    parser.add_argument('--save', action='store_true',
                        help='Save figures as PNG files')
    parser.add_argument('--outdir', type=str, default=None,
                        help='Output directory for PNG files '
                             '(default: plots/ next to the log/metrics file)')
    args = parser.parse_args()

    # Backward-compat default: if neither log nor metrics is given, use the
    # original default log path.
    if args.file_name is None and args.metrics is None:
        args.file_name = 'simulator_logs/mpc_log.csv'

    cfg = dataclasses.replace(
        METRICS_CFG,
        max_speed=args.max_speed if args.max_speed is not None else METRICS_CFG.max_speed,
    )

    # --- Helpers -----------------------------------------------------------

    def _label_from_path(path: Optional[str], suffix_strip: str = '') -> str:
        if not path:
            return ''
        base = os.path.splitext(os.path.basename(path))[0]
        if suffix_strip and base.endswith(suffix_strip):
            base = base[:-len(suffix_strip)]
        return base

    def _load_metrics(metrics_path: str, segments_path: Optional[str],
                      data: dict, cfg: MetricsConfig) -> dict:
        """Load pre-computed metrics from CSV and optionally enrich with raw
        signals computed from the log data dict."""
        m = read_metrics_csv(metrics_path)
        seg_file = segments_path or _derive_segments_path(metrics_path)
        if os.path.exists(seg_file):
            m['seg_metrics'] = read_segments_csv(seg_file)
        if data:
            # Recompute raw signals (acceleration, jerk, etc.) from the log CSV
            # so the smoothness figure remains available. Scalar metrics and
            # seg_metrics from the pre-computed CSV are preserved.
            raw = compute_metrics(data, cfg)
            for key in _RAW_SIGNAL_KEYS:
                m[key] = raw[key]
        return m

    def _outdir_default(primary_path: Optional[str]) -> str:
        if primary_path:
            return os.path.join(os.path.dirname(os.path.abspath(primary_path)), 'plots')
        return 'plots'

    # --- Load primary dataset ----------------------------------------------

    data1: dict = {}
    if args.file_name is not None:
        fp1 = os.path.abspath(args.file_name)
        print(f'Reading log: {fp1}')
        data1 = read_csv(fp1)
        if not data1 or not data1.get('time'):
            print('No data in primary log file')
            return

    if args.metrics is not None:
        m1 = _load_metrics(args.metrics, args.segments, data1, cfg)
        label1 = args.label1 or _label_from_path(args.metrics, '_metrics')
    elif data1:
        m1 = compute_metrics(data1, cfg)
        label1 = args.label1 or _label_from_path(args.file_name)
    else:
        print('Error: provide at least -f (log CSV) or -m (metrics CSV)')
        return

    if not label1:
        label1 = 'dataset1'
    print_metrics(m1, label=label1, cfg=cfg)

    # --- Comparison mode ---------------------------------------------------

    comparison = args.file_name2 is not None or args.metrics2 is not None

    if comparison:
        data2: dict = {}
        if args.file_name2 is not None:
            fp2 = os.path.abspath(args.file_name2)
            print(f'\nReading log: {fp2}')
            data2 = read_csv(fp2)
            if not data2 or not data2.get('time'):
                print('No data in secondary log file')
                return

        if args.metrics2 is not None:
            m2 = _load_metrics(args.metrics2, args.segments2, data2, cfg)
            label2 = args.label2 or _label_from_path(args.metrics2, '_metrics')
        elif data2:
            m2 = compute_metrics(data2, cfg)
            label2 = args.label2 or _label_from_path(args.file_name2)
        else:
            print('Error: provide -f2 (log CSV) or -m2 (metrics CSV) for comparison mode')
            return

        if not label2:
            label2 = 'dataset2'
        print_metrics(m2, label=label2, cfg=cfg)
        print_metrics_comparison(m1, m2, label1=label1, label2=label2, cfg=cfg)

        if data1 and data2:
            figs = make_comparison_figures(data1, data2, m1, m2, label1, label2)
            fig_names = [
                f'{label1}_vs_{label2}_trajectory.png',
                f'{label1}_vs_{label2}_states.png',
                f'{label1}_vs_{label2}_controls.png',
                f'{label1}_vs_{label2}_smoothness.png',
                f'{label1}_vs_{label2}_bars.png',
            ]
        else:
            # No log CSVs — only generate the metrics bar chart.
            figs = [plot_comparison_bars(m1, m2, label1, label2)]
            fig_names = [f'{label1}_vs_{label2}_bars.png']
    else:
        if data1:
            figs = make_figures(data1, m1, label=label1)
            fig_names = [
                f'{label1}_trajectory.png',
                f'{label1}_states.png',
                f'{label1}_controls.png',
                f'{label1}_smoothness.png',
            ]
        else:
            print('No log CSV provided — cannot generate time-series figures.')
            figs = []
            fig_names = []

    if not figs:
        print('Nothing to plot.')
        return

    # --- Save PNGs ---------------------------------------------------------

    do_save = args.save or args.no_show
    if do_save:
        ref_path = args.file_name or args.metrics or ''
        outdir = args.outdir or _outdir_default(ref_path)
        save_figures(figs, outdir, fig_names)

    # --- Display -----------------------------------------------------------

    if not args.no_show:
        for fig in figs:
            fig.show()
        plt.pause(0.001)
        if sys.stdin.isatty():
            print('\nPress [Enter] to close.')
            try:
                input()
            except KeyboardInterrupt:
                pass

    plt.close('all')
    print('Plotting finished')


if __name__ == '__main__':
    main()
