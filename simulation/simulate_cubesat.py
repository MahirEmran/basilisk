import os
import argparse
import subprocess
import sys
import time
import atexit
import csv
import numpy as np
from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody, orbitalMotion, RigidBodyKinematics as rbk
from Basilisk.simulation import (
    spacecraft,
    extForceTorque,
    simpleNav,
    eclipse,
    simpleSolarPanel,
    simpleBattery,
    simplePowerSink,
    sensorThermal,
    simSynch,
)
from Basilisk.fswAlgorithms import attTrackingError, mrpPD
from Basilisk.architecture import messaging

from guidance_math import approx_sun_hat_from_epoch, compute_compromise_x, solve_roll_for_lost_clearance
from active_guidance import ActiveGuidance
from visual_model import build_satellite_obj, apply_visual_model
from uptime_metrics import compute_camera_uptime_flags
from vizard_scene import (
    enable_vizard,
    add_vizard_scene_overlays,
    add_spacecraft_status_overlay,
    update_spacecraft_status_overlay,
    create_battery_storage_panel,
)

try:
    from hil.hil_orchestrator import create_hil_orchestrator
    HIL_AVAILABLE = True
except ImportError:
    HIL_AVAILABLE = False


DEFAULT_ADCS_MODE = "HYBRID"
DEFAULT_SIM_HOURS = 24.0
DEFAULT_BODY_X_M = 0.30
DEFAULT_BODY_YZ_M = 0.10
DEFAULT_LOST_FOV_DEG = 25.5
DEFAULT_FOUND_FOV_DEG = 75.6
DEFAULT_LOST_EXCLUSION_BUFFER_DEG = 10.0
DEFAULT_FOUND_EXCLUSION_BUFFER_DEG = 10.0
DEFAULT_STATUS_PERIOD_SEC = 60.0
DEFAULT_BIN_PATH = "./output.bin"
DEFAULT_GUIDANCE_BACKEND = "EXTERNAL_CPP"
DEFAULT_GNSS_FIX_PERIOD_SEC = 600.0
DEFAULT_GNSS_FIX_DURATION_SEC = 60.0
DEFAULT_GNSS_ZENITH_HALF_ANGLE_DEG = 45.0
DEFAULT_GNSS_DEAD_RECKONING_SEC = 900.0
DEFAULT_DOWNLINK_WINDOW_SEC = 420.0
DEFAULT_RT_FACTOR = 1.0  # [-]
DEFAULT_GROUND_STATIONS = [
    "SEATTLE:47.6062:-122.3321",
    "DARMSTADT:49.8728:8.6512",
    "CANBERRA:-35.2809:149.1300",
    "HONOLULU:21.3069:-157.8583",
    "SANTIAGO:-33.4489:-70.6693",
]


PROGRAM_START_TIME_SEC = time.perf_counter()


def _print_total_runtime():
    """Print total wall-clock runtime from process start to exit."""
    elapsed_sec = time.perf_counter() - PROGRAM_START_TIME_SEC
    print(f"[TIMER] Total runtime: {elapsed_sec:,.2f} s ({elapsed_sec / 60.0:,.2f} min)")


atexit.register(_print_total_runtime)


def _fit_exponential_convergence(time_hours, uptime_pct):
    """Fit y(t) = a + b*exp(-c*t) with a simple grid-search on c and least squares on a,b."""
    t = np.asarray(time_hours, dtype=float)
    y = np.asarray(uptime_pct, dtype=float)
    valid = np.isfinite(t) & np.isfinite(y)
    t = t[valid]
    y = y[valid]

    if t.size < 3:
        return None

    t = np.maximum(t, 0.0)
    max_t = float(np.max(t))
    if max_t <= 0.0:
        return None

    c_candidates = np.logspace(-4.0, 2.0, 220)
    best = None
    best_sse = np.inf

    for c_val in c_candidates:
        x = np.exp(-c_val * t)
        design = np.column_stack([np.ones_like(x), x])
        coeffs, _, _, _ = np.linalg.lstsq(design, y, rcond=None)
        a_val, b_val = coeffs
        y_hat = a_val + b_val * x
        sse = float(np.sum((y - y_hat) ** 2))
        if sse < best_sse:
            best_sse = sse
            best = (float(a_val), float(b_val), float(c_val))

    if best is None:
        return None

    y_mean = float(np.mean(y))
    sst = float(np.sum((y - y_mean) ** 2))
    r2 = 1.0 - (best_sse / sst) if sst > 1e-12 else 1.0
    a_val, b_val, c_val = best
    return {
        "a": a_val,
        "b": b_val,
        "c": c_val,
        "r2": float(r2),
    }


def _generate_uptime_plots(save_path, history):
    """Write uptime convergence plots for overall, CHARGING, EXPERIMENT, GNSS_FIX, and DOWNLINK."""
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"[PLOT] Skipping uptime plots (matplotlib unavailable): {exc}")
        return []

    bin_path = os.path.abspath(save_path)
    bin_dir = os.path.dirname(bin_path)
    bin_stem = os.path.splitext(os.path.basename(bin_path))[0]
    plot_dir = os.path.join(bin_dir, f"{bin_stem}_plots")
    os.makedirs(plot_dir, exist_ok=True)

    t_hours = np.asarray(history["time_hours"], dtype=float)
    if t_hours.size == 0:
        print("[PLOT] No uptime samples collected; skipping plot generation.")
        return []

    max_t = float(np.max(t_hours))
    lin_thresh = max(0.05, min(1.0, 0.05 * max_t))

    series_meta = [
        ("lost_overall_pct", "LOST Overall Uptime", "lost_overall_uptime.png"),
        ("found_overall_pct", "FOUND Overall Uptime", "found_overall_uptime.png"),
        ("lost_charging_pct", "LOST CHARGING Uptime", "lost_charging_uptime.png"),
        ("found_charging_pct", "FOUND CHARGING Uptime", "found_charging_uptime.png"),
        ("lost_experiment_pct", "LOST EXPERIMENT Uptime", "lost_experiment_uptime.png"),
        ("found_experiment_pct", "FOUND EXPERIMENT Uptime", "found_experiment_uptime.png"),
        ("lost_gnss_pct", "LOST GNSS_FIX Uptime", "lost_gnss_uptime.png"),
        ("found_gnss_pct", "FOUND GNSS_FIX Uptime", "found_gnss_uptime.png"),
        ("lost_downlink_pct", "LOST DOWNLINK Uptime", "lost_downlink_uptime.png"),
        ("found_downlink_pct", "FOUND DOWNLINK Uptime", "found_downlink_uptime.png"),
    ]

    written_paths = []

    for key, title, filename in series_meta:
        y_vals = np.asarray(history[key], dtype=float)
        valid = np.isfinite(y_vals)

        fig, ax = plt.subplots(figsize=(10.0, 5.2))
        ax.plot(t_hours[valid], y_vals[valid], color="tab:blue", linewidth=1.8, label="cumulative uptime")

        fit_result = _fit_exponential_convergence(t_hours[valid], y_vals[valid])
        if fit_result is not None:
            t_fit = np.linspace(max(0.0, float(np.min(t_hours[valid]))), max_t, 400)
            y_fit = fit_result["a"] + fit_result["b"] * np.exp(-fit_result["c"] * t_fit)
            y_fit = np.clip(y_fit, 0.0, 100.0)
            ax.plot(
                t_fit,
                y_fit,
                color="tab:orange",
                linestyle="--",
                linewidth=1.6,
                label=(
                    f"fit: y={fit_result['a']:.2f}+{fit_result['b']:.2f}*exp(-{fit_result['c']:.4g}*t), "
                    f"R^2={fit_result['r2']:.3f}"
                ),
            )
            ax.axhline(
                fit_result["a"],
                color="tab:green",
                linestyle=":",
                linewidth=1.3,
                label=f"asymptote ~ {fit_result['a']:.2f}%",
            )

        ax.set_title(title)
        ax.set_xlabel("Time [hours]")
        ax.set_ylabel("Availability [%]")
        if key.startswith("lost_"):
            ax.set_ylim(95.0, 100.0)
        else:
            ax.set_ylim(60.0, 100.0)
        ax.set_xscale("symlog", linthresh=lin_thresh)
        ax.set_xlim(left=0.0, right=max(max_t, 1.0e-6))
        ax.grid(True, which="both", alpha=0.3)

        if np.any(valid):
            ax.legend(loc="best")
        else:
            ax.text(0.5, 0.5, "State not visited in this run", transform=ax.transAxes, ha="center", va="center")

        fig.tight_layout()
        plot_path = os.path.join(plot_dir, filename)
        fig.savefig(plot_path, dpi=160)
        plt.close(fig)
        written_paths.append(plot_path)

    return written_paths


def _write_uptime_points_csv(save_path, history):
    """Write cumulative uptime trace points to a CSV next to the plot outputs."""
    bin_path = os.path.abspath(save_path)
    bin_dir = os.path.dirname(bin_path)
    bin_stem = os.path.splitext(os.path.basename(bin_path))[0]
    plot_dir = os.path.join(bin_dir, f"{bin_stem}_plots")
    os.makedirs(plot_dir, exist_ok=True)

    csv_path = os.path.join(plot_dir, "uptime_points.csv")
    headers = [
        "time_hours",
        "lost_overall_pct",
        "found_overall_pct",
        "lost_charging_pct",
        "found_charging_pct",
        "lost_experiment_pct",
        "found_experiment_pct",
        "lost_gnss_pct",
        "found_gnss_pct",
        "lost_downlink_pct",
        "found_downlink_pct",
    ]

    def _fmt(value):
        value_f = float(value)
        if not np.isfinite(value_f):
            return ""
        return f"{value_f:.8f}"

    row_count = len(history.get("time_hours", []))
    with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for idx in range(row_count):
            row = []
            for key in headers:
                values = history.get(key, [])
                if idx >= len(values):
                    row.append("")
                else:
                    row.append(_fmt(values[idx]))
            writer.writerow(row)

    return csv_path


def _write_uptime_summary_txt(save_path,
                              lost_uptime_pct, found_uptime_pct,
                              lost_uptime_charging_pct, found_uptime_charging_pct,
                              lost_uptime_experiment_pct, found_uptime_experiment_pct,
                              lost_experiment_fail_nanos_by_body, lost_charging_fail_nanos_by_body,
                              found_charging_fail_nanos_by_reason, found_experiment_fail_nanos_by_reason,
                              lost_gnss_fail_nanos_by_body, lost_downlink_fail_nanos_by_body,
                              found_gnss_fail_nanos_by_reason, found_downlink_fail_nanos_by_reason,
                              sampled_time_nanos, sampled_time_charging_nanos, sampled_time_experiment_nanos,
                              sampled_time_by_state_nanos, lost_uptime_by_state_nanos, found_uptime_by_state_nanos,
                              state_names):
    """Write final uptime summary statistics to a text file."""
    bin_path = os.path.abspath(save_path)
    bin_dir = os.path.dirname(bin_path)
    bin_stem = os.path.splitext(os.path.basename(bin_path))[0]
    plot_dir = os.path.join(bin_dir, f"{bin_stem}_plots")
    os.makedirs(plot_dir, exist_ok=True)

    txt_path = os.path.join(plot_dir, "uptime_summary.txt")

    with open(txt_path, "w", encoding="utf-8") as txt_file:
        txt_file.write("UPTIME SUMMARY REPORT\n")
        txt_file.write("=" * 60 + "\n\n")

        txt_file.write("OVERALL UPTIME:\n")
        txt_file.write(f"  LOST clear uptime (total):    {lost_uptime_pct:6.2f}%\n")
        txt_file.write(f"  FOUND valid uptime (total):   {found_uptime_pct:6.2f}%\n\n")

        txt_file.write("CHARGING STATE UPTIME:\n")
        if lost_uptime_charging_pct is None:
            txt_file.write("  LOST clear uptime: N/A (state not visited)\n")
        else:
            txt_file.write(f"  LOST clear uptime: {lost_uptime_charging_pct:6.2f}%\n")
        if found_uptime_charging_pct is None:
            txt_file.write("  FOUND valid uptime: N/A (state not visited)\n")
        else:
            txt_file.write(f"  FOUND valid uptime: {found_uptime_charging_pct:6.2f}%\n")
        txt_file.write("\n")

        txt_file.write("EXPERIMENT STATE UPTIME:\n")
        if lost_uptime_experiment_pct is None:
            txt_file.write("  LOST clear uptime: N/A (state not visited)\n")
        else:
            txt_file.write(f"  LOST clear uptime: {lost_uptime_experiment_pct:6.2f}%\n")
        if found_uptime_experiment_pct is None:
            txt_file.write("  FOUND valid uptime: N/A (state not visited)\n")
        else:
            txt_file.write(f"  FOUND valid uptime: {found_uptime_experiment_pct:6.2f}%\n")
        txt_file.write("\n")

        if sampled_time_experiment_nanos > 0:
            txt_file.write("LOST EXPERIMENT FAIL-TIME BREAKDOWN (inner red keep-out):\n")
            for body_name in ("earth", "sun", "moon"):
                fail_pct = 100.0 * lost_experiment_fail_nanos_by_body[body_name] / sampled_time_experiment_nanos
                txt_file.write(f"  {body_name.upper():>5}: {fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when multiple bodies violate at once.\n\n")

        if sampled_time_charging_nanos > 0:
            txt_file.write("LOST CHARGING FAIL-TIME BREAKDOWN (inner red keep-out):\n")
            for body_name in ("earth", "sun", "moon"):
                fail_pct = 100.0 * lost_charging_fail_nanos_by_body[body_name] / sampled_time_charging_nanos
                txt_file.write(f"  {body_name.upper():>5}: {fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when multiple bodies violate at once.\n\n")

        if sampled_time_charging_nanos > 0:
            txt_file.write("FOUND CHARGING FAIL-TIME BREAKDOWN:\n")
            charge_earth_fail_pct = (
                100.0 * found_charging_fail_nanos_by_reason["earth_not_visible"] / sampled_time_charging_nanos
            )
            charge_sun_fail_pct = (
                100.0 * found_charging_fail_nanos_by_reason["sun_keepout_violation"] / sampled_time_charging_nanos
            )
            txt_file.write(f"  EARTH_NOT_VISIBLE:    {charge_earth_fail_pct:6.2f}%\n")
            txt_file.write(f"  SUN_KEEPOUT_VIOLATION:{charge_sun_fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when both FOUND conditions fail at once.\n\n")

        if sampled_time_experiment_nanos > 0:
            txt_file.write("FOUND EXPERIMENT FAIL-TIME BREAKDOWN:\n")
            exp_earth_fail_pct = (
                100.0 * found_experiment_fail_nanos_by_reason["earth_not_visible"] / sampled_time_experiment_nanos
            )
            exp_sun_fail_pct = (
                100.0 * found_experiment_fail_nanos_by_reason["sun_keepout_violation"] / sampled_time_experiment_nanos
            )
            txt_file.write(f"  EARTH_NOT_VISIBLE:    {exp_earth_fail_pct:6.2f}%\n")
            txt_file.write(f"  SUN_KEEPOUT_VIOLATION:{exp_sun_fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when both FOUND conditions fail at once.\n\n")

        gnss_time_nanos = sampled_time_by_state_nanos.get("GNSS_FIX", 0)  # [ns]
        if gnss_time_nanos > 0:
            txt_file.write("LOST GNSS_FIX FAIL-TIME BREAKDOWN (inner red keep-out):\n")
            for body_name in ("earth", "sun", "moon"):
                fail_pct = 100.0 * lost_gnss_fail_nanos_by_body[body_name] / gnss_time_nanos  # [%]
                txt_file.write(f"  {body_name.upper():>5}: {fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when multiple bodies violate at once.\n\n")

            txt_file.write("FOUND GNSS_FIX FAIL-TIME BREAKDOWN:\n")
            gnss_earth_fail_pct = (
                100.0 * found_gnss_fail_nanos_by_reason["earth_not_visible"] / gnss_time_nanos
            )  # [%]
            gnss_sun_fail_pct = (
                100.0 * found_gnss_fail_nanos_by_reason["sun_keepout_violation"] / gnss_time_nanos
            )  # [%]
            txt_file.write(f"  EARTH_NOT_VISIBLE:    {gnss_earth_fail_pct:6.2f}%\n")
            txt_file.write(f"  SUN_KEEPOUT_VIOLATION:{gnss_sun_fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when both FOUND conditions fail at once.\n\n")

        downlink_time_nanos = sampled_time_by_state_nanos.get("DOWNLINK", 0)  # [ns]
        if downlink_time_nanos > 0:
            txt_file.write("LOST DOWNLINK FAIL-TIME BREAKDOWN (inner red keep-out):\n")
            for body_name in ("earth", "sun", "moon"):
                fail_pct = 100.0 * lost_downlink_fail_nanos_by_body[body_name] / downlink_time_nanos  # [%]
                txt_file.write(f"  {body_name.upper():>5}: {fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when multiple bodies violate at once.\n\n")

            txt_file.write("FOUND DOWNLINK FAIL-TIME BREAKDOWN:\n")
            downlink_earth_fail_pct = (
                100.0 * found_downlink_fail_nanos_by_reason["earth_not_visible"] / downlink_time_nanos
            )  # [%]
            downlink_sun_fail_pct = (
                100.0 * found_downlink_fail_nanos_by_reason["sun_keepout_violation"] / downlink_time_nanos
            )  # [%]
            txt_file.write(f"  EARTH_NOT_VISIBLE:    {downlink_earth_fail_pct:6.2f}%\n")
            txt_file.write(f"  SUN_KEEPOUT_VIOLATION:{downlink_sun_fail_pct:6.2f}%\n")
            txt_file.write("  Note: percentages can overlap when both FOUND conditions fail at once.\n\n")

        txt_file.write("STATE-BY-STATE AVAILABILITY SUMMARY:\n")
        for state_name in state_names:
            state_time_nanos = sampled_time_by_state_nanos[state_name]
            if state_time_nanos <= 0:
                txt_file.write(f"  {state_name:>9}: not visited\n")
                continue

            state_time_pct = 100.0 * state_time_nanos / sampled_time_nanos
            state_lost_pct = 100.0 * lost_uptime_by_state_nanos[state_name] / state_time_nanos
            state_found_pct = 100.0 * found_uptime_by_state_nanos[state_name] / state_time_nanos
            txt_file.write(
                f"  {state_name:>9}: time={state_time_pct:6.2f}% "
                f"LOST={state_lost_pct:6.2f}% FOUND={state_found_pct:6.2f}%\n"
            )

    return txt_path


class LiveTelemetryPlot:
    """Small live plot window for power, battery charge, and temperature history."""

    def __init__(self, enabled, compression_power=0.5):
        """Initialize the live plot window if a GUI matplotlib backend is available."""
        self.enabled = bool(enabled)
        self.compression_power = float(compression_power)
        self.times_hr = []
        self.net_power_w = []
        self.soc_pct = []
        self.temp_c = []
        self.state_text = "init"
        self._plt = None
        self._fig = None
        self._axes = None
        self._power_line = None
        self._soc_line = None
        self._temp_line = None

        if not self.enabled:
            return

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            print(f"[PLOT] Live telemetry window disabled (matplotlib unavailable): {exc}")
            return

        try:
            plt.ion()
            self._plt = plt
            self._fig, self._axes = plt.subplots(3, 1, figsize=(7.6, 7.2), sharex=True)
            self._fig.suptitle("Live EPS / Thermal Telemetry")

            self._power_line, = self._axes[0].plot([], [], color="tab:red", linewidth=1.6)
            self._soc_line, = self._axes[1].plot([], [], color="tab:green", linewidth=1.6)
            self._temp_line, = self._axes[2].plot([], [], color="tab:blue", linewidth=1.6)

            self._axes[0].set_ylabel("Net Power [W]")
            self._axes[1].set_ylabel("SOC [%]")
            self._axes[2].set_ylabel("Temp [C]")
            self._axes[2].set_xlabel("Time [h]")
            self._axes[0].grid(True, alpha=0.25)
            self._axes[1].grid(True, alpha=0.25)
            self._axes[2].grid(True, alpha=0.25)
            self._axes[0].set_title("Mission state: init")
            self._fig.tight_layout()
            self._fig.show()
            self.enabled = True
        except Exception as exc:
            print(f"[PLOT] Live telemetry window disabled (GUI backend unavailable): {exc}")

    def update(self, time_sec, net_power_w, soc_pct, temp_c, mission_state):
        """Append one telemetry sample and refresh the live plot window."""
        if not self.enabled:
            return

        time_hr = float(time_sec) / 3600.0
        self.times_hr.append(time_hr)
        self.net_power_w.append(float(net_power_w))
        self.soc_pct.append(float(soc_pct))
        self.temp_c.append(float(temp_c))
        self.state_text = "UNKNOWN" if mission_state is None else str(mission_state)

        x_plot = np.power(np.maximum(np.asarray(self.times_hr, dtype=float), 0.0), self.compression_power)
        self._power_line.set_data(x_plot, self.net_power_w)
        self._soc_line.set_data(x_plot, self.soc_pct)
        self._temp_line.set_data(x_plot, self.temp_c)

        for axis in self._axes:
            axis.relim()
            axis.autoscale_view()
            axis.grid(True, alpha=0.25)

        self._axes[0].set_title(f"Mission state: {self.state_text.replace('_', ' ')}")
        self._axes[2].set_xlabel("Time [h] (sqrt-compressed)")
        max_time_hr = max(self.times_hr) if self.times_hr else 0.0
        max_plot_hr = np.power(max(max_time_hr, 1.0e-6), self.compression_power)
        for axis in self._axes:
            axis.set_xlim(0.0, max_plot_hr)
        tick_hours = np.linspace(0.0, max(max_time_hr, 1.0e-6), num=6)
        tick_positions = np.power(tick_hours, self.compression_power)
        self._axes[2].set_xticks(tick_positions)
        self._axes[2].set_xticklabels([f"{tick:.1f}" for tick in tick_hours])
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        self._plt.pause(0.001)

    def save_eps_plots(self, save_path):
        """Save EPS telemetry plots to file with log scale time axis."""
        if not self.times_hr:
            print("[PLOT] No EPS telemetry data to save")
            return []

        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            print(f"[PLOT] Skipping EPS plots (matplotlib unavailable): {exc}")
            return []

        bin_path = os.path.abspath(save_path)
        bin_dir = os.path.dirname(bin_path)
        bin_stem = os.path.splitext(os.path.basename(bin_path))[0]
        plot_dir = os.path.join(bin_dir, f"{bin_stem}_plots")
        os.makedirs(plot_dir, exist_ok=True)

        t_hours = np.asarray(self.times_hr, dtype=float)
        net_power = np.asarray(self.net_power_w, dtype=float)
        soc = np.asarray(self.soc_pct, dtype=float)
        temp = np.asarray(self.temp_c, dtype=float)

        saved_paths = []

        fig, axes = plt.subplots(3, 1, figsize=(10.0, 8.0), sharex=True)
        fig.suptitle("EPS / Thermal Telemetry (Log Scale Time)")

        axes[0].semilogx(t_hours, net_power, color="tab:red", linewidth=1.6)
        axes[0].set_ylabel("Net Power [W]")
        axes[0].grid(True, alpha=0.25)
        axes[0].set_title("Power")

        axes[1].semilogx(t_hours, soc, color="tab:green", linewidth=1.6)
        axes[1].set_ylabel("SOC [%]")
        axes[1].grid(True, alpha=0.25)
        axes[1].set_title("Battery State of Charge")

        axes[2].semilogx(t_hours, temp, color="tab:blue", linewidth=1.6)
        axes[2].set_ylabel("Temp [C]")
        axes[2].set_xlabel("Time [h] (log scale)")
        axes[2].grid(True, alpha=0.25)
        axes[2].set_title("Temperature")

        fig.tight_layout()

        eps_plot_path = os.path.join(plot_dir, f"{bin_stem}_eps_telemetry.png")
        fig.savefig(eps_plot_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        saved_paths.append(eps_plot_path)

        return saved_paths


def parse_cli_args():
    parser = argparse.ArgumentParser(description="Run CubeSat camera simulation with selectable ADCS mode.")
    parser.add_argument(
        "--mode",
        choices=["ROLL_ONLY", "EXPERIMENT", "COMPROMISE", "HYBRID", "BOTH"],
        default=DEFAULT_ADCS_MODE,
        type=str.upper,
        help="ADCS mode to run (default: HYBRID). COMPROMISE is accepted as a legacy alias for EXPERIMENT.",
    )
    parser.add_argument(
        "--hours",
        type=float,
        default=DEFAULT_SIM_HOURS,
        help="Simulation duration in hours (default: 1).",
    )
    parser.add_argument(
        "--body-x",
        type=float,
        default=DEFAULT_BODY_X_M,
        help="CubeSat long body dimension in meters (legacy name); applied along +Z so +/-Z are square faces (default: 0.30).",
    )
    parser.add_argument(
        "--body-yz",
        type=float,
        default=DEFAULT_BODY_YZ_M,
        help="CubeSat square face side length for X/Y in meters (default: 0.10).",
    )
    parser.add_argument(
        "--lost-fov",
        type=float,
        default=DEFAULT_LOST_FOV_DEG,
        help="LOST camera full FOV in degrees (default: 15).",
    )
    parser.add_argument(
        "--found-fov",
        type=float,
        default=DEFAULT_FOUND_FOV_DEG,
        help="FOUND camera full FOV in degrees (default: 60).",
    )
    parser.add_argument(
        "--lost-exclusion-buffer",
        type=float,
        default=DEFAULT_LOST_EXCLUSION_BUFFER_DEG,
        help="LOST exclusion cone buffer in degrees added to half-FOV (default: 10).",
    )
    parser.add_argument(
        "--found-exclusion-buffer",
        type=float,
        default=DEFAULT_FOUND_EXCLUSION_BUFFER_DEG,
        help="FOUND exclusion cone buffer in degrees added to half-FOV (default: 10).",
    )
    parser.add_argument(
        "--status-period",
        type=float,
        default=DEFAULT_STATUS_PERIOD_SEC,
        help="Status print period in seconds for HYBRID mode (default: 60).",
    )
    parser.add_argument(
        "--gnss-fix-period",
        type=float,
        default=DEFAULT_GNSS_FIX_PERIOD_SEC,
        help="GNSS fix cadence in seconds while in experiment context (default: 600).",
    )
    parser.add_argument(
        "--gnss-fix-duration",
        type=float,
        default=DEFAULT_GNSS_FIX_DURATION_SEC,
        help="Duration of each GNSS_FIX state in seconds (default: 60).",
    )
    parser.add_argument(
        "--gnss-zenith-half-angle",
        type=float,
        default=DEFAULT_GNSS_ZENITH_HALF_ANGLE_DEG,
        help="Half-angle cone from zenith where GNSS reception is considered healthy (default: 45 deg).",
    )
    parser.add_argument(
        "--gnss-dead-reckoning",
        type=float,
        default=DEFAULT_GNSS_DEAD_RECKONING_SEC,
        help="Maximum dead-reckoning horizon in seconds before forcing GNSS_FIX (default: 900).",
    )
    parser.add_argument(
        "--downlink-window",
        type=float,
        default=DEFAULT_DOWNLINK_WINDOW_SEC,
        help="Maximum DOWNLINK dwell in seconds per station visibility pass (default: 420).",
    )
    parser.add_argument(
        "--ground-station",
        action="append",
        default=None,
        help=(
            "Ground station specification as LABEL:LAT_DEG:LON_DEG. "
            "Repeat for multiple stations. If omitted, built-in defaults are used."
        ),
    )
    parser.add_argument(
        "--guidance-backend",
        choices=["PYTHON", "EXTERNAL_CPP"],
        default=DEFAULT_GUIDANCE_BACKEND,
        type=str.upper,
        help="Guidance implementation backend. EXTERNAL_CPP expects Basilisk.ExternalModules.activeGuidance.",
    )
    parser.add_argument(
        "--bin-path",
        type=str,
        default=DEFAULT_BIN_PATH,
        help="Output .bin path (including filename). Default: ./output.bin",
    )
    parser.add_argument(
        "--enable-plots",
        action="store_true",
        default=True,
        help="Enable end-of-run uptime plots (enabled by default).",
    )
    parser.add_argument(
        "--enable-live-eps-plots",
        action="store_true",
        help="Enable live EPS telemetry plotting during simulation (disabled by default for performance).",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Only generate uptime summary.txt, skip .bin file and plots.",
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Enable real-time simulation pacing (ClockSynch).",
    )
    parser.add_argument(
        "--rt-factor",
        type=float,
        default=DEFAULT_RT_FACTOR,
        help="Real-time acceleration factor (1.0 = real time).",
    )
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="Enable live streaming to Vizard while the sim runs.",
    )
    parser.add_argument(
        "--broadcast-stream",
        action="store_true",
        help="Broadcast live Vizard stream for remote subscribers.",
    )
    # HIL arguments
    parser.add_argument(
        "--hil-enable",
        action="store_true",
        help="Enable HIL mode for BeagleBone Black integration.",
    )
    parser.add_argument(
        "--hil-role",
        choices=["truth", "proxy"],
        default="truth",
        help="HIL role: truth (Basilisk) or proxy (BBB gateway).",
    )
    parser.add_argument(
        "--hil-transport",
        choices=["udp", "zmq", "uart"],
        default="zmq",
        help="HIL transport protocol.",
    )
    parser.add_argument(
        "--hil-bind-ip",
        default="0.0.0.0",
        help="HIL bind IP address for truth role.",
    )
    parser.add_argument(
        "--hil-bind-port",
        type=int,
        default=5555,
        help="HIL bind port for truth role.",
    )
    parser.add_argument(
        "--hil-peer-ip",
        default="127.0.0.1",
        help="HIL peer IP address (BBB for truth role, Basilisk for proxy role).",
    )
    parser.add_argument(
        "--hil-peer-port",
        type=int,
        default=5556,
        help="HIL peer port.",
    )
    parser.add_argument(
        "--hil-cycle-ms",
        type=int,
        default=10,
        help="HIL cycle time in milliseconds (default: 10ms = 100Hz).",
    )
    parser.add_argument(
        "--hil-timeout-ms",
        type=int,
        default=5000,
        help="HIL command timeout in milliseconds.",
    )
    parser.add_argument(
        "--hil-hold-last-max-ms",
        type=int,
        default=10000,
        help="HIL maximum hold-last duration in milliseconds.",
    )
    args = parser.parse_args()
    if args.hours <= 0.0:
        parser.error("--hours must be greater than 0")
    if args.body_x <= 0.0 or args.body_yz <= 0.0:
        parser.error("--body-x and --body-yz must be greater than 0")
    if args.lost_fov <= 0.0 or args.found_fov <= 0.0:
        parser.error("--lost-fov and --found-fov must be greater than 0")
    if args.lost_exclusion_buffer < 0.0:
        parser.error("--lost-exclusion-buffer must be >= 0")
    if args.found_exclusion_buffer < 0.0:
        parser.error("--found-exclusion-buffer must be >= 0")
    if args.status_period <= 0.0:
        parser.error("--status-period must be greater than 0")
    if args.gnss_fix_period <= 0.0:
        parser.error("--gnss-fix-period must be greater than 0")
    if args.gnss_fix_duration <= 0.0:
        parser.error("--gnss-fix-duration must be greater than 0")
    if args.gnss_zenith_half_angle <= 0.0 or args.gnss_zenith_half_angle > 90.0:
        parser.error("--gnss-zenith-half-angle must be in (0, 90]")
    if args.gnss_dead_reckoning <= 0.0:
        parser.error("--gnss-dead-reckoning must be greater than 0")
    if args.downlink_window <= 0.0:
        parser.error("--downlink-window must be greater than 0")
    if args.rt_factor <= 0.0:
        parser.error("--rt-factor must be greater than 0")
    if args.hil_transport == "zmq":
        if args.hil_bind_port == 5555 and args.hil_peer_port == 5556:
            args.hil_bind_port = 5566
            args.hil_peer_port = 5555
    if args.ground_station is None:
        args.ground_station = list(DEFAULT_GROUND_STATIONS)
    return args


CLI_ARGS = parse_cli_args()
LIVE_PLOTS_ENABLED = bool(CLI_ARGS.enable_live_eps_plots)
END_OF_RUN_PLOTS_ENABLED = bool(CLI_ARGS.enable_plots)
REALTIME_ENABLED = bool(CLI_ARGS.realtime) or bool(CLI_ARGS.hil_enable)
LIVE_STREAM_ENABLED = bool(CLI_ARGS.live_stream) or bool(CLI_ARGS.hil_enable)
BROADCAST_STREAM_ENABLED = bool(CLI_ARGS.broadcast_stream)


def _create_guidance_module(
    backend,
    mode,
    epoch_iso_utc,
    lost_excl_half_deg,
    status_period_sec,
    pos_found_b,
    pos_comms_b,
    gnss_fix_period_sec,
    gnss_fix_duration_sec,
    gnss_zenith_half_angle_deg,
    gnss_dead_reckoning_sec,
    downlink_window_sec,
    ground_stations,
):
    if backend == "EXTERNAL_CPP":
        try:
            from Basilisk.ExternalModules import activeGuidance
        except Exception as exc:
            raise RuntimeError(
                "EXTERNAL_CPP backend requested, but Basilisk.ExternalModules.activeGuidance "
                "is unavailable. Rebuild Basilisk from source with "
                "--pathToExternalModules pointing to simulation/External."
            ) from exc

        module = activeGuidance.ActiveGuidance()
        module.setModeString(mode)
        module.setLostExclHalfDeg(float(lost_excl_half_deg))
        module.setStatusPeriodSec(float(status_period_sec))
        module.setGnssFixPeriodSec(float(gnss_fix_period_sec))
        module.setGnssFixDurationSec(float(gnss_fix_duration_sec))
        module.setGnssZenithHalfAngleDeg(float(gnss_zenith_half_angle_deg))
        module.setGnssDeadReckoningSec(float(gnss_dead_reckoning_sec))
        module.setDownlinkWindowSec(float(downlink_window_sec))
        module.setGroundStationsCsv(";".join(ground_stations))
        module.setPosFound_B(float(pos_found_b[0]), float(pos_found_b[1]), float(pos_found_b[2]))
        module.setPosComms_B(float(pos_comms_b[0]), float(pos_comms_b[1]), float(pos_comms_b[2]))
        sun_hat = approx_sun_hat_from_epoch(epoch_iso_utc)
        module.setDefaultSunHat_N(float(sun_hat[0]), float(sun_hat[1]), float(sun_hat[2]))
        return module

    return ActiveGuidance(
        mode=mode,
        epoch_iso_utc=epoch_iso_utc,
        lost_excl_half_deg=lost_excl_half_deg,
        status_period_sec=status_period_sec,
        pos_found_b=pos_found_b,
        pos_comms_b=pos_comms_b,
        gnss_fix_period_sec=gnss_fix_period_sec,
        gnss_fix_duration_sec=gnss_fix_duration_sec,
        gnss_zenith_half_angle_deg=gnss_zenith_half_angle_deg,
        gnss_dead_reckoning_sec=gnss_dead_reckoning_sec,
        downlink_window_sec=downlink_window_sec,
        ground_stations=ground_stations,
    )


def _get_guidance_state(guidance):
    if hasattr(guidance, "state"):
        state = guidance.state
        if isinstance(state, bytes):
            state = state.decode("utf-8", errors="ignore")
        if isinstance(state, str):
            return state

    if hasattr(guidance, "getState"):
        state = guidance.getState()
        if isinstance(state, bytes):
            state = state.decode("utf-8", errors="ignore")
        if isinstance(state, str):
            return state

    return None


def _get_active_ground_station(guidance):
    if hasattr(guidance, "active_ground_station"):
        station = guidance.active_ground_station
        if isinstance(station, bytes):
            station = station.decode("utf-8", errors="ignore")
        if isinstance(station, str):
            return station

    if hasattr(guidance, "getActiveGroundStation"):
        station = guidance.getActiveGroundStation()
        if isinstance(station, bytes):
            station = station.decode("utf-8", errors="ignore")
        if isinstance(station, str):
            return station

    return "NONE"


def run_both_modes(args):
    script_path = os.path.abspath(__file__)
    base_output_path = os.path.abspath(args.bin_path)
    base_dir = os.path.dirname(base_output_path)
    base_name = os.path.basename(base_output_path)
    base_stem, base_ext = os.path.splitext(base_name)
    if not base_ext:
        base_ext = ".bin"

    roll_only_output = os.path.join(base_dir, f"{base_stem}_ROLL_ONLY{base_ext}")
    experiment_output = os.path.join(base_dir, f"{base_stem}_EXPERIMENT{base_ext}")

    shared = [
        "--hours", str(args.hours),
        "--body-x", str(args.body_x),
        "--body-yz", str(args.body_yz),
        "--lost-fov", str(args.lost_fov),
        "--found-fov", str(args.found_fov),
        "--lost-exclusion-buffer", str(args.lost_exclusion_buffer),
        "--found-exclusion-buffer", str(args.found_exclusion_buffer),
        "--status-period", str(args.status_period),
        "--gnss-fix-period", str(args.gnss_fix_period),
        "--gnss-fix-duration", str(args.gnss_fix_duration),
        "--gnss-zenith-half-angle", str(args.gnss_zenith_half_angle),
        "--gnss-dead-reckoning", str(args.gnss_dead_reckoning),
        "--downlink-window", str(args.downlink_window),
        "--guidance-backend", str(args.guidance_backend),
    ]
    for station_spec in args.ground_station:
        shared.extend(["--ground-station", station_spec])
    commands = [
        [sys.executable, script_path, "--mode", "ROLL_ONLY", "--bin-path", roll_only_output, *shared],
        [sys.executable, script_path, "--mode", "EXPERIMENT", "--bin-path", experiment_output, *shared],
    ]

    print("Mode: BOTH (launching ROLL_ONLY and EXPERIMENT subprocesses)")
    processes = [subprocess.Popen(cmd) for cmd in commands]
    return_codes = [proc.wait() for proc in processes]

    if any(code != 0 for code in return_codes):
        raise SystemExit(max(return_codes))
    raise SystemExit(0)


def _mrp_to_quaternion(attitude_mrp):
    sigma = np.array(attitude_mrp, dtype=float)
    sigma_squared = float(np.sum(sigma ** 2))
    denominator = 1.0 + sigma_squared
    q0 = (1.0 - sigma_squared) / denominator
    q123 = 2.0 * sigma / denominator
    return np.array([q0, q123[0], q123[1], q123[2]])


def _build_hil_config(cli_args):
    return {
        "hil_enable": True,
        "hil_role": cli_args.hil_role,
        "hil_transport": cli_args.hil_transport,
        "hil_bind_ip": cli_args.hil_bind_ip,
        "hil_bind_port": cli_args.hil_bind_port,
        "hil_peer_ip": cli_args.hil_peer_ip,
        "hil_peer_port": cli_args.hil_peer_port,
        "hil_cycle_ms": cli_args.hil_cycle_ms,
        "hil_timeout_ms": cli_args.hil_timeout_ms,
        "hil_hold_last_max_ms": cli_args.hil_hold_last_max_ms,
    }


def _initialize_hil_orchestrator(cli_args):
    if not cli_args.hil_enable:
        return None
    if not HIL_AVAILABLE:
        print("[HIL] ERROR: HIL modules not available. Install simulation/hil package.")
        sys.exit(1)

    print("[HIL] Initializing HIL orchestrator...")
    hil_orchestrator = create_hil_orchestrator(_build_hil_config(cli_args))
    if not hil_orchestrator.initialize():
        print("[HIL] ERROR: Failed to initialize HIL orchestrator")
        sys.exit(1)
    if not hil_orchestrator.start():
        print("[HIL] ERROR: Failed to start HIL orchestrator")
        sys.exit(1)
    print("[HIL] HIL orchestrator started successfully")
    return hil_orchestrator


def _update_hil_orchestrator(hil_orchestrator, sc_state_out_msg):
    if hil_orchestrator is None:
        return
    try:
        sc_state_now = sc_state_out_msg.read()
        position = sc_state_now.r_BN_N
        velocity = sc_state_now.v_BN_N
        attitude_quat = _mrp_to_quaternion(sc_state_now.sigma_BN)
        rate = sc_state_now.omega_BN_B
        hil_orchestrator.send_truth_nav(position, velocity, attitude_quat, rate)

        command = hil_orchestrator.get_command()
        if command:
            pass
    except Exception as e:
        print(f"[HIL] Warning: HIL integration error: {e}")


def _shutdown_hil_orchestrator(hil_orchestrator):
    if hil_orchestrator is None:
        return
    print("[HIL] Stopping HIL orchestrator...")
    hil_orchestrator.stop()
    stats = hil_orchestrator.get_statistics()
    print("[HIL] HIL Statistics:")
    print(f"[HIL]   Sequence TX: {stats.get('seq_tx', 0)}")
    print(f"[HIL]   Sequence RX: {stats.get('seq_rx', 0)}")
    print(f"[HIL]   Dropped frames: {stats.get('dropped_frames', 0)}")
    print(f"[HIL]   Stale frames: {stats.get('stale_frames', 0)}")
    print(f"[HIL]   Avg loop jitter: {stats.get('avg_loop_jitter_ms', 0):.2f} ms")
    print(f"[HIL]   Avg loop latency: {stats.get('avg_loop_latency_ms', 0):.2f} ms")
    print(f"[HIL]   Avg command age: {stats.get('avg_command_age_ms', 0):.2f} ms")
    print(f"[HIL]   Watchdog trips: {stats.get('watchdog_trips', 0)}")

ADCS_MODE = CLI_ARGS.mode

if ADCS_MODE == "COMPROMISE":
    ADCS_MODE = "EXPERIMENT"

if ADCS_MODE == "BOTH":
    run_both_modes(CLI_ARGS)

SIM_EPOCH_UTC = "2026-01-01T12:00:00.000Z"

# Body-frame layout: +X FOUND face, +Z LOST face, -X GNSS antenna, +X comms antenna.
BODY_LONG_M = CLI_ARGS.body_x
BODY_SIDE_M = CLI_ARGS.body_yz

BODY_SIZE_XY_M = BODY_SIDE_M
BODY_SIZE_Z_M = BODY_LONG_M

BODY_SIZE_X_M = BODY_SIZE_XY_M
BODY_SIZE_Y_M = BODY_SIZE_XY_M
FOUND_Z_OFFSET_FRAC = 0.35

VEC_LOST_B  = [0, 0,  1]   # LOST points out +Z (square face)
VEC_FOUND_B = [1, 0,  0]   # FOUND points out +X (long face)
VEC_ANT_B   = [-1, 0,  0]   # GNSS antenna points out -X (opposite long side)
VEC_COMMS_B = [1, 0, 0]  # Comms antenna points out +X (same face as FOUND)

POS_LOST_B  = [0.0, 0.0,  0.5 * BODY_SIZE_Z_M]
POS_FOUND_B = [0.5 * BODY_SIZE_X_M, 0.0, FOUND_Z_OFFSET_FRAC * BODY_SIZE_Z_M]
POS_ANT_B   = [-0.5 * BODY_SIZE_X_M, 0.0, 0.0]  # [m] -X long side
POS_COMMS_B = [0.5 * BODY_SIZE_X_M, 0.0, 0.0]  # [m] same +X face as FOUND, lowered toward bus mid-height

LOST_FOV_DEG  = CLI_ARGS.lost_fov
FOUND_FOV_DEG = CLI_ARGS.found_fov

LOST_HALF_DEG  = LOST_FOV_DEG  / 2.0
FOUND_HALF_DEG = FOUND_FOV_DEG / 2.0

LOST_EXCL_HALF_DEG  = LOST_HALF_DEG  + CLI_ARGS.lost_exclusion_buffer
FOUND_EXCL_HALF_DEG = FOUND_HALF_DEG + CLI_ARGS.found_exclusion_buffer

SIM_HOURS = CLI_ARGS.hours

sun_hat = approx_sun_hat_from_epoch(SIM_EPOCH_UTC)

mu_init = 3.986004415e14
oe_init = orbitalMotion.ClassicElements()
oe_init.a = (6371.0 + 400.0) * 1000.0  # 400 km circular
oe_init.e = 0.0001; oe_init.i = 51.6 * macros.D2R  # ISS-like inclination
oe_init.Omega = 0.0; oe_init.omega = 0.0; oe_init.f = 90.0 * macros.D2R
rN_init, _     = orbitalMotion.elem2rv(mu_init, oe_init)
earth_hat_init = -np.array(rN_init) / np.linalg.norm(rN_init)

if ADCS_MODE == "ROLL_ONLY":
    x_B_init = -sun_hat
    init_state = "CHARGING"
elif ADCS_MODE == "EXPERIMENT":
    rho_rad = np.arcsin(6371.0 / (6371.0 + 400.0))
    x_B_init = compute_compromise_x(earth_hat_init, sun_hat, rho_rad)
    init_state = "EXPERIMENT"
else:
    charge_enter_clear_deg = LOST_EXCL_HALF_DEG + 2.0
    earth_half_angle_deg = np.degrees(np.arcsin(6371.0 / (6371.0 + 400.0)))
    charge_enter_sun_vis_deg = earth_half_angle_deg + 2.0
    _, _, _, charge_score_init = solve_roll_for_lost_clearance(-sun_hat, earth_hat_init, sun_hat)
    sun_earth_ang_init = np.degrees(np.arccos(np.clip(np.dot(sun_hat, earth_hat_init), -1.0, 1.0)))
    if charge_score_init >= charge_enter_clear_deg and sun_earth_ang_init >= charge_enter_sun_vis_deg:
        x_B_init = -sun_hat
        init_state = "CHARGING"
    else:
        rho_rad = np.arcsin(6371.0 / (6371.0 + 400.0))
        x_B_init = compute_compromise_x(earth_hat_init, sun_hat, rho_rad)
        init_state = "EXPERIMENT"

_, y_B_init, z_B_init, _ = solve_roll_for_lost_clearance(x_B_init, earth_hat_init, sun_hat)
sigma_BN_init = rbk.C2MRP(np.array([x_B_init, y_B_init, z_B_init]))
print(f"Initial sigma_BN (near target): {sigma_BN_init.tolist()}")
print(f"Initial ADCS state: {init_state}")


scSim       = SimulationBaseClass.SimBaseClass()
simTaskName = "dynamicsTask"
dynProcess  = scSim.CreateNewProcess("dynamicsProcess")

dynProcess.addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(0.5)))

if REALTIME_ENABLED:
    clock_sync = simSynch.ClockSynch()
    clock_sync.accelFactor = float(CLI_ARGS.rt_factor)
    scSim.AddModelToTask(simTaskName, clock_sync)
    print(f"[REALTIME] ClockSynch enabled at {clock_sync.accelFactor:.2f}x")

scObject          = spacecraft.Spacecraft()
scObject.ModelTag = "cubesat"
scObject.hub.mHub = 12.0
scObject.hub.IHubPntBc_B = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]  # kg*m^2, roughly 3U-ish
scSim.AddModelToTask(simTaskName, scObject)

gravFactory = simIncludeGravBody.gravBodyFactory()
earth = gravFactory.createEarth(); earth.isCentralBody = True
sun   = gravFactory.createSun()
moon  = gravFactory.createMoon()
gravFactory.createSpiceInterface(time=SIM_EPOCH_UTC, epochInMsg=True)
gravFactory.spiceObject.zeroBase = "earth"  # all positions relative to Earth center
scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

mu   = 3.986004415e14
oe   = orbitalMotion.ClassicElements()
oe.a = (6371.0 + 400.0) * 1000.0
oe.e = 0.0001; oe.i = 51.6 * macros.D2R
oe.Omega = 0.0; oe.omega = 0.0; oe.f = 90.0 * macros.D2R
rN, vN = orbitalMotion.elem2rv(mu, oe)
scObject.hub.r_CN_NInit     = rN
scObject.hub.v_CN_NInit     = vN
scObject.hub.sigma_BNInit   = sigma_BN_init.tolist()  # near target, not identity
scObject.hub.omega_BN_BInit = [0.0, 0.0, 0.0]

sNav          = simpleNav.SimpleNav()
sNav.ModelTag = "SimpleNav"
scSim.AddModelToTask(simTaskName, sNav)
sNav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

guidance = _create_guidance_module(
    backend=CLI_ARGS.guidance_backend,
    mode=ADCS_MODE,
    epoch_iso_utc=SIM_EPOCH_UTC,
    lost_excl_half_deg=LOST_EXCL_HALF_DEG,
    status_period_sec=CLI_ARGS.status_period,
    pos_found_b=POS_FOUND_B,
    pos_comms_b=POS_COMMS_B,
    gnss_fix_period_sec=CLI_ARGS.gnss_fix_period,
    gnss_fix_duration_sec=CLI_ARGS.gnss_fix_duration,
    gnss_zenith_half_angle_deg=CLI_ARGS.gnss_zenith_half_angle,
    gnss_dead_reckoning_sec=CLI_ARGS.gnss_dead_reckoning,
    downlink_window_sec=CLI_ARGS.downlink_window,
    ground_stations=CLI_ARGS.ground_station,
)
guidance.ModelTag = "activeGuidance"
guidance.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

sun_index = 1
moon_index = 2
if hasattr(gravFactory, "spicePlanetNames"):
    planet_names = [name.lower() for name in gravFactory.spicePlanetNames]
    if "sun" in planet_names:
        sun_index = planet_names.index("sun")
    if "moon" in planet_names:
        moon_index = planet_names.index("moon")
guidance.sunStateInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun_index])
guidance.moonStateInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[moon_index])

scSim.AddModelToTask(simTaskName, guidance)

attErr          = attTrackingError.attTrackingError()
attErr.ModelTag = "attError"
scSim.AddModelToTask(simTaskName, attErr)
attErr.attNavInMsg.subscribeTo(sNav.attOutMsg)
attErr.attRefInMsg.subscribeTo(guidance.attRefOutMsg)

mrpControl          = mrpPD.mrpPD()
mrpControl.ModelTag = "mrpPD"
mrpControl.K        = 0.05
mrpControl.P        = 0.3
scSim.AddModelToTask(simTaskName, mrpControl)
mrpControl.guidInMsg.subscribeTo(attErr.attGuidOutMsg)

vehicleConfig = messaging.VehicleConfigMsgPayload()
vehicleConfig.ISCPntB_B = [0.1, 0.0, 0.0,
                            0.0, 0.1, 0.0,
                            0.0, 0.0, 0.1]
vehConfigMsg = messaging.VehicleConfigMsg().write(vehicleConfig)
mrpControl.vehConfigInMsg.subscribeTo(vehConfigMsg)

extFT          = extForceTorque.ExtForceTorque()
extFT.ModelTag = "extFT"
scSim.AddModelToTask(simTaskName, extFT)
scObject.addDynamicEffector(extFT)
extFT.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

payload_power_draw_w = 8.0  # [W]
battery_capacity_whr = 75.0  # [W*hr]
battery_initial_charge_whr = 50.0  # [W*hr]
solar_cell_count = 18.0  # [-]
solar_cell_area_m2 = 27.0e-4  # [m^2]
solar_panel_area_m2 = solar_cell_count * solar_cell_area_m2  # [m^2]
solar_panel_efficiency = 0.29  # [-]
solar_panel_normal_b = [-1.0, 0.0, 0.0]  # [-]
sensor_area_m2 = 0.02  # [m^2]
sensor_absorptivity = 0.80  # [-]
sensor_emissivity = 0.80  # [-]
sensor_mass_kg = 0.35  # [kg]
sensor_specific_heat_j_per_kg_k = 900.0  # [J/kg/K]
sensor_initial_temp_c = 8.0  # [C]
sensor_normal_b = [1.0, 0.0, 0.0]  # [-]

eclipseObject = eclipse.Eclipse()
eclipseObject.ModelTag = "statusEclipse"
eclipseObject.addSpacecraftToModel(scObject.scStateOutMsg)
eclipseObject.addPlanetToModel(gravFactory.spiceObject.planetStateOutMsgs[0])
eclipseObject.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun_index])
scSim.AddModelToTask(simTaskName, eclipseObject)

solarPanel = simpleSolarPanel.SimpleSolarPanel()
solarPanel.ModelTag = "statusSolarPanel"
solarPanel.stateInMsg.subscribeTo(scObject.scStateOutMsg)
solarPanel.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun_index])
solarPanel.sunEclipseInMsg.subscribeTo(eclipseObject.eclipseOutMsgs[0])
solarPanel.setPanelParameters(solar_panel_normal_b, solar_panel_area_m2, solar_panel_efficiency)
scSim.AddModelToTask(simTaskName, solarPanel)

payloadPowerSink = simplePowerSink.SimplePowerSink()
payloadPowerSink.ModelTag = "statusPayloadLoad"
payloadPowerSink.nodePowerOut = -payload_power_draw_w
scSim.AddModelToTask(simTaskName, payloadPowerSink)

powerMonitor = simpleBattery.SimpleBattery()
powerMonitor.ModelTag = "statusBattery"
powerMonitor.storageCapacity = battery_capacity_whr * 3600.0  # [W*s]
powerMonitor.storedCharge_Init = battery_initial_charge_whr * 3600.0  # [W*s]
powerMonitor.addPowerNodeToModel(solarPanel.nodePowerOutMsg)
powerMonitor.addPowerNodeToModel(payloadPowerSink.nodePowerOutMsg)
scSim.AddModelToTask(simTaskName, powerMonitor)

battery_storage_reader = messaging.PowerStorageStatusMsgReader()
battery_storage_reader.subscribeTo(powerMonitor.batPowerOutMsg)
battery_storage_panel = create_battery_storage_panel(battery_storage_reader)
viz_storage_list = [[battery_storage_panel]]

thermalSensor = sensorThermal.SensorThermal()
thermalSensor.ModelTag = "statusSensorThermal"
thermalSensor.T_0 = sensor_initial_temp_c
thermalSensor.nHat_B = sensor_normal_b
thermalSensor.sensorArea = sensor_area_m2
thermalSensor.sensorAbsorptivity = sensor_absorptivity
thermalSensor.sensorEmissivity = sensor_emissivity
thermalSensor.sensorMass = sensor_mass_kg
thermalSensor.sensorSpecificHeat = sensor_specific_heat_j_per_kg_k
thermalSensor.sensorPowerDraw = payload_power_draw_w
thermalSensor.sunInMsg.subscribeTo(gravFactory.spiceObject.planetStateOutMsgs[sun_index])
thermalSensor.stateInMsg.subscribeTo(scObject.scStateOutMsg)
thermalSensor.sunEclipseInMsg.subscribeTo(eclipseObject.eclipseOutMsgs[0])
scSim.AddModelToTask(simTaskName, thermalSensor)

save_path = os.path.abspath(CLI_ARGS.bin_path)
save_dir = os.path.dirname(save_path)
if save_dir:
    os.makedirs(save_dir, exist_ok=True)
if os.path.exists(save_path):
    os.remove(save_path)  # clear old run so Vizard doesn't load stale data

viz = enable_vizard(
    scSim,
    simTaskName,
    scObject,
    save_path,
    generic_storage_list=viz_storage_list,
    live_stream=LIVE_STREAM_ENABLED,
    broadcast_stream=BROADCAST_STREAM_ENABLED,
)
live_telemetry_plot = LiveTelemetryPlot(enabled=LIVE_PLOTS_ENABLED, compression_power=0.5)  # [-]

models_dir = os.path.join(os.getcwd(), "models")
os.makedirs(models_dir, exist_ok=True)
obj_path_open = os.path.join(models_dir, f"cubesat_{ADCS_MODE}_open.obj")
obj_path_closed = os.path.join(models_dir, f"cubesat_{ADCS_MODE}_closed.obj")
build_satellite_obj(
    obj_path_open,
    panels_open=True,
    body_size_x_m=BODY_SIZE_X_M,
    body_size_y_m=BODY_SIZE_Y_M,
    body_size_z_m=BODY_SIZE_Z_M,
)
build_satellite_obj(
    obj_path_closed,
    panels_open=False,
    body_size_x_m=BODY_SIZE_X_M,
    body_size_y_m=BODY_SIZE_Y_M,
    body_size_z_m=BODY_SIZE_Z_M,
)

initial_visual_state = "OPEN" if init_state == "CHARGING" else "CLOSED"
initial_visual_path = obj_path_open if initial_visual_state == "OPEN" else obj_path_closed
apply_visual_model(viz, scObject.ModelTag, initial_visual_path)
print(f"Visual model: panels={initial_visual_state} (based on initial ADCS state={init_state})")

add_vizard_scene_overlays(
    viz,
    spacecraft_tag=scObject.ModelTag,
    vec_lost_b=VEC_LOST_B,
    vec_found_b=VEC_FOUND_B,
    pos_lost_b=POS_LOST_B,
    pos_found_b=POS_FOUND_B,
    lost_fov_deg=LOST_FOV_DEG,
    found_fov_deg=FOUND_FOV_DEG,
    lost_half_deg=LOST_HALF_DEG,
    found_half_deg=FOUND_HALF_DEG,
    lost_excl_half_deg=LOST_EXCL_HALF_DEG,
    found_excl_half_deg=FOUND_EXCL_HALF_DEG,
    ground_station_specs=CLI_ARGS.ground_station,
    earth_body_name=earth.displayName,
)
status_overlay_station = add_spacecraft_status_overlay(viz, scObject.ModelTag)
status_overlay_update_period_sec = 1.0  # [s]
status_overlay_update_period_nanos = macros.sec2nano(status_overlay_update_period_sec)
last_status_overlay_update_nanos = None
live_plot_update_period_sec = 10.0  # [s]
live_plot_update_period_nanos = macros.sec2nano(live_plot_update_period_sec)
last_live_plot_update_nanos = None

print(f"Mode: {ADCS_MODE}")
print(f"Guidance backend: {CLI_ARGS.guidance_backend}")
print(f"GNSS fix cadence: {CLI_ARGS.gnss_fix_period:.1f} s, duration: {CLI_ARGS.gnss_fix_duration:.1f} s")
print(f"GNSS zenith half-angle: {CLI_ARGS.gnss_zenith_half_angle:.1f} deg")
print(f"GNSS dead-reckoning limit: {CLI_ARGS.gnss_dead_reckoning:.1f} s")
print(f"Downlink window: {CLI_ARGS.downlink_window:.1f} s")
print("Ground stations:")
for station_spec in CLI_ARGS.ground_station:
    print(f"  - {station_spec}")
scSim.InitializeSimulation()

hil_orchestrator = _initialize_hil_orchestrator(CLI_ARGS)

stop_time_nanos = macros.hour2nano(SIM_HOURS)
visual_update_step_nanos = macros.sec2nano(1.0)
next_stop_nanos = 0
current_visual_state = None
# current_visual_state = initial_visual_state
last_reported_state = None
last_reported_station = "NONE"

telemetry_soc_pct = 100.0 * battery_initial_charge_whr / battery_capacity_whr  # [%]
telemetry_temp_c = sensor_initial_temp_c  # [C]
telemetry_net_power_w = 0.0  # [W]

sampled_time_nanos = 0
lost_uptime_nanos = 0
found_uptime_nanos = 0

sampled_time_charging_nanos = 0
sampled_time_experiment_nanos = 0
lost_uptime_charging_nanos = 0
lost_uptime_experiment_nanos = 0
found_uptime_charging_nanos = 0
found_uptime_experiment_nanos = 0

STATE_NAMES = ("CHARGING", "EXPERIMENT", "GNSS_FIX", "DOWNLINK")
sampled_time_by_state_nanos = {state_name: 0 for state_name in STATE_NAMES}
lost_uptime_by_state_nanos = {state_name: 0 for state_name in STATE_NAMES}
found_uptime_by_state_nanos = {state_name: 0 for state_name in STATE_NAMES}

lost_experiment_fail_nanos_by_body = {
    "earth": 0,
    "sun": 0,
    "moon": 0,
}

lost_charging_fail_nanos_by_body = {
    "earth": 0,
    "sun": 0,
    "moon": 0,
}

found_charging_fail_nanos_by_reason = {
    "earth_not_visible": 0,
    "sun_keepout_violation": 0,
}

found_experiment_fail_nanos_by_reason = {
    "earth_not_visible": 0,
    "sun_keepout_violation": 0,
}

lost_gnss_fail_nanos_by_body = {
    "earth": 0,  # [ns]
    "sun": 0,  # [ns]
    "moon": 0,  # [ns]
}

lost_downlink_fail_nanos_by_body = {
    "earth": 0,  # [ns]
    "sun": 0,  # [ns]
    "moon": 0,  # [ns]
}

found_gnss_fail_nanos_by_reason = {
    "earth_not_visible": 0,  # [ns]
    "sun_keepout_violation": 0,  # [ns]
}

found_downlink_fail_nanos_by_reason = {
    "earth_not_visible": 0,  # [ns]
    "sun_keepout_violation": 0,  # [ns]
}

uptime_warning_printed = False

uptime_history = {
    "time_hours": [],
    "lost_overall_pct": [],
    "found_overall_pct": [],
    "lost_charging_pct": [],
    "found_charging_pct": [],
    "lost_experiment_pct": [],
    "found_experiment_pct": [],
    "lost_gnss_pct": [],
    "found_gnss_pct": [],
    "lost_downlink_pct": [],
    "found_downlink_pct": [],
}

while next_stop_nanos < stop_time_nanos:
    previous_stop_nanos = next_stop_nanos
    next_stop_nanos = min(next_stop_nanos + visual_update_step_nanos, stop_time_nanos)

    commanded_state = _get_guidance_state(guidance)
    payload_is_on = commanded_state != "CHARGING"
    payload_power_status = 1 if payload_is_on else 0
    payloadPowerSink.powerStatus = payload_power_status
    thermalSensor.sensorPowerStatus = payload_power_status

    scSim.ConfigureStopTime(next_stop_nanos)
    scSim.ExecuteSimulation()

    if CLI_ARGS.hil_enable:
        _update_hil_orchestrator(hil_orchestrator, scObject.scStateOutMsg)

    dt_nanos = next_stop_nanos - previous_stop_nanos
    if dt_nanos > 0:
        sampled_time_nanos += dt_nanos
        try:
            sc_state_now = scObject.scStateOutMsg.read()
            sun_state_now = gravFactory.spiceObject.planetStateOutMsgs[sun_index].read()
            moon_state_now = gravFactory.spiceObject.planetStateOutMsgs[moon_index].read()
            lost_ok, found_ok, uptime_details = compute_camera_uptime_flags(
                sc_state_now,
                sun_state_now,
                moon_state_now,
                vec_lost_b=VEC_LOST_B,
                vec_found_b=VEC_FOUND_B,
                pos_lost_b=POS_LOST_B,
                pos_found_b=POS_FOUND_B,
                lost_inner_keepout_half_deg=LOST_EXCL_HALF_DEG,
                found_earth_keepin_half_deg=FOUND_EXCL_HALF_DEG,
                found_sun_keepout_half_deg=FOUND_EXCL_HALF_DEG,
                return_details=True,
            )
            if lost_ok:
                lost_uptime_nanos += dt_nanos
            if found_ok:
                found_uptime_nanos += dt_nanos

            sample_state = _get_guidance_state(guidance)
            if sample_state not in STATE_NAMES:
                if ADCS_MODE == "ROLL_ONLY":
                    sample_state = "CHARGING"
                elif ADCS_MODE == "EXPERIMENT":
                    sample_state = "EXPERIMENT"

            if sample_state in STATE_NAMES:
                sampled_time_by_state_nanos[sample_state] += dt_nanos
                if lost_ok:
                    lost_uptime_by_state_nanos[sample_state] += dt_nanos
                if found_ok:
                    found_uptime_by_state_nanos[sample_state] += dt_nanos

            if sample_state == "CHARGING":
                sampled_time_charging_nanos += dt_nanos
                if lost_ok:
                    lost_uptime_charging_nanos += dt_nanos
                if found_ok:
                    found_uptime_charging_nanos += dt_nanos
                for body_name in ("earth", "sun", "moon"):
                    if not uptime_details["lost_body_ok"][body_name]:
                        lost_charging_fail_nanos_by_body[body_name] += dt_nanos

                found_earth_visible = (
                    uptime_details["found_earth_ang_deg"]
                    <= (FOUND_EXCL_HALF_DEG + uptime_details["earth_half_angle_deg"])
                )
                found_sun_clear = uptime_details["found_sun_ang_deg"] >= FOUND_EXCL_HALF_DEG
                if not found_earth_visible:
                    found_charging_fail_nanos_by_reason["earth_not_visible"] += dt_nanos
                if not found_sun_clear:
                    found_charging_fail_nanos_by_reason["sun_keepout_violation"] += dt_nanos
            elif sample_state in ("EXPERIMENT", "GNSS_FIX", "DOWNLINK"):
                sampled_time_experiment_nanos += dt_nanos
                if lost_ok:
                    lost_uptime_experiment_nanos += dt_nanos
                if found_ok:
                    found_uptime_experiment_nanos += dt_nanos
                for body_name in ("earth", "sun", "moon"):
                    if not uptime_details["lost_body_ok"][body_name]:
                        lost_experiment_fail_nanos_by_body[body_name] += dt_nanos

                found_earth_visible = (
                    uptime_details["found_earth_ang_deg"]
                    <= (FOUND_EXCL_HALF_DEG + uptime_details["earth_half_angle_deg"])
                )
                found_sun_clear = uptime_details["found_sun_ang_deg"] >= FOUND_EXCL_HALF_DEG
                if not found_earth_visible:
                    found_experiment_fail_nanos_by_reason["earth_not_visible"] += dt_nanos
                if not found_sun_clear:
                    found_experiment_fail_nanos_by_reason["sun_keepout_violation"] += dt_nanos

                if sample_state == "GNSS_FIX":
                    for body_name in ("earth", "sun", "moon"):
                        if not uptime_details["lost_body_ok"][body_name]:
                            lost_gnss_fail_nanos_by_body[body_name] += dt_nanos
                    if not found_earth_visible:
                        found_gnss_fail_nanos_by_reason["earth_not_visible"] += dt_nanos
                    if not found_sun_clear:
                        found_gnss_fail_nanos_by_reason["sun_keepout_violation"] += dt_nanos
                elif sample_state == "DOWNLINK":
                    for body_name in ("earth", "sun", "moon"):
                        if not uptime_details["lost_body_ok"][body_name]:
                            lost_downlink_fail_nanos_by_body[body_name] += dt_nanos
                    if not found_earth_visible:
                        found_downlink_fail_nanos_by_reason["earth_not_visible"] += dt_nanos
                    if not found_sun_clear:
                        found_downlink_fail_nanos_by_reason["sun_keepout_violation"] += dt_nanos
        except Exception as exc:
            if not uptime_warning_printed:
                print(f"[UPTIME] sample evaluation warning: {exc}")
                uptime_warning_printed = True

        time_hours = next_stop_nanos * 1.0e-9 / 3600.0
        uptime_history["time_hours"].append(time_hours)

        overall_lost_pct = 100.0 * lost_uptime_nanos / sampled_time_nanos
        overall_found_pct = 100.0 * found_uptime_nanos / sampled_time_nanos
        uptime_history["lost_overall_pct"].append(overall_lost_pct)
        uptime_history["found_overall_pct"].append(overall_found_pct)

        charging_lost_pct = (
            100.0 * lost_uptime_charging_nanos / sampled_time_charging_nanos
            if sampled_time_charging_nanos > 0
            else np.nan
        )
        charging_found_pct = (
            100.0 * found_uptime_charging_nanos / sampled_time_charging_nanos
            if sampled_time_charging_nanos > 0
            else np.nan
        )
        experiment_lost_pct = (
            100.0 * lost_uptime_experiment_nanos / sampled_time_experiment_nanos
            if sampled_time_experiment_nanos > 0
            else np.nan
        )
        experiment_found_pct = (
            100.0 * found_uptime_experiment_nanos / sampled_time_experiment_nanos
            if sampled_time_experiment_nanos > 0
            else np.nan
        )

        uptime_history["lost_charging_pct"].append(charging_lost_pct)
        uptime_history["found_charging_pct"].append(charging_found_pct)
        uptime_history["lost_experiment_pct"].append(experiment_lost_pct)
        uptime_history["found_experiment_pct"].append(experiment_found_pct)

        gnss_lost_pct = (
            100.0 * lost_uptime_by_state_nanos["GNSS_FIX"] / sampled_time_by_state_nanos["GNSS_FIX"]
            if sampled_time_by_state_nanos["GNSS_FIX"] > 0
            else np.nan
        )
        gnss_found_pct = (
            100.0 * found_uptime_by_state_nanos["GNSS_FIX"] / sampled_time_by_state_nanos["GNSS_FIX"]
            if sampled_time_by_state_nanos["GNSS_FIX"] > 0
            else np.nan
        )
        downlink_lost_pct = (
            100.0 * lost_uptime_by_state_nanos["DOWNLINK"] / sampled_time_by_state_nanos["DOWNLINK"]
            if sampled_time_by_state_nanos["DOWNLINK"] > 0
            else np.nan
        )
        downlink_found_pct = (
            100.0 * found_uptime_by_state_nanos["DOWNLINK"] / sampled_time_by_state_nanos["DOWNLINK"]
            if sampled_time_by_state_nanos["DOWNLINK"] > 0
            else np.nan
        )
        uptime_history["lost_gnss_pct"].append(gnss_lost_pct)
        uptime_history["found_gnss_pct"].append(gnss_found_pct)
        uptime_history["lost_downlink_pct"].append(downlink_lost_pct)
        uptime_history["found_downlink_pct"].append(downlink_found_pct)

    active_state = _get_guidance_state(guidance)
    active_station = _get_active_ground_station(guidance)
    state_changed = active_state != last_reported_state
    station_changed = active_station != last_reported_station

    if state_changed or station_changed:
        print(
            f"[MODE] t={next_stop_nanos * 1.0e-9:8.1f}s "
            f"state={active_state} station={active_station}"
        )
        last_reported_state = active_state
        last_reported_station = active_station

    battery_state = powerMonitor.batPowerOutMsg.read()
    thermal_state = thermalSensor.temperatureOutMsg.read()
    telemetry_net_power_w = float(battery_state.currentNetPower)
    telemetry_temp_c = float(thermal_state.temperature)
    if powerMonitor.storageCapacity > 0.0:
        telemetry_soc_pct = float(
            np.clip(
                100.0 * battery_state.storageLevel / powerMonitor.storageCapacity, 0.0, 100.0
            )
        )
    else:
        telemetry_soc_pct = 0.0
    is_charging_state = telemetry_net_power_w >= 0.0
    should_update_overlay = (
        last_status_overlay_update_nanos is None
        or (next_stop_nanos - last_status_overlay_update_nanos) >= status_overlay_update_period_nanos
        or state_changed
        or station_changed
    )
    if should_update_overlay:
        update_spacecraft_status_overlay(
            viz,
            station_name=status_overlay_station,
            is_charging=is_charging_state,
            guidance_state=active_state,
            net_power_w=telemetry_net_power_w,
            temp_c=telemetry_temp_c,
            soc_pct=telemetry_soc_pct,
        )
        last_status_overlay_update_nanos = next_stop_nanos

    if LIVE_PLOTS_ENABLED:
        should_update_live_plot = (
            last_live_plot_update_nanos is None
            or (next_stop_nanos - last_live_plot_update_nanos) >= live_plot_update_period_nanos
            or state_changed
            or station_changed
        )
        if should_update_live_plot:
            live_telemetry_plot.update(
                next_stop_nanos * 1.0e-9,
                net_power_w=telemetry_net_power_w,
                soc_pct=telemetry_soc_pct,
                temp_c=telemetry_temp_c,
                mission_state=active_state,
            )
            last_live_plot_update_nanos = next_stop_nanos

    if active_state == "CHARGING":
        target_visual_state = "OPEN"
    elif active_state in ("EXPERIMENT", "GNSS_FIX", "DOWNLINK"):
        target_visual_state = "CLOSED"
    else:
        target_visual_state = current_visual_state

    if target_visual_state != current_visual_state:
        target_model = obj_path_open if target_visual_state == "OPEN" else obj_path_closed
        apply_visual_model(viz, scObject.ModelTag, target_model)
        print(f"[VIZ] t={next_stop_nanos * 1.0e-9:8.1f}s switched visual model -> {target_visual_state}")
        current_visual_state = target_visual_state

if sampled_time_nanos > 0:
    lost_uptime_pct = 100.0 * lost_uptime_nanos / sampled_time_nanos
    found_uptime_pct = 100.0 * found_uptime_nanos / sampled_time_nanos
else:
    lost_uptime_pct = 0.0
    found_uptime_pct = 0.0

if sampled_time_charging_nanos > 0:
    lost_uptime_charging_pct = 100.0 * lost_uptime_charging_nanos / sampled_time_charging_nanos
    found_uptime_charging_pct = 100.0 * found_uptime_charging_nanos / sampled_time_charging_nanos
else:
    lost_uptime_charging_pct = None
    found_uptime_charging_pct = None

if sampled_time_experiment_nanos > 0:
    lost_uptime_experiment_pct = 100.0 * lost_uptime_experiment_nanos / sampled_time_experiment_nanos
    found_uptime_experiment_pct = 100.0 * found_uptime_experiment_nanos / sampled_time_experiment_nanos
else:
    lost_uptime_experiment_pct = None
    found_uptime_experiment_pct = None

print(f"[UPTIME] LOST clear uptime (total):    {lost_uptime_pct:6.2f}%")
print(f"[UPTIME] FOUND valid uptime (total):   {found_uptime_pct:6.2f}%")
if lost_uptime_charging_pct is None:
    print("[UPTIME] LOST clear uptime (CHARGING): N/A (state not visited)")
else:
    print(f"[UPTIME] LOST clear uptime (CHARGING): {lost_uptime_charging_pct:6.2f}%")
if found_uptime_charging_pct is None:
    print("[UPTIME] FOUND valid uptime (CHARGING): N/A (state not visited)")
else:
    print(f"[UPTIME] FOUND valid uptime (CHARGING): {found_uptime_charging_pct:6.2f}%")
if lost_uptime_experiment_pct is None:
    print("[UPTIME] LOST clear uptime (EXPERIMENT): N/A (state not visited)")
else:
    print(f"[UPTIME] LOST clear uptime (EXPERIMENT): {lost_uptime_experiment_pct:6.2f}%")
if found_uptime_experiment_pct is None:
    print("[UPTIME] FOUND valid uptime (EXPERIMENT): N/A (state not visited)")
else:
    print(f"[UPTIME] FOUND valid uptime (EXPERIMENT): {found_uptime_experiment_pct:6.2f}%")

if sampled_time_experiment_nanos > 0:
    print("[UPTIME] LOST EXPERIMENT fail-time breakdown (inner red keep-out):")
    for body_name in ("earth", "sun", "moon"):
        fail_pct = 100.0 * lost_experiment_fail_nanos_by_body[body_name] / sampled_time_experiment_nanos
        print(f"[UPTIME]   {body_name.upper():>5}: {fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when multiple bodies violate at once.")

if sampled_time_charging_nanos > 0:
    print("[UPTIME] LOST CHARGING fail-time breakdown (inner red keep-out):")
    for body_name in ("earth", "sun", "moon"):
        fail_pct = 100.0 * lost_charging_fail_nanos_by_body[body_name] / sampled_time_charging_nanos
        print(f"[UPTIME]   {body_name.upper():>5}: {fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when multiple bodies violate at once.")

if sampled_time_charging_nanos > 0:
    print("[UPTIME] FOUND CHARGING fail-time breakdown:")
    charge_earth_fail_pct = (
        100.0 * found_charging_fail_nanos_by_reason["earth_not_visible"] / sampled_time_charging_nanos
    )
    charge_sun_fail_pct = (
        100.0 * found_charging_fail_nanos_by_reason["sun_keepout_violation"] / sampled_time_charging_nanos
    )
    print(f"[UPTIME]   EARTH_NOT_VISIBLE:    {charge_earth_fail_pct:6.2f}%")
    print(f"[UPTIME]   SUN_KEEPOUT_VIOLATION:{charge_sun_fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when both FOUND conditions fail at once.")

if sampled_time_experiment_nanos > 0:
    print("[UPTIME] FOUND EXPERIMENT fail-time breakdown:")
    exp_earth_fail_pct = (
        100.0 * found_experiment_fail_nanos_by_reason["earth_not_visible"] / sampled_time_experiment_nanos
    )
    exp_sun_fail_pct = (
        100.0 * found_experiment_fail_nanos_by_reason["sun_keepout_violation"] / sampled_time_experiment_nanos
    )
    print(f"[UPTIME]   EARTH_NOT_VISIBLE:    {exp_earth_fail_pct:6.2f}%")
    print(f"[UPTIME]   SUN_KEEPOUT_VIOLATION:{exp_sun_fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when both FOUND conditions fail at once.")

gnss_time_nanos = sampled_time_by_state_nanos.get("GNSS_FIX", 0)  # [ns]
if gnss_time_nanos > 0:
    print("[UPTIME] LOST GNSS_FIX fail-time breakdown (inner red keep-out):")
    for body_name in ("earth", "sun", "moon"):
        fail_pct = 100.0 * lost_gnss_fail_nanos_by_body[body_name] / gnss_time_nanos  # [%]
        print(f"[UPTIME]   {body_name.upper():>5}: {fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when multiple bodies violate at once.")

    print("[UPTIME] FOUND GNSS_FIX fail-time breakdown:")
    gnss_earth_fail_pct = (
        100.0 * found_gnss_fail_nanos_by_reason["earth_not_visible"] / gnss_time_nanos
    )  # [%]
    gnss_sun_fail_pct = (
        100.0 * found_gnss_fail_nanos_by_reason["sun_keepout_violation"] / gnss_time_nanos
    )  # [%]
    print(f"[UPTIME]   EARTH_NOT_VISIBLE:    {gnss_earth_fail_pct:6.2f}%")
    print(f"[UPTIME]   SUN_KEEPOUT_VIOLATION:{gnss_sun_fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when both FOUND conditions fail at once.")

downlink_time_nanos = sampled_time_by_state_nanos.get("DOWNLINK", 0)  # [ns]
if downlink_time_nanos > 0:
    print("[UPTIME] LOST DOWNLINK fail-time breakdown (inner red keep-out):")
    for body_name in ("earth", "sun", "moon"):
        fail_pct = 100.0 * lost_downlink_fail_nanos_by_body[body_name] / downlink_time_nanos  # [%]
        print(f"[UPTIME]   {body_name.upper():>5}: {fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when multiple bodies violate at once.")

    print("[UPTIME] FOUND DOWNLINK fail-time breakdown:")
    downlink_earth_fail_pct = (
        100.0 * found_downlink_fail_nanos_by_reason["earth_not_visible"] / downlink_time_nanos
    )  # [%]
    downlink_sun_fail_pct = (
        100.0 * found_downlink_fail_nanos_by_reason["sun_keepout_violation"] / downlink_time_nanos
    )  # [%]
    print(f"[UPTIME]   EARTH_NOT_VISIBLE:    {downlink_earth_fail_pct:6.2f}%")
    print(f"[UPTIME]   SUN_KEEPOUT_VIOLATION:{downlink_sun_fail_pct:6.2f}%")
    print("[UPTIME]   Note: percentages can overlap when both FOUND conditions fail at once.")

print("[UPTIME] State-by-state availability summary:")
for state_name in STATE_NAMES:
    state_time_nanos = sampled_time_by_state_nanos[state_name]
    if state_time_nanos <= 0:
        print(f"[UPTIME]   {state_name:>9}: not visited")
        continue

    state_time_pct = 100.0 * state_time_nanos / sampled_time_nanos
    state_lost_pct = 100.0 * lost_uptime_by_state_nanos[state_name] / state_time_nanos
    state_found_pct = 100.0 * found_uptime_by_state_nanos[state_name] / state_time_nanos
    print(
        f"[UPTIME]   {state_name:>9}: time={state_time_pct:6.2f}% "
        f"LOST={state_lost_pct:6.2f}% FOUND={state_found_pct:6.2f}%"
    )

plot_paths = _generate_uptime_plots(save_path, uptime_history)
if END_OF_RUN_PLOTS_ENABLED and plot_paths:
    print("[PLOT] Wrote uptime convergence plots:")
    for path in plot_paths:
        print(f"[PLOT]   {path}")

csv_path = _write_uptime_points_csv(save_path, uptime_history)
if END_OF_RUN_PLOTS_ENABLED:
    print(f"[PLOT] Wrote uptime points CSV: {csv_path}")

eps_plot_paths = live_telemetry_plot.save_eps_plots(save_path)
if eps_plot_paths:
    print("[PLOT] Wrote EPS telemetry plots:")
    for path in eps_plot_paths:
        print(f"[PLOT]   {path}")

txt_path = _write_uptime_summary_txt(
    save_path,
    lost_uptime_pct, found_uptime_pct,
    lost_uptime_charging_pct, found_uptime_charging_pct,
    lost_uptime_experiment_pct, found_uptime_experiment_pct,
    lost_experiment_fail_nanos_by_body, lost_charging_fail_nanos_by_body,
    found_charging_fail_nanos_by_reason, found_experiment_fail_nanos_by_reason,
    lost_gnss_fail_nanos_by_body, lost_downlink_fail_nanos_by_body,
    found_gnss_fail_nanos_by_reason, found_downlink_fail_nanos_by_reason,
    sampled_time_nanos, sampled_time_charging_nanos, sampled_time_experiment_nanos,
    sampled_time_by_state_nanos, lost_uptime_by_state_nanos, found_uptime_by_state_nanos,
    STATE_NAMES
)
print(f"[SUMMARY] Wrote uptime summary TXT: {txt_path}")

if CLI_ARGS.hil_enable:
    _shutdown_hil_orchestrator(hil_orchestrator)

print(f"Done! Load {save_path} in Vizard.")
