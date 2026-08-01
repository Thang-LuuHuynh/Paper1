#!/usr/bin/env python3
"""Publication figures comparing Backstepping, BSMC, and pure SMC.

The report consumes the CSV and summary JSON files produced by
``EmbeddedPaperCapture``.  It never smooths or modifies source measurements.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from amr_control.publication_aggregator import (
    _local_arrays,
    _read_csv,
    _save,
    _setup_matplotlib,
)


CONTROLLERS = ("Backstepping", "BSMC", "SMC")
COLORS = {
    "Backstepping": "#E69F00",
    "BSMC": "#0072B2",
    "SMC": "#CC79A7",
}
DISPLAY = {
    "Backstepping": "Backstepping",
    "BSMC": "BSMC",
    "SMC": "SMC",
}


def _load_summaries(root):
    summaries = []
    for path in sorted(Path(root).glob("*_summary.json")):
        try:
            summary = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if (
            summary.get("controller") not in CONTROLLERS
            or summary.get("trajectory") not in {"circle", "eight", "square"}
            or not summary.get("valid", True)
            or int(summary.get("n_metric", 0)) < 3
        ):
            continue
        csv_path = Path(str(summary.get("csv", ""))).expanduser()
        if not csv_path.exists():
            csv_path = path.parent / csv_path.name
        if not csv_path.exists():
            continue
        summary = dict(summary)
        summary["_summary_path"] = str(path)
        summary["_csv_path"] = str(csv_path)
        summaries.append(summary)
    return summaries


def _latest_complete_sets(root):
    grouped = {}
    for summary in _load_summaries(root):
        key = (summary["trajectory"], summary["controller"])
        current = grouped.get(key)
        if current is None or summary["_summary_path"] > current["_summary_path"]:
            grouped[key] = summary
    result = {}
    for trajectory in ("circle", "square", "eight"):
        selected = {
            controller: grouped.get((trajectory, controller))
            for controller in CONTROLLERS
        }
        result[trajectory] = selected
    return result


def _arrays(summary):
    path_rotation = float(
        summary.get("parameters", {}).get("path_rotation_deg", 0.0)
    )
    frame_rotation = -path_rotation if summary["trajectory"] == "eight" else 0.0
    return _local_arrays(
        _read_csv(summary["_csv_path"]),
        figure_rotation_deg=frame_rotation,
    )


def _relative_motion_time(data):
    import numpy as np

    t = data["t"].copy()
    moving = np.isfinite(data["cmd_v"]) & (np.abs(data["cmd_v"]) > 0.005)
    if moving.any():
        t = t - float(t[np.flatnonzero(moving)[0]])
    else:
        finite = np.flatnonzero(np.isfinite(t))
        if finite.size:
            t = t - float(t[finite[0]])
    return t


def _active_motion_mask(data):
    """Keep only the commanded-motion interval used by comparison plots."""
    import numpy as np

    t = _relative_motion_time(data)
    moving = (
        np.isfinite(data["cmd_v"])
        & np.isfinite(t)
        & (np.abs(data["cmd_v"]) > 0.005)
    )
    indices = np.flatnonzero(moving)
    if not indices.size:
        return np.isfinite(t) & (t >= 0.0)
    return (
        np.isfinite(t)
        & (t >= 0.0)
        & (np.arange(t.size) <= indices[-1])
    )


def _fresh_camera(data):
    import numpy as np

    return (
        np.isfinite(data["camera_x_local"])
        & np.isfinite(data["camera_y_local"])
        & np.isfinite(data["camera_age_s"])
        & (data["camera_age_s"] <= 0.30)
    )


def _trajectory_figure(
    plt, output, trajectory, data_by_controller, controllers, tag
):
    import numpy as np

    fig, ax = plt.subplots(figsize=(3.5, 3.2), constrained_layout=True)
    reference = data_by_controller.get("BSMC", data_by_controller[controllers[0]])
    reference_active = _active_motion_mask(reference)
    ax.plot(
        reference["desired_x_local"][reference_active],
        reference["desired_y_local"][reference_active],
        "--",
        color="#222222",
        lw=1.5,
        label="Reference",
    )
    for controller in controllers:
        data = data_by_controller[controller]
        valid = _fresh_camera(data) & _active_motion_mask(data)
        ax.plot(
            data["camera_x_local"][valid],
            data["camera_y_local"][valid],
            color=COLORS[controller],
            lw=1.05,
            alpha=0.92,
            label=DISPLAY[controller],
        )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3, lw=0.5)
    ax.legend(loc="best", frameon=True)
    paths = _save(fig, output, f"{trajectory}_{tag}_trajectory")
    plt.close(fig)
    return paths


def _errors_figure(
    plt, output, trajectory, data_by_controller, controllers, tag
):
    fig, axes = plt.subplots(
        3, 1, figsize=(3.5, 4.8), sharex=True, constrained_layout=True
    )
    fields = ("camera_error_ex", "camera_error_ey", "camera_error_etheta")
    labels = (r"$e_x$ (m)", r"$e_y$ (m)", r"$e_\theta$ (rad)")
    for controller in controllers:
        data = data_by_controller[controller]
        t = _relative_motion_time(data)
        active = _active_motion_mask(data)
        for ax, field in zip(axes, fields):
            ax.plot(
                t[active],
                data[field][active],
                color=COLORS[controller],
                lw=0.8,
                alpha=0.9,
                label=DISPLAY[controller],
            )
    for ax, label in zip(axes, labels):
        ax.axhline(0.0, color="#555555", lw=0.5, ls="--")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3, lw=0.5)
    axes[0].legend(loc="best", frameon=True, ncol=len(controllers))
    axes[-1].set_xlabel("time from motion onset (s)")
    paths = _save(fig, output, f"{trajectory}_{tag}_errors")
    plt.close(fig)
    suffixes = ("ex", "ey", "etheta")
    for field, label, suffix in zip(fields, labels, suffixes):
        fig, ax = plt.subplots(figsize=(3.5, 2.45), constrained_layout=True)
        for controller in controllers:
            data = data_by_controller[controller]
            t = _relative_motion_time(data)
            active = _active_motion_mask(data)
            ax.plot(
                t[active],
                data[field][active],
                color=COLORS[controller],
                lw=0.9,
                alpha=0.9,
                label=DISPLAY[controller],
            )
        ax.axhline(0.0, color="#555555", lw=0.5, ls="--")
        ax.set_xlabel("time from motion onset (s)")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3, lw=0.5)
        ax.legend(loc="best", frameon=True, ncol=len(controllers))
        paths += _save(
            fig, output, f"{trajectory}_{tag}_{suffix}"
        )
        plt.close(fig)
    return paths


def _commands_figure(
    plt, output, trajectory, data_by_controller, controllers, tag
):
    fig, axes = plt.subplots(
        2, 1, figsize=(3.5, 3.45), sharex=True, constrained_layout=True
    )
    fields = ("cmd_v", "cmd_w")
    labels = (r"$v_{cmd}$ (m/s)", r"$\omega_{cmd}$ (rad/s)")
    for controller in controllers:
        data = data_by_controller[controller]
        t = _relative_motion_time(data)
        active = _active_motion_mask(data)
        for ax, field in zip(axes, fields):
            ax.plot(
                t[active],
                data[field][active],
                color=COLORS[controller],
                lw=0.8,
                alpha=0.9,
                label=DISPLAY[controller],
            )
    for ax, label in zip(axes, labels):
        ax.axhline(0.0, color="#555555", lw=0.5, ls="--")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3, lw=0.5)
    axes[0].legend(loc="best", frameon=True, ncol=len(controllers))
    axes[-1].set_xlabel("time from motion onset (s)")
    paths = _save(fig, output, f"{trajectory}_{tag}_commands")
    plt.close(fig)
    suffixes = ("vcmd", "wcmd")
    for field, label, suffix in zip(fields, labels, suffixes):
        fig, ax = plt.subplots(figsize=(3.5, 2.45), constrained_layout=True)
        for controller in controllers:
            data = data_by_controller[controller]
            t = _relative_motion_time(data)
            active = _active_motion_mask(data)
            ax.plot(
                t[active],
                data[field][active],
                color=COLORS[controller],
                lw=0.9,
                alpha=0.9,
                label=DISPLAY[controller],
            )
        ax.axhline(0.0, color="#555555", lw=0.5, ls="--")
        ax.set_xlabel("time from motion onset (s)")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3, lw=0.5)
        ax.legend(loc="best", frameon=True, ncol=len(controllers))
        paths += _save(
            fig, output, f"{trajectory}_{tag}_{suffix}"
        )
        plt.close(fig)
    return paths


def _event_index(data):
    import numpy as np

    indices = np.flatnonzero(
        np.isfinite(data["disturbance_event"])
        & (data["disturbance_event"] > 0.5)
    )
    return int(indices[0]) if indices.size else None


def _disturbance_baseline(data, event_index, error):
    """Return nominal pre-disturbance baseline and its endpoint.

    For the paused displacement protocol, the baseline ends when pause starts,
    before the operator repositions the robot.  For an active perturbation or
    legacy log without ``control_paused``, it ends at the release event.
    """
    import numpy as np

    event_t = float(data["t"][event_index])
    baseline_end_t = event_t
    pause = data["control_paused"]
    paused_before = np.flatnonzero(
        (np.arange(len(pause)) < event_index)
        & np.isfinite(pause)
        & (pause > 0.5)
    )
    if paused_before.size:
        pause_start = int(paused_before[-1])
        while (
            pause_start > 0
            and np.isfinite(pause[pause_start - 1])
            and pause[pause_start - 1] > 0.5
        ):
            pause_start -= 1
        baseline_end_t = float(data["t"][pause_start])
    pre = (
        (data["t"] >= baseline_end_t - 3.0)
        & (data["t"] < baseline_end_t)
        & np.isfinite(error)
    )
    baseline = float(np.mean(error[pre])) if pre.any() else 0.0
    return baseline, baseline_end_t


def _disturbance_figures(
    plt, output, data_by_controller, controllers, tag
):
    import numpy as np

    if any(_event_index(data_by_controller[c]) is None for c in controllers):
        return []

    outputs = []
    fig, ax = plt.subplots(figsize=(3.5, 2.8), constrained_layout=True)
    disturbance_rows = {}
    for controller in controllers:
        data = data_by_controller[controller]
        event_index = _event_index(data)
        event_t = float(data["t"][event_index])
        relative_t = data["t"] - event_t
        error = np.hypot(data["camera_error_ex"], data["camera_error_ey"])
        baseline, baseline_end_t = _disturbance_baseline(
            data, event_index, error
        )
        incremental = error - baseline
        window = (relative_t >= -3.0) & (relative_t <= 15.0)
        ax.plot(
            relative_t[window],
            incremental[window],
            color=COLORS[controller],
            lw=1.0,
            label=DISPLAY[controller],
        )
        disturbance_rows[controller] = {
            "event_time_s": event_t,
            "pre_event_position_error_m": baseline,
            "baseline_end_time_s": baseline_end_t,
        }
    ax.axvline(0.0, color="#222222", ls="--", lw=0.8, label="release/resume")
    ax.axhline(
        0.03,
        color="#009E73",
        ls=":",
        lw=0.8,
        label="3 cm above pre-event baseline",
    )
    ax.set_xlabel("time from disturbance release/resume (s)")
    ax.set_ylabel(r"$e_p-\bar e_{pre}$ (m)")
    ax.grid(True, alpha=0.3, lw=0.5)
    ax.legend(loc="best", frameon=True)
    outputs += _save(fig, output, f"circle_{tag}_disturbance_recovery")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(3.5, 3.2), constrained_layout=True)
    reference = data_by_controller.get("BSMC", data_by_controller[controllers[0]])
    ax.plot(
        reference["desired_x_local"],
        reference["desired_y_local"],
        "--",
        color="#222222",
        lw=1.3,
        label="Reference",
    )
    for controller in controllers:
        data = data_by_controller[controller]
        event_index = _event_index(data)
        event_t = float(data["t"][event_index])
        window = (
            (data["t"] >= event_t - 3.0)
            & (data["t"] <= event_t + 15.0)
            & _fresh_camera(data)
        )
        ax.plot(
            data["camera_x_local"][window],
            data["camera_y_local"][window],
            color=COLORS[controller],
            lw=1.1,
            label=DISPLAY[controller],
        )
        if window.any():
            nearest = int(np.nanargmin(np.abs(data["t"] - event_t)))
            ax.plot(
                data["camera_x_local"][nearest],
                data["camera_y_local"][nearest],
                marker="o",
                ms=4,
                color=COLORS[controller],
                linestyle="none",
            )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3, lw=0.5)
    ax.legend(loc="best", frameon=True)
    outputs += _save(
        fig, output, f"circle_{tag}_disturbance_trajectory"
    )
    plt.close(fig)

    path = output / f"circle_{tag}_disturbance_alignment.json"
    path.write_text(json.dumps(disturbance_rows, indent=2), encoding="utf-8")
    outputs.append(path)
    return outputs


def _controller_tag(controllers):
    if tuple(controllers) == CONTROLLERS:
        return "three_controller"
    return "_".join(controller.lower() for controller in controllers)


def refresh_three_controller_assets(
    root, only=None, allow_partial=False, output_dir=None
):
    """Render every complete three-controller set found in ``root``.

    A directory may contain one trajectory only (recommended) or all three.
    Incomplete sets are reported in the returned dictionary and are not
    treated as errors so automatic export remains safe after run one and two.
    """
    root = Path(root)
    output = Path(output_dir) if output_dir else root / "publication_three"
    output.mkdir(parents=True, exist_ok=True)
    plt = _setup_matplotlib()
    selected_sets = _latest_complete_sets(root)
    requested = [only] if only else ["circle", "square", "eight"]
    generated = []
    missing = {}
    provenance = {}
    metric_rows = []
    for trajectory in requested:
        selected = selected_sets[trajectory]
        controllers = tuple(
            controller for controller in CONTROLLERS
            if selected.get(controller) is not None
        )
        absent = [
            controller
            for controller in CONTROLLERS
            if selected.get(controller) is None
        ]
        if not controllers or (absent and not allow_partial):
            missing[trajectory] = absent
            continue
        tag = _controller_tag(controllers)
        data_by_controller = {
            controller: _arrays(selected[controller])
            for controller in controllers
        }
        generated += _trajectory_figure(
            plt, output, trajectory, data_by_controller, controllers, tag
        )
        generated += _errors_figure(
            plt, output, trajectory, data_by_controller, controllers, tag
        )
        generated += _commands_figure(
            plt, output, trajectory, data_by_controller, controllers, tag
        )
        if trajectory == "circle":
            generated += _disturbance_figures(
                plt, output, data_by_controller, controllers, tag
            )
        provenance[trajectory] = {
            controller: {
                "run_id": selected[controller].get("run_id"),
                "csv": selected[controller]["_csv_path"],
                "summary": selected[controller]["_summary_path"],
            }
            for controller in controllers
        }
        for controller in controllers:
            summary = selected[controller]
            metric_rows.append({
                "trajectory": trajectory,
                "controller": controller,
                "run_id": summary.get("run_id", ""),
                "position_rmse_m": summary.get(
                    "camera_rmse_position_m", math.nan
                ),
                "heading_rmse_deg": summary.get(
                    "camera_rmse_heading_deg", math.nan
                ),
                "disturbance_peak_incremental_m": summary.get(
                    "disturbance_peak_incremental_position_error_m", math.nan
                ),
                "disturbance_iae10_m_s": summary.get(
                    "disturbance_iae_10_m_s", math.nan
                ),
                "disturbance_recovery_s": summary.get(
                    "disturbance_recovery_time_s", math.nan
                ),
                "csv": summary["_csv_path"],
            })
    if provenance:
        bundle_tag = (
            "available_controller" if allow_partial else "three_controller"
        )
        path = output / f"{bundle_tag}_provenance.json"
        path.write_text(
            json.dumps(
                {
                    "selection": "latest valid run per controller and trajectory",
                    "camera_freshness_limit_s": 0.30,
                    "processing": (
                        "camera-derived errors; rigid local-frame alignment; "
                        "no smoothing"
                    ),
                    "runs": provenance,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        generated.append(path)
        metrics_path = output / f"{bundle_tag}_metrics.csv"
        with metrics_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(metric_rows[0]))
            writer.writeheader()
            writer.writerows(metric_rows)
        generated.append(metrics_path)
    return {"generated": generated, "missing": missing}


def _parser():
    parser = argparse.ArgumentParser(
        description=(
            "Render Backstepping/BSMC/SMC trajectory, error, command, and "
            "Circle disturbance comparison figures."
        )
    )
    parser.add_argument(
        "--input-dir",
        required=True,
        help="Directory containing run CSV and *_summary.json files.",
    )
    parser.add_argument(
        "--trajectory",
        choices=("circle", "square", "eight"),
        help="Render one trajectory only; default scans all three.",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help=(
            "Render the available one/two-controller logs without inventing "
            "missing SMC data."
        ),
    )
    parser.add_argument(
        "--output-dir",
        help="Optional output directory; default is INPUT_DIR/publication_three.",
    )
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    result = refresh_three_controller_assets(
        args.input_dir,
        only=args.trajectory,
        allow_partial=args.allow_partial,
        output_dir=args.output_dir,
    )
    for path in result["generated"]:
        print(path)
    if result["missing"]:
        for trajectory, controllers in result["missing"].items():
            print(
                f"INCOMPLETE {trajectory}: missing "
                + ", ".join(controllers)
            )
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
