#!/usr/bin/env python3
"""
Experimental results: target trajectories vs measured ball paths.

Reads the per-frame logs written by 4_gui_track_w_mode.py (TRACK_LOG=...),
computes tracking metrics per pattern phase and writes the figures and LaTeX
tables used in the report.

    python3 analyze_runs.py            # writes figures/*.pdf|png, results_tables.tex, metrics.json

Data (data/runN.csv, recorded on the rig on 2026-10-24/25, 60 Hz):
    run1        first run after the detection change: old D term (derivative of
                the ball position only), raw per-frame Z           -> "before"
    run7..run10 final control: D term on (ball - target) velocity, median-
                smoothed Z, plate-height loop                      -> "after"

Definitions (also stated in the report):
    error e(t)   = |p_ball(t) - p_target(t)| in mm, p_target = the target the
                   Teensy reports (TARGET_POS), i.e. exactly what the PID used.
    steady state = last STEADY_S seconds of each 25 s pattern phase (the final
                   END_TRIM_S are dropped: the target already moves to the next pattern).
    settling     = time after the pattern switch until the 1 s moving RMS of
                   e(t) enters, and stays inside, 1.1 x the largest 1 s moving
                   RMS seen in the steady-state window.
    lag          = time shift of the measured path that best matches the target
                   (least mean squared distance), steady state only.
    amplitude    = mean ball distance from the origin / mean target distance.
"""
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
FIG = os.path.join(HERE, "figures")

AFTER_RUNS = [7, 8, 9, 10]
BEFORE_RUN = 1
SHOW_RUN = 9                     # run drawn in full colour in the figures
PATTERNS = {0: "CENTER", 1: "CIRCLE", 2: "STAR", 3: "FIGURE 8"}
STEADY_S = 15.0
MIN_PHASE_S = 20.0
END_TRIM_S = 0.5                 # the Teensy switches its target a few frames before the Pi's
                                 # pattern label does; drop those frames at the end of each phase
FPS = 60.0

# Computer Modern, like the LaTeX report the figures go into
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "savefig.bbox": "tight", "figure.dpi": 150,
                     "font.family": "serif", "font.serif": ["cmr10"], "mathtext.fontset": "cm",
                     "axes.formatter.use_mathtext": True, "axes.unicode_minus": False})
C_TARGET, C_BALL, C_OTHER, C_BEFORE = "#d62728", "#1f77b4", "#7f7f7f", "#ff7f0e"


# ---------------------------------------------------------------------------
# Ideal (mathematical) trajectories, as defined in teensy_code/src/main.cpp
# ---------------------------------------------------------------------------
def ideal_path(pattern, n=2000):
    if pattern == 1:                                  # circle r = 50 mm
        a = np.linspace(0, 2 * math.pi, n)
        return 50 * np.cos(a), 50 * np.sin(a)
    if pattern == 2:                                  # star 60/25 mm, 10 vertices
        idx = np.arange(11) % 10
        ang = math.pi / 2 + idx * math.pi / 5
        r = np.where(idx % 2 == 0, 60.0, 25.0)
        return r * np.cos(ang), r * np.sin(ang)
    if pattern == 3:                                  # figure 8, 60 x 30 mm
        a = np.linspace(0, 2 * math.pi, n)
        return 60 * np.sin(a), 30 * np.sin(2 * a)
    return np.zeros(1), np.zeros(1)


def distance_to_path(x, y, px, py):
    """Distance from each (x, y) to the polyline (px, py)."""
    ax, ay, bx, by = px[:-1], py[:-1], px[1:], py[1:]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    out = np.empty(len(x))
    for i, (qx, qy) in enumerate(zip(x, y)):
        u = np.clip(((qx - ax) * dx + (qy - ay) * dy) / np.where(L2 > 0, L2, 1), 0, 1)
        out[i] = np.min(np.hypot(ax + u * dx - qx, ay + u * dy - qy))
    return out


# ---------------------------------------------------------------------------
# Loading and segmenting
# ---------------------------------------------------------------------------
def load(run):
    d = np.genfromtxt(os.path.join(DATA, f"run{run}.csv"), delimiter=",", names=True)
    d = d[np.argsort(d["t"])]
    return d, d["t"] - d["t"][0]


def phases(d, t):
    """Complete pattern phases: (pattern, start index, end index)."""
    cut = np.flatnonzero(np.diff(d["pattern"]) != 0) + 1
    bounds = np.r_[0, cut, len(d)]
    for a, e in zip(bounds[:-1], bounds[1:]):
        if a > 0 and t[e - 1] - t[a] >= MIN_PHASE_S:   # starts at a real switch
            e = a + int(np.searchsorted(t[a:e], t[e - 1] - END_TRIM_S))
            yield int(d["pattern"][a]), a, e


def moving_rms(tt, err, found, window=1.0):
    out = np.full(len(tt), np.nan)
    j = 0
    for i in range(len(tt)):
        while tt[j] < tt[i] - window:
            j += 1
        m = found[j:i + 1]
        if m.sum() >= 0.5 * window * FPS:
            out[i] = math.sqrt(np.mean(err[j:i + 1][m] ** 2))
    return out


def best_lag(tt, x, y, tx, ty, max_lag_s=1.5):
    """Shift (s) of the ball path vs the target that minimises the mean squared distance.
    Positive: the ball is behind the target; negative: the ball is ahead of it."""
    n, dt = len(x), float((tt[-1] - tt[0]) / (len(tt) - 1))   # mean frame spacing
    best = (0, np.inf)
    for k in range(-int(max_lag_s / dt), int(max_lag_s / dt) + 1):
        if abs(k) >= n - 10:
            continue
        if k >= 0:
            ex, ey = x[k:] - tx[:n - k], y[k:] - ty[:n - k]
        else:
            ex, ey = x[:n + k] - tx[-k:], y[:n + k] - ty[-k:]
        mse = np.mean(ex * ex + ey * ey)
        if mse < best[1]:
            best = (k, mse)
    return best[0] * dt, math.sqrt(best[1])


def analyse_phase(d, t, pattern, a, e):
    tt = t[a:e] - t[a]
    found = d["found"][a:e] == 1
    x, y = d["x"][a:e], d["y"][a:e]
    tx, ty = d["target_x"][a:e], d["target_y"][a:e]
    err = np.hypot(x - tx, y - ty)

    steady = tt >= tt[-1] - STEADY_S
    rms1 = moving_rms(tt, err, found)
    band = 1.1 * np.nanmax(rms1[steady])
    above = np.flatnonzero(rms1 > band)
    settle = float(tt[above[-1]]) if len(above) else 0.0

    s = steady & found
    if pattern == 0:
        k = int(0.25 * FPS)
        xs = np.convolve(x, np.ones(k) / k, mode="same"); ys = np.convolve(y, np.ones(k) / k, mode="same")
        inner = s.copy(); inner[:k] = inner[-k:] = False
        noise = np.hypot(x - xs, y - ys)[inner]
    res = {
        "pattern": PATTERNS[pattern],
        "duration_s": float(tt[-1]),
        "found_pct": float(100 * found.mean()),
        "settling_s": settle,
        "band_mm": float(band),
        "mean_mm": float(err[s].mean()),
        "rms_mm": float(math.sqrt(np.mean(err[s] ** 2))),
        "p95_mm": float(np.percentile(err[s], 95)),
        "max_mm": float(err[s].max()),
    }
    if pattern == 0:
        res["noise_rms_mm"] = float(math.sqrt(np.mean(noise ** 2)))
    if pattern != 0:
        px, py = ideal_path(pattern)
        # the commanded target follows the mathematical curve (rate limiter inactive)
        res["target_dev_mm"] = float(np.max(distance_to_path(tx[s][::6], ty[s][::6], px, py)))
        lag, rms_shifted = best_lag(tt[s], x[s], y[s], tx[s], ty[s])
        res["lag_s"] = float(lag)
        res["rms_lag_comp_mm"] = float(rms_shifted)
        res["shape_dev_mm"] = float(np.mean(distance_to_path(x[s][::3], y[s][::3], px, py)))
        res["amplitude_ratio"] = float(np.mean(np.hypot(x[s], y[s])) / np.mean(np.hypot(tx[s], ty[s])))
    return res, dict(tt=tt, found=found, x=x, y=y, tx=tx, ty=ty, err=err, rms1=rms1,
                     steady=steady, settle=settle, band=band)


# ---------------------------------------------------------------------------
def main():
    os.makedirs(FIG, exist_ok=True)
    results, traces = {}, {}
    for run in [BEFORE_RUN] + AFTER_RUNS:
        d, t = load(run)
        for pattern, a, e in phases(d, t):
            r, tr = analyse_phase(d, t, pattern, a, e)
            # keep the first complete phase of each pattern per run
            results.setdefault(run, {}).setdefault(r["pattern"], r)
            traces.setdefault(run, {}).setdefault(r["pattern"], tr)
        traces[run]["_full"] = (d, t)

    with open(os.path.join(HERE, "metrics.json"), "w") as f:
        json.dump(results, f, indent=1)

    # ---------------- aggregate (after runs) ----------------
    keys = ["noise_rms_mm", "settling_s", "mean_mm", "rms_mm", "p95_mm", "max_mm", "lag_s", "rms_lag_comp_mm",
            "shape_dev_mm", "amplitude_ratio", "found_pct", "target_dev_mm"]
    agg = {}
    for name in PATTERNS.values():
        rows = [results[r][name] for r in AFTER_RUNS if name in results.get(r, {})]
        agg[name] = {"n": len(rows)}
        for k in keys:
            v = [row[k] for row in rows if k in row]
            if v:
                agg[name][k] = (float(np.mean(v)), float(np.std(v, ddof=1)) if len(v) > 1 else 0.0)
    with open(os.path.join(HERE, "metrics_summary.json"), "w") as f:
        json.dump(agg, f, indent=1)

    # ---------------- LaTeX tables ----------------
    def pm(name, k, fmt="{:.1f}"):
        m, s = agg[name][k]
        if k == "settling_s" and m == 0 and s == 0:
            return "$0$"
        return f"${fmt.format(m)} \\pm {fmt.format(s)}$"

    lines = [r"\begin{table}[H]", r"\centering",
             r"\begin{tabularx}{\textwidth}{|l|c|X|X|X|X|X|}", r"\hline",
             r"\textbf{Pattern} & \textbf{Runs} & \textbf{Settling [s]} & \textbf{Mean [mm]} & "
             r"\textbf{RMS [mm]} & \textbf{P95 [mm]} & \textbf{Max [mm]} \\ \hline"]
    for name in ["CENTER", "CIRCLE", "STAR", "FIGURE 8"]:
        lines.append(f"{name} & {agg[name]['n']} & {pm(name, 'settling_s')} & {pm(name, 'mean_mm')} & "
                     f"{pm(name, 'rms_mm')} & {pm(name, 'p95_mm')} & {pm(name, 'max_mm')} \\\\ \\hline")
    lines += [r"\end{tabularx}",
              r"\caption{\textit{Settling time and steady-state tracking error of the final configuration "
              r"(mean $\pm$ standard deviation over the runs).}}",
              r"\label{tab:tracking_error}", r"\end{table}", ""]

    lines += [r"\begin{table}[H]", r"\centering",
              r"\begin{tabularx}{\textwidth}{|l|X|X|X|X|}", r"\hline",
              r"\textbf{Pattern} & \textbf{Lag [s]} & \textbf{RMS without lag [mm]} & "
              r"\textbf{Shape deviation [mm]} & \textbf{Amplitude ratio} \\ \hline"]
    for name in ["CIRCLE", "STAR", "FIGURE 8"]:
        lines.append(f"{name} & {pm(name, 'lag_s', '{:.2f}')} & {pm(name, 'rms_lag_comp_mm')} & "
                     f"{pm(name, 'shape_dev_mm')} & {pm(name, 'amplitude_ratio', '{:.2f}')} \\\\ \\hline")
    lines += [r"\end{tabularx}",
              r"\caption{\textit{Decomposition of the tracking error on the moving patterns: time lag, error "
              r"without the lag, deviation from the ideal shape and amplitude ratio.}}",
              r"\label{tab:lag}", r"\end{table}", ""]

    b, a9 = results[BEFORE_RUN], agg
    lines += [r"\begin{table}[H]", r"\centering",
              r"\begin{tabularx}{\textwidth}{|l|X|X|X|}", r"\hline",
              r"\textbf{Pattern} & \textbf{Before: mean / P95 [mm]} & \textbf{After: mean / P95 [mm]} & "
              r"\textbf{Mean error reduction} \\ \hline"]
    for name in ["CIRCLE", "STAR", "FIGURE 8", "CENTER"]:
        if name not in b:
            continue
        bm, bp = b[name]["mean_mm"], b[name]["p95_mm"]
        am, ap = a9[name]["mean_mm"][0], a9[name]["p95_mm"][0]
        lines.append(f"{name} & {bm:.1f} / {bp:.1f} & {am:.1f} / {ap:.1f} & "
                     f"{100 * (1 - am / bm):.0f}\\,\\% \\\\ \\hline")
    lines += [r"\end{tabularx}",
              r"\caption{\textit{Steady-state tracking error before (run 1) and after (runs 7--10) the control "
              r"changes.}}",
              r"\label{tab:before_after}", r"\end{table}", ""]

    lines += [r"\begin{table}[H]", r"\centering",
              r"\begin{tabularx}{\textwidth}{|c|l|X|X|X|X|X|}", r"\hline",
              r"\textbf{Run} & \textbf{Pattern} & \textbf{Settling [s]} & \textbf{Mean [mm]} & \textbf{RMS [mm]} & "
              r"\textbf{P95 [mm]} & \textbf{Max [mm]} \\ \hline"]
    for run in AFTER_RUNS:
        for name in ["CENTER", "CIRCLE", "STAR", "FIGURE 8"]:
            r = results[run].get(name)
            if r:
                lines.append(f"{run} & {name} & {r['settling_s']:.2f} & {r['mean_mm']:.1f} & {r['rms_mm']:.1f} & "
                             f"{r['p95_mm']:.1f} & {r['max_mm']:.1f} \\\\ \\hline")
    lines += [r"\end{tabularx}",
              r"\caption{\textit{Settling time and steady-state tracking error for each run.}}",
              r"\label{tab:per_run}", r"\end{table}"]
    # one file per table, so each can be \input where it belongs in the text
    text = "\n".join(lines) + "\n"
    for chunk in text.split("\\begin{table}")[1:]:
        label = chunk.split("\\label{")[1].split("}")[0]
        with open(os.path.join(HERE, label.replace(":", "_") + ".tex"), "w") as f:
            f.write("\\begin{table}" + chunk.rstrip() + "\n")

    # ---------------- Figure 1: XY target vs measured ----------------
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.6))
    for ax, (pat, name) in zip(axes, [(1, "CIRCLE"), (2, "STAR"), (3, "FIGURE 8")]):
        for run in AFTER_RUNS:
            tr = traces[run].get(name)
            if tr is None or run == SHOW_RUN:
                continue
            s = tr["steady"] & tr["found"]
            ax.plot(tr["x"][s], tr["y"][s], color=C_OTHER, lw=0.5, alpha=0.35)
        tr = traces[SHOW_RUN][name]
        s = tr["steady"] & tr["found"]
        ax.plot(tr["x"][s], tr["y"][s], color=C_BALL, lw=0.9, label=f"measured ball (run {SHOW_RUN})")
        px, py = ideal_path(pat)
        ax.plot(px, py, color=C_TARGET, lw=1.8, ls="--", label="target (mathematical)")
        ax.plot([], [], color=C_OTHER, lw=0.8, alpha=0.6, label="measured, other runs")
        ax.set_aspect("equal")
        ax.set_xlim(-95, 95); ax.set_ylim(-95, 95)
        ax.set_title(name)
        ax.set_xlabel("x [mm]")
    axes[0].set_ylabel("y [mm]")
    axes[0].legend(loc="lower left", fontsize=7, framealpha=0.9)
    fig.suptitle("Target trajectories vs measured ball paths (steady state, last 15 s of each phase)", y=1.02)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"trajectories_xy.{ext}"))
    plt.close(fig)

    # ---------------- Figure 2: x(t), y(t) for each pattern ----------------
    fig, axes = plt.subplots(3, 2, figsize=(10, 7), sharex="row")
    for row, (pat, name, span) in enumerate([(1, "CIRCLE", 10), (2, "STAR", 10), (3, "FIGURE 8", 12)]):
        tr = traces[SHOW_RUN][name]
        t0 = tr["tt"][-1] - span
        w = tr["tt"] >= t0
        for col, (axis, key, tkey) in enumerate([("x", "x", "tx"), ("y", "y", "ty")]):
            ax = axes[row, col]
            ax.plot(tr["tt"][w] - t0, tr[tkey][w], color=C_TARGET, lw=1.5, ls="--", label="target")
            m = w & tr["found"]
            ax.plot(tr["tt"][m] - t0, tr[key][m], color=C_BALL, lw=1.0, label="measured")
            ax.set_ylabel(f"{axis} [mm]")
            ax.set_title(f"{name}: {axis}(t)", fontsize=9)
        axes[row, 0].legend(loc="upper right", fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"trajectories_time.{ext}"))
    plt.close(fig)

    # ---------------- Figure 3: error over a full cycle with settling ----------------
    d, t = traces[SHOW_RUN]["_full"]
    err = np.hypot(d["x"] - d["target_x"], d["y"] - d["target_y"])
    found = d["found"] == 1
    fig, ax = plt.subplots(figsize=(10, 3.4))
    ax.plot(t[found], err[found], color=C_BALL, lw=0.5, alpha=0.5, label="tracking error $e(t)$")
    rms_full = moving_rms(t, err, found)
    ax.plot(t, rms_full, color="k", lw=1.2, label="1 s moving RMS")
    colours = {"CENTER": "#2ca02c", "CIRCLE": "#9467bd", "STAR": "#8c564b", "FIGURE 8": "#e377c2"}
    for pattern, a, e in phases(d, t):
        name = PATTERNS[pattern]
        r = results[SHOW_RUN][name]
        ax.axvspan(t[a], t[e - 1], color=colours[name], alpha=0.08)
        ax.axvline(t[a], color="0.4", lw=0.8)
        ax.axvline(t[a] + r["settling_s"], color=colours[name], lw=1.2, ls=":")
        ax.text(t[a] + 0.4, 62, f"{name}\nsettled {r['settling_s']:.1f} s\nRMS {r['rms_mm']:.1f} mm",
                fontsize=7, va="top")
        ax.axvspan(t[e - 1] - STEADY_S, t[e - 1], ymin=0, ymax=0.03, color=colours[name], alpha=0.8)
    ax.set_xlim(t[0], t[-1]); ax.set_ylim(0, 65)
    ax.set_xlabel("time [s]"); ax.set_ylabel("error [mm]")
    ax.set_title(f"Tracking error over a full pattern cycle (run {SHOW_RUN}); dotted = settling time, "
                 "bar at the bottom = steady-state window")
    ax.legend(loc="upper right", fontsize=7)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"error_cycle.{ext}"))
    plt.close(fig)

    # ---------------- Figure 4: before / after on the circle ----------------
    fig, axes = plt.subplots(1, 2, figsize=(7.5, 3.6))
    px, py = ideal_path(1)
    for ax, run, col, title in [(axes[0], BEFORE_RUN, C_BEFORE, "Before (run 1)"),
                                (axes[1], SHOW_RUN, C_BALL, f"After (run {SHOW_RUN})")]:
        tr = traces[run]["CIRCLE"]
        s = tr["steady"] & tr["found"]
        ax.plot(tr["x"][s], tr["y"][s], color=col, lw=0.8, label="measured ball")
        ax.plot(px, py, color=C_TARGET, lw=1.8, ls="--", label="target")
        r = results[run]["CIRCLE"]
        ax.set_title(f"{title}: mean error {r['mean_mm']:.1f} mm", fontsize=9)
        ax.set_aspect("equal"); ax.set_xlim(-100, 100); ax.set_ylim(-100, 100)
        ax.set_xlabel("x [mm]")
    axes[0].set_ylabel("y [mm]")
    axes[0].legend(loc="lower left", fontsize=7)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"circle_before_after.{ext}"))
    plt.close(fig)

    # ---------------- Figure 5: CENTER steady state ----------------
    fig, axes = plt.subplots(1, 2, figsize=(8, 3.4), gridspec_kw={"width_ratios": [1, 1.4]})
    for run in AFTER_RUNS:
        tr = traces[run].get("CENTER")
        if tr is None:
            continue
        s = tr["steady"] & tr["found"]
        kw = dict(s=2, alpha=0.6) if run == SHOW_RUN else dict(s=1, alpha=0.15, color=C_OTHER)
        axes[0].scatter(tr["x"][s], tr["y"][s], **({"color": C_BALL} | kw))
    th = np.linspace(0, 2 * math.pi, 200)
    for rad in (5, 10):
        axes[0].plot(rad * np.cos(th), rad * np.sin(th), color=C_TARGET, lw=0.8, ls="--")
        axes[0].text(rad * 0.71, rad * 0.71, f"{rad} mm", fontsize=7, color=C_TARGET)
    axes[0].plot(0, 0, "+", color=C_TARGET, ms=10, mew=2)
    axes[0].set_aspect("equal"); axes[0].set_xlim(-15, 15); axes[0].set_ylim(-15, 15)
    axes[0].set_xlabel("x [mm]"); axes[0].set_ylabel("y [mm]")
    axes[0].set_title("CENTER: ball positions, steady state", fontsize=9)
    tr = traces[SHOW_RUN]["CENTER"]
    axes[1].plot(tr["tt"][tr["found"]], tr["err"][tr["found"]], color=C_BALL, lw=0.7)
    axes[1].axhline(tr["band"], color="k", lw=0.8, ls="--", label="settling band")
    axes[1].axvline(tr["settle"], color="#2ca02c", lw=1.2, ls=":", label=f"settled at {tr['settle']:.2f} s")
    axes[1].set_xlabel("time since switch to CENTER [s]"); axes[1].set_ylabel("error [mm]")
    axes[1].set_ylim(0, 40)
    axes[1].set_title(f"CENTER: error after the switch (run {SHOW_RUN})", fontsize=9)
    axes[1].legend(fontsize=7)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(FIG, f"center_steady.{ext}"))
    plt.close(fig)

    # ---------------- console summary ----------------
    print("per run:")
    for run in [BEFORE_RUN] + AFTER_RUNS:
        for name, r in results[run].items():
            extra = (f" lag {r['lag_s']:.2f}s rms_lagcomp {r['rms_lag_comp_mm']:.1f} shape {r['shape_dev_mm']:.1f} "
                     f"amp {r['amplitude_ratio']:.2f} target_dev {r['target_dev_mm']:.2f}") if "lag_s" in r else ""
            print(f"  run{run:<2} {name:8s} settle {r['settling_s']:5.2f}s mean {r['mean_mm']:5.1f} rms {r['rms_mm']:5.1f} "
                  f"p95 {r['p95_mm']:5.1f} max {r['max_mm']:5.1f} found {r['found_pct']:5.1f}%{extra}")
    print("aggregate (runs %s):" % AFTER_RUNS)
    for name, a in agg.items():
        print("  " + name, {k: (round(v[0], 2), round(v[1], 2)) if isinstance(v, tuple) else v for k, v in a.items()})


if __name__ == "__main__":
    main()
