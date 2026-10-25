#!/usr/bin/env python3
"""Analysis of the automated test protocol (pi_code/ball_tracking/5_test_runner.py).

Input: data/test_*.csv.gz, one row per camera frame with the test phase.
Output: metrics.json, tab_*.tex and figures/*.pdf|png for the report.

Ball positions are taken at the frame's exposure time (t_exposure), the target
is interpolated to that same instant, so the camera and processing delay
(~25 ms) does not show up as lag.

Screw contacts: a contact shows as a sudden change of the ball's velocity near a
bracket (the screws are on the brackets, on the diagonals of the camera frame).
Normal control changes the velocity smoothly. Contact = velocity change over
2 x 5 frames > CONTACT_DV, at >= CONTACT_R from the centre, within CONTACT_DEG of
a bracket direction; frames closer than 0.2 s are one contact.
"""
import csv
import glob
import gzip
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
DATA = sorted(glob.glob(os.path.join(HERE, "data", "test_*.csv.gz")))[-1]

STEP_MM = 30.0
BAND_MM = 2.0                 # settling band of the step response, absolute
CENTER_SKIP_S = 5.0           # CENTER: ignore the first seconds (ball placed / rest)
PATTERN_STEADY_S = (20.0, 59.5)
CONTACT_DV = 100.0            # mm/s
CONTACT_R = 65.0              # mm
CONTACT_DEG = 15.0
CONTACT_EXCLUDE_S = 0.3       # +- around a contact, for the "without contacts" metrics
BRACKETS_DEG = (-135.0, -45.0, 45.0, 135.0)
# marker positions in the plate frame (mm), plate reference at working height
BRACKETS_MM = ((-74.6, -72.7), (73.2, -71.5), (73.2, 73.1), (-71.8, 71.0))
PATTERNS = {"CIRCLE": 1, "STAR": 2, "FIGURE 8": 3}
TAP_RECOVER_MM = 5.0

plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "savefig.bbox": "tight", "figure.dpi": 150,
                     "font.family": "serif", "font.serif": ["cmr10"], "mathtext.fontset": "cm",
                     "axes.formatter.use_mathtext": True, "axes.unicode_minus": False})
C_TARGET, C_BALL, C_OTHER, C_CONTACT = "#d62728", "#1f77b4", "#7f7f7f", "#ff7f0e"


# ---------------------------------------------------------------------------
def load():
    with gzip.open(DATA, "rt") as f:
        rows = list(csv.DictReader(f))
    num = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    d = {k: num(k) for k in ("t", "t_exposure", "found", "x", "y", "z", "target_x", "target_y", "height_deg")}
    d["run"] = np.array([int(r["run"]) for r in rows])
    d["phase"] = np.array([r["phase"] for r in rows])
    d["event"] = np.array([r["event"] for r in rows])
    d["found"] = np.nan_to_num(d["found"]) == 1
    te = d["t_exposure"].copy()
    bad = ~np.isfinite(te)
    te[bad] = d["t"][bad] - 0.025
    d["te"] = te
    # target at the exposure instant
    ok = np.isfinite(d["target_x"])
    d["tx"] = np.interp(te, d["t"][ok], d["target_x"][ok])
    d["ty"] = np.interp(te, d["t"][ok], d["target_y"][ok])
    return d


def segments(d):
    """(run, phase, start, end) for every phase occurrence; a phase repeated after a
    ball loss keeps only the attempt that ran to the end (the last one)."""
    key = np.char.add(d["run"].astype(str), np.char.add("|", d["phase"]))
    cut = np.flatnonzero(key[1:] != key[:-1]) + 1
    bounds = np.r_[0, cut, len(key)]
    out = {}
    for a, e in zip(bounds[:-1], bounds[1:]):
        run, ph = int(d["run"][a]), str(d["phase"][a])
        if ph in ("WAIT_BALL", "SETTLE", "REST"):
            continue
        out[(run, ph)] = (a, e)          # later attempts overwrite aborted ones
    return [(r, p, a, e) for (r, p), (a, e) in out.items()]


def ideal_path(pattern, n=2000):
    if pattern == 1:
        a = np.linspace(0, 2 * math.pi, n)
        return 50 * np.cos(a), 50 * np.sin(a)
    if pattern == 2:
        idx = np.arange(11) % 10
        ang = math.pi / 2 + idx * math.pi / 5
        r = np.where(idx % 2 == 0, 60.0, 25.0)
        return r * np.cos(ang), r * np.sin(ang)
    a = np.linspace(0, 2 * math.pi, n)
    return 60 * np.sin(a), 30 * np.sin(2 * a)


def distance_to_path(x, y, px, py):
    ax, ay, bx, by = px[:-1], py[:-1], px[1:], py[1:]
    dx, dy = bx - ax, by - ay
    L2 = np.where(dx * dx + dy * dy > 0, dx * dx + dy * dy, 1)
    out = np.empty(len(x))
    for i, (qx, qy) in enumerate(zip(x, y)):
        u = np.clip(((qx - ax) * dx + (qy - ay) * dy) / L2, 0, 1)
        out[i] = np.min(np.hypot(ax + u * dx - qx, ay + u * dy - qy))
    return out


def best_lag(tt, x, y, tx, ty, max_lag_s=1.0):
    """Shift (s) of the ball path vs the target minimising the mean squared distance;
    positive: ball behind the target."""
    n, dt = len(x), float((tt[-1] - tt[0]) / (len(tt) - 1))
    best = (0, np.inf)
    for k in range(-int(max_lag_s / dt), int(max_lag_s / dt) + 1):
        if k >= 0:
            ex, ey = x[k:] - tx[:n - k], y[k:] - ty[:n - k]
        else:
            ex, ey = x[:n + k] - tx[-k:], y[:n + k] - ty[-k:]
        m = np.mean(ex * ex + ey * ey)
        if m < best[1]:
            best = (k, m)
    return best[0] * dt


def moving_rms(tt, err, window=1.0):
    c = np.cumsum(np.r_[0, err ** 2])
    j = np.searchsorted(tt, tt - window)
    i = np.arange(len(tt)) + 1
    return np.sqrt((c[i] - c[j]) / np.maximum(i - j, 1))


# ---------------------------------------------------------------------------
def contacts(d):
    """Indices and positions of screw contacts."""
    te, x, y, f = d["te"], d["x"], d["y"], d["found"]
    n = len(te)
    dv = np.full(n, np.nan)
    for i in range(5, n - 5):
        if not f[i - 5:i + 5].all() or te[i + 5] - te[i - 5] > 0.25:
            continue
        sl_b, sl_a = slice(i - 5, i), slice(i, i + 5)
        vb = [np.polyfit(te[sl_b] - te[i], v[sl_b], 1)[0] for v in (x, y)]
        va = [np.polyfit(te[sl_a] - te[i], v[sl_a], 1)[0] for v in (x, y)]
        dv[i] = math.hypot(va[0] - vb[0], va[1] - vb[1])
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))
    diag = np.min([np.abs(((ang - a + 180) % 360) - 180) for a in BRACKETS_DEG], axis=0)
    hit = (np.nan_to_num(dv) > CONTACT_DV) & (r >= CONTACT_R) & (diag <= CONTACT_DEG) & (d["phase"] != "DISTURB")
    events = []
    for i in np.flatnonzero(hit):
        if events and te[i] - te[events[-1][-1]] < 0.2:
            events[-1].append(i)
        else:
            events.append([i])
    peaks = [max(ev, key=lambda i: dv[i]) for ev in events]
    near = [int(np.argmin([abs(((ang[i] - a + 180) % 360) - 180) for a in BRACKETS_DEG])) for i in peaks]
    return dict(dv=dv, index=np.array(peaks, int), bracket=np.array(near, int))


def excluded_mask(d, contact_idx, a, e):
    m = np.ones(e - a, bool)
    te = d["te"][a:e]
    for i in contact_idx:
        if a <= i < e:
            m &= np.abs(te - d["te"][i]) > CONTACT_EXCLUDE_S
    return m


# ---------------------------------------------------------------------------
def analyse_center(d, a, e):
    tt = d["te"][a:e] - d["te"][a]
    s = (tt >= CENTER_SKIP_S) & d["found"][a:e]
    ex, ey = d["x"][a:e] - d["tx"][a:e], d["y"][a:e] - d["ty"][a:e]
    err = np.hypot(ex, ey)[s]
    k = 15
    xs = np.convolve(d["x"][a:e], np.ones(k) / k, mode="same")
    ys = np.convolve(d["y"][a:e], np.ones(k) / k, mode="same")
    noise = np.hypot(d["x"][a:e] - xs, d["y"][a:e] - ys)[s][k:-k]
    return dict(mean=float(err.mean()), rms=float(np.sqrt(np.mean(err ** 2))),
                p95=float(np.percentile(err, 95)), max=float(err.max()),
                within5=float(np.mean(err < 5) * 100), noise=float(np.sqrt(np.mean(noise ** 2))))


def analyse_steps(d, a, e, axis):
    """Full +-STEP_MM steps of one STEP phase."""
    tt = d["te"][a:e]
    tgt = (d["tx"] if axis == "x" else d["ty"])[a:e]
    pos = (d["x"] if axis == "x" else d["y"])[a:e]
    other = (d["y"] if axis == "x" else d["x"])[a:e]
    f = d["found"][a:e]
    out = []
    sign_change = np.flatnonzero(np.sign(tgt[1:]) != np.sign(tgt[:-1])) + 1
    for c in sign_change:
        new = STEP_MM * np.sign(tgt[c])
        prev = -new
        # target leaves the previous level / reaches the new one
        i0 = c
        while i0 > 0 and abs(tgt[i0 - 1] - prev) > 0.5:
            i0 -= 1
        if abs(tgt[i0 - 1] - prev) > 0.5 if i0 > 0 else True:
            continue                     # not a full step (e.g. the first one from 0)
        t0 = tt[i0]
        w = (tt >= t0) & (tt < t0 + 7.9) & f
        if w.sum() < 300:
            continue
        T = tt[w] - t0
        y = (pos[w] - prev) / (new - prev)           # 0 -> 1
        yt = (tgt[w] - prev) / (new - prev)

        def first(curve, level):
            k = np.flatnonzero(curve >= level)
            return float(T[k[0]]) if len(k) else np.nan
        err_mm = np.abs(pos[w] - new)
        outside = np.flatnonzero(err_mm > BAND_MM)
        outside5 = np.flatnonzero(err_mm > 0.05 * 2 * STEP_MM)
        out.append(dict(
            t0=float(t0), dir=int(np.sign(new)), T=T, y=y, yt=yt,
            rise=first(y, 0.9) - first(y, 0.1),
            delay=first(y, 0.5) - first(yt, 0.5),
            overshoot=float(max(0.0, (y.max() - 1)) * 100),
            overshoot_mm=float(max(0.0, (y.max() - 1)) * 2 * STEP_MM),
            settle2=float(T[outside[-1]]) if len(outside) else 0.0,
            settle5=float(T[outside5[-1]]) if len(outside5) else 0.0,
            steady=float(np.mean(err_mm[T > T[-1] - 2.0])),
            cross_rms=float(np.sqrt(np.mean(other[w] ** 2)))))
    return out


def analyse_pattern(d, a, e, pattern, cidx):
    tt = d["te"][a:e] - d["te"][a]
    f = d["found"][a:e]
    x, y, tx, ty = d["x"][a:e], d["y"][a:e], d["tx"][a:e], d["ty"][a:e]
    err = np.hypot(x - tx, y - ty)
    steady = (tt >= PATTERN_STEADY_S[0]) & (tt <= PATTERN_STEADY_S[1]) & f
    rms1 = moving_rms(tt, err)
    band = 1.1 * np.nanmax(rms1[steady])
    above = np.flatnonzero((rms1 > band) & (tt < PATTERN_STEADY_S[0]))
    keep = excluded_mask(d, cidx, a, e)
    px, py = ideal_path(pattern)
    s = steady
    s_nc = steady & keep
    tdev = distance_to_path(tx[s][::4], ty[s][::4], px, py)
    res = dict(
        settling=float(tt[above[-1]]) if len(above) else 0.0,
        mean=float(err[s].mean()), rms=float(np.sqrt(np.mean(err[s] ** 2))),
        p95=float(np.percentile(err[s], 95)), max=float(err[s].max()),
        mean_nc=float(err[s_nc].mean()), p95_nc=float(np.percentile(err[s_nc], 95)),
        lag=float(best_lag(tt[s], x[s], y[s], tx[s], ty[s])),
        amplitude=float(np.mean(np.hypot(x[s], y[s])) / np.mean(np.hypot(tx[s], ty[s]))),
        shape=float(np.mean(distance_to_path(x[s][::3], y[s][::3], px, py))),
        guard_pct=float(np.mean(tdev > 3.0) * 100),       # target pulled off its path: edge guard
        contacts=int(np.sum((cidx >= a) & (cidx < e))),
        found=float(f.mean() * 100))
    return res, dict(tt=tt, x=x, y=y, tx=tx, ty=ty, err=err, steady=steady, keep=keep, rms1=rms1, band=band)


def analyse_taps(d, a, e):
    out = []
    te, f = d["te"], d["found"]
    for i in [k for k in range(a, e) if d["event"][k].startswith("tap")]:
        w = np.flatnonzero((te >= te[i] - 0.3) & (te <= te[i] + 7.5) & f)
        if len(w) < 200:
            continue
        T = te[w] - te[i]
        err = np.hypot(d["x"][w] - d["tx"][w], d["y"][w] - d["ty"][w])
        base = np.median(err[T < 0])
        hit = np.flatnonzero((T > -0.3) & (T < 3.0) & (err > max(8.0, base + 6.0)))
        if not len(hit):
            out.append(dict(detected=False))
            continue
        t_hit = T[hit[0]]
        k_peak = hit[0] + int(np.argmax(err[hit[0]:hit[0] + 90]))
        after = T > T[k_peak]
        # recovered: from here on the error stays below TAP_RECOVER_MM for 1 s
        rec = np.nan
        idx = np.flatnonzero(after)
        for j in idx:
            win = (T >= T[j]) & (T <= T[j] + 1.0)
            if np.all(err[win] < TAP_RECOVER_MM):
                rec = float(T[j] - t_hit)
                break
        out.append(dict(detected=True, peak=float(err[k_peak]), t_peak=float(T[k_peak] - t_hit),
                        recovery=rec, T=T - t_hit, err=err))
    return out


# ---------------------------------------------------------------------------
def pm(vals, fmt="{:.1f}"):
    v = np.array([x for x in vals if np.isfinite(x)], float)
    if not len(v):
        return "--"
    return f"${fmt.format(v.mean())} \\pm {fmt.format(v.std())}$"


def table(fn, header, rows, cols, caption, label):
    with open(os.path.join(HERE, fn), "w") as f:
        f.write("\\begin{table}[H]\n\\centering\n")
        f.write(f"\\begin{{tabularx}}{{\\textwidth}}{{|{cols}|}}\n\\hline\n")
        f.write(" & ".join(f"\\textbf{{{h}}}" for h in header) + " \\\\ \\hline\n")
        for r in rows:
            f.write(" & ".join(r) + " \\\\ \\hline\n")
        f.write(f"\\end{{tabularx}}\n\\caption{{\\textit{{{caption}}}}}\n\\label{{{label}}}\n\\end{{table}}\n")


def main():
    os.makedirs(FIG, exist_ok=True)
    d = load()
    segs = segments(d)
    con = contacts(d)
    cidx = con["index"]
    M = {"source": os.path.basename(DATA), "frames": int(len(d["t"])),
         "duration_min": float((d["t"][-1] - d["t"][0]) / 60),
         "fps": float(len(d["t"]) / (d["t"][-1] - d["t"][0])),
         "latency_ms": float(np.nanmedian(d["t"] - d["t_exposure"]) * 1000),
         "ball_lost": int(np.sum(d["event"] == "ball_lost"))}
    center, steps, pats, taps, traces = [], {"x": [], "y": []}, {p: [] for p in PATTERNS}, [], {}
    for run, ph, a, e in segs:
        if ph == "CENTER":
            center.append(dict(run=run, **analyse_center(d, a, e)))
        elif ph in ("STEP_X", "STEP_Y"):
            ax = ph[-1].lower()
            steps[ax] += [dict(run=run, **s) for s in analyse_steps(d, a, e, ax)]
        elif ph in PATTERNS:
            r, tr = analyse_pattern(d, a, e, PATTERNS[ph], cidx)
            pats[ph].append(dict(run=run, **r))
            traces[(run, ph)] = tr
        elif ph == "DISTURB":
            taps += [dict(run=run, **t) for t in analyse_taps(d, a, e)]

    # ball loss: where was the ball just before
    losses = []
    for i in np.flatnonzero(d["event"] == "ball_lost"):
        j = i - 1
        while j > 0 and not (d["found"][j] and np.isfinite(d["x"][j])):
            j -= 1
        k = j
        while k > 0 and d["found"][k] and d["te"][j] - d["te"][k] < 1.0:
            k -= 1
        rr = np.hypot(d["x"][k:j + 1], d["y"][k:j + 1])
        kk = k + int(np.argmax(rr))
        recent = cidx[(d["te"][cidx] > d["te"][j] - 2.0) & (d["te"][cidx] <= d["te"][j])]
        losses.append(dict(run=int(d["run"][i]), phase=str(d["phase"][j]),
                           max_r=float(rr.max()), angle=float(math.degrees(math.atan2(d["y"][kk], d["x"][kk]))),
                           contacts_2s_before=int(len(recent))))

    # ---- metrics.json
    strip = lambda L, drop: [{k: v for k, v in x.items() if k not in drop} for x in L]
    M["center"] = center
    M["steps"] = {ax: strip(v, ("T", "y", "yt")) for ax, v in steps.items()}
    M["patterns"] = pats
    M["taps"] = strip(taps, ("T", "err"))
    M["contacts"] = dict(total=int(len(cidx)),
                         per_bracket=[int(np.sum(con["bracket"] == b)) for b in range(4)],
                         per_phase={ph: int(np.sum(d["phase"][cidx] == ph)) for ph in set(d["phase"][cidx])})
    M["losses"] = losses
    with open(os.path.join(HERE, "metrics.json"), "w") as f:
        json.dump(M, f, indent=1, default=float)

    # ---- tables
    table("tab_test_center.tex", ["Runs", "Mean [mm]", "RMS [mm]", "P95 [mm]", "Max [mm]", "Within 5 mm", "Noise [mm]"],
          [[str(len(center)), pm([c["mean"] for c in center]), pm([c["rms"] for c in center]),
            pm([c["p95"] for c in center]), pm([c["max"] for c in center]),
            pm([c["within5"] for c in center], "{:.0f}") + "\\,\\%", pm([c["noise"] for c in center], "{:.2f}")]],
          "c|X|X|X|X|X|X", "CENTER hold after the ball was placed at rest (60\\,s per run, first 5\\,s excluded).", "tab:test_center")
    rows = []
    for ax in ("x", "y"):
        S = steps[ax]
        rows.append([f"STEP\\_{ax.upper()}", str(len(S)), pm([s["rise"] for s in S], "{:.2f}"), pm([s["delay"] for s in S], "{:.2f}"),
                     pm([s["overshoot"] for s in S], "{:.0f}") + "\\,\\%", pm([s["settle2"] for s in S], "{:.2f}"),
                     pm([s["settle5"] for s in S], "{:.2f}"), pm([s["steady"] for s in S]), pm([s["cross_rms"] for s in S])])
    table("tab_test_steps.tex", ["Axis", "Steps", "Rise 10--90\\% [s]", "Delay [s]", "Overshoot", "Settling $\\pm$2\\,mm [s]",
                                 "Settling $\\pm$5\\% [s]", "Remaining error [mm]", "Cross-axis RMS [mm]"],
          rows, "l|c|X|X|X|X|X|X|X",
          f"Step response to {2*STEP_MM:.0f}\\,mm target steps (rate-limited to 120\\,mm/s, i.e.\\ a 0.5\\,s ramp); times from the start of the target ramp, delay between the 50\\,\\% points of target and ball.",
          "tab:test_steps")
    rows = []
    for ph, P in pats.items():
        rows.append([ph, str(len(P)), pm([p["settling"] for p in P]), pm([p["mean"] for p in P]), pm([p["p95"] for p in P]),
                     pm([p["mean_nc"] for p in P]), pm([p["lag"] for p in P], "{:.2f}"), pm([p["amplitude"] for p in P], "{:.2f}"),
                     pm([p["shape"] for p in P]), pm([p["contacts"] for p in P], "{:.1f}")])
    table("tab_test_patterns.tex", ["Pattern", "Runs", "Settling [s]", "Mean [mm]", "P95 [mm]", "Mean w/o contacts [mm]",
                                    "Lag [s]", "Amplitude ratio", "Shape dev. [mm]", "Screw contacts"],
          rows, "l|c|X|X|X|X|X|X|X|X",
          "Trajectory tracking, each pattern started from the ball at rest in CENTER; steady state 20--60\\,s; lag from the exposure time of each frame (camera delay removed).",
          "tab:test_patterns")
    det = [t for t in taps if t.get("detected")]
    table("tab_test_disturb.tex", ["Taps", "Detected", "Peak deviation [mm]", "Time to peak [s]", f"Recovery to {TAP_RECOVER_MM:.0f}\\,mm [s]"],
          [[str(len(taps)), str(len(det)), pm([t["peak"] for t in det]), pm([t["t_peak"] for t in det], "{:.2f}"),
            pm([t["recovery"] for t in det], "{:.2f}")]],
          "c|c|X|X|X", "Disturbance rejection: a light sideways tap on the ball while held in CENTER (5 per run).", "tab:test_disturb")

    # ---- figures
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.4), sharey=True)
    for ax, axis in zip(axs, ("x", "y")):
        S = steps[axis]
        grid = np.linspace(0, 7.5, 451)
        Y = np.array([np.interp(grid, s["T"], s["y"]) for s in S])
        for yv in Y:
            ax.plot(grid, yv, color=C_OTHER, lw=0.4, alpha=0.4)
        ax.plot(grid, np.median(Y, axis=0), color=C_BALL, lw=1.4, label="ball, median of all steps")
        ax.plot(grid, np.median([np.interp(grid, s["T"], s["yt"]) for s in S], axis=0), color=C_TARGET, ls="--", lw=1.2, label="target")
        b = BAND_MM / (2 * STEP_MM)
        ax.axhspan(1 - b, 1 + b, color="#2ca02c", alpha=0.12, label="$\\pm$2 mm band")
        ax.set_title(f"STEP {axis.upper()}: {len(S)} steps of {2*STEP_MM:.0f} mm")
        ax.set_xlabel("time from the start of the step [s]")
        ax.set_xlim(0, 7.5)
    axs[0].set_ylabel("normalised position")
    axs[0].legend(loc="lower right", fontsize=7)
    fig.savefig(os.path.join(FIG, "test_steps.pdf")); fig.savefig(os.path.join(FIG, "test_steps.png")); plt.close(fig)

    fig, axs = plt.subplots(1, 3, figsize=(10, 3.6))
    for ax, ph in zip(axs, PATTERNS):
        px, py = ideal_path(PATTERNS[ph])
        for (run, p), tr in traces.items():
            if p != ph:
                continue
            s = tr["steady"]
            ax.plot(tr["x"][s], tr["y"][s], color=C_BALL if run == 1 else C_OTHER, lw=0.6 if run == 1 else 0.4,
                    alpha=0.9 if run == 1 else 0.35)
        ax.plot(px, py, color=C_TARGET, ls="--", lw=1.6)
        sel = cidx[d["phase"][cidx] == ph]
        ax.scatter(d["x"][sel], d["y"][sel], s=10, color=C_CONTACT, zorder=5, label="screw contact")
        for bx, by in BRACKETS_MM:
            ax.plot(bx, by, "s", color="k", ms=6)
        ax.set_aspect("equal"); ax.set_xlim(-110, 110); ax.set_ylim(-110, 110)
        ax.set_title(ph); ax.set_xlabel("x [mm]")
    axs[0].set_ylabel("y [mm]")
    axs[0].plot([], [], color=C_BALL, label="run 1"); axs[0].plot([], [], color=C_OTHER, label="runs 2-5")
    axs[0].plot([], [], color=C_TARGET, ls="--", label="target"); axs[0].plot([], [], "s", color="k", label="bracket marker")
    axs[0].legend(loc="lower left", fontsize=6)
    fig.savefig(os.path.join(FIG, "test_patterns.pdf")); fig.savefig(os.path.join(FIG, "test_patterns.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.6, 4.4))
    f = d["found"] & np.isfinite(d["x"])
    ax.hexbin(d["x"][f], d["y"][f], gridsize=60, bins="log", cmap="Greys", extent=(-110, 110, -110, 110))
    ax.scatter(d["x"][cidx], d["y"][cidx], s=9, color=C_CONTACT, label=f"screw contacts ({len(cidx)})")
    for k, (bx, by) in enumerate(BRACKETS_MM):
        ax.plot(bx, by, "s", color="k", ms=7)
        ax.annotate(str(int(np.sum(con["bracket"] == k))), (bx, by), xytext=(6, 6), textcoords="offset points")
    ax.set_aspect("equal"); ax.set_xlim(-110, 110); ax.set_ylim(-110, 110)
    ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.legend(loc="lower left", fontsize=7)
    ax.set_title("Ball positions (grey) and screw contacts")
    fig.savefig(os.path.join(FIG, "test_contacts.pdf")); fig.savefig(os.path.join(FIG, "test_contacts.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.2))
    for t in det:
        ax.plot(t["T"], t["err"], color=C_OTHER, lw=0.5, alpha=0.5)
    grid = np.linspace(-0.3, 7.0, 400)
    ax.plot(grid, np.median([np.interp(grid, t["T"], t["err"]) for t in det], axis=0), color=C_BALL, lw=1.4, label="median")
    ax.axhline(TAP_RECOVER_MM, color="#2ca02c", ls=":", lw=1, label=f"{TAP_RECOVER_MM:.0f} mm")
    ax.set_xlabel("time from the tap [s]"); ax.set_ylabel("distance from the centre [mm]")
    ax.set_title(f"Disturbance rejection: {len(det)} taps"); ax.legend(fontsize=7)
    fig.savefig(os.path.join(FIG, "test_disturb.pdf")); fig.savefig(os.path.join(FIG, "test_disturb.png")); plt.close(fig)

    # ---- console summary
    print(json.dumps({k: M[k] for k in ("frames", "duration_min", "fps", "latency_ms", "ball_lost", "contacts", "losses")}, indent=1, default=float))


if __name__ == "__main__":
    main()
