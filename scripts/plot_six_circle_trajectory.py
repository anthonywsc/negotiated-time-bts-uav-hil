import argparse
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


N_UAV = 6


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, name):
    return [float(r[name]) for r in rows]


def save_fig(fig, path, bbox_inches=None, pad_inches=0.1):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches=bbox_inches, pad_inches=pad_inches)
    plt.close(fig)
    print(f"Saved plot: {path}")


def output_path(base_out, suffix):
    root, ext = os.path.splitext(base_out)
    if not ext:
        ext = ".png"
    return f"{root}_{suffix}{ext}"


def stage_start_time(rows, stage_names):
    for r in rows:
        if r["stage"] in stage_names:
            return float(r["t"])
    return float(rows[0]["t"])


def nearest_index(values, target):
    return min(range(len(values)), key=lambda i: abs(values[i] - target))


def set_3d_equal(ax, xs, ys, zs, pad=3.0):
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    cx, cy, cz = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0
    span = max(xmax - xmin, ymax - ymin, zmax - zmin, 1.0) / 2.0 + pad
    ax.set_xlim(cx - span, cx + span)
    ax.set_ylim(cy - span, cy + span)
    ax.set_zlim(cz - span, cz + span)


def closure_ring_points(pos, t_index):
    pts = []
    for i in range(1, N_UAV + 1):
        pts.append((pos[i][0][t_index], pos[i][1][t_index], -pos[i][2][t_index]))
    cx_i = sum(p[0] for p in pts) / float(N_UAV)
    cy_i = sum(p[1] for p in pts) / float(N_UAV)
    ordered = sorted(pts, key=lambda p: math.atan2(p[1] - cy_i, p[0] - cx_i))
    ordered.append(ordered[0])
    return ordered


def main():
    parser = argparse.ArgumentParser(description="Plot six-UAV BTS trajectory, attitude, and time-negotiation curves.")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rows_all = read_rows(args.csv)
    if not rows_all:
        raise SystemExit(f"No rows in {args.csv}")

    stages_all = [r["stage"] for r in rows_all]
    t_time0 = stage_start_time(rows_all, ("time_consensus", "bts", "hold"))
    t_bts0 = stage_start_time(rows_all, ("bts", "hold"))

    rows = [r for r in rows_all if r["stage"] in ("bts", "hold")]
    if not rows:
        rows = rows_all
        t_bts0 = float(rows[0]["t"])

    stages = [r["stage"] for r in rows]
    t = [float(r["t"]) - t_bts0 for r in rows]
    spread = col(rows, "spread")
    err = col(rows, "formation_error")
    cx, cy, cz = col(rows, "center_x"), col(rows, "center_y"), col(rows, "center_z")
    negotiated_t = float(rows[-1].get("negotiated_T", "nan"))

    pos = {}
    qpos = {}
    attitude_available = "roll1" in rows[0]
    att = {}
    for i in range(1, N_UAV + 1):
        pos[i] = (
            col(rows, f"p{i}_x"),
            col(rows, f"p{i}_y"),
            col(rows, f"p{i}_z"),
        )
        offx = col(rows, f"offset{i}_x")
        offy = col(rows, f"offset{i}_y")
        qpos[i] = (
            [pos[i][0][k] - offx[k] for k in range(len(rows))],
            [pos[i][1][k] - offy[k] for k in range(len(rows))],
            pos[i][2],
        )
        if attitude_available:
            att[i] = (
                [math.degrees(x) for x in col(rows, f"roll{i}")],
                [math.degrees(x) for x in col(rows, f"pitch{i}")],
                [math.degrees(x) for x in col(rows, f"yaw{i}")],
            )

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    for i in range(1, N_UAV + 1):
        x, y, z = pos[i]
        ax.plot(x, y, [-zz for zz in z], label=f"UAV{i}", linewidth=1.7)
        ax.scatter([x[-1]], [y[-1]], [-z[-1]], marker="x", s=45)
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Altitude / m")
    ax.legend(fontsize=8)
    save_fig(fig, output_path(args.out, "3d"))

    if "hold" in stages:
        form_t = t[stages.index("hold")]
    else:
        form_t = t[nearest_index(err, min(err))]
    ring_times = []
    rt = 0.0
    while rt < form_t - 1e-9:
        ring_times.append(rt)
        rt += 3.0
    ring_times.append(form_t)
    ring_times = [rt for rt in ring_times if t[0] <= rt <= t[-1]]

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    for i in range(1, N_UAV + 1):
        x, y, _ = pos[i]
        ax.plot(x, y, label=f"UAV{i}", linewidth=1.7)
        ax.scatter([x[-1]], [y[-1]], marker="x", s=40)
    if ring_times:
        cmap = plt.get_cmap("viridis")
        for n, rt in enumerate(ring_times):
            k = nearest_index(t, rt)
            ordered = closure_ring_points(pos, k)
            ring_x = [p[0] for p in ordered]
            ring_y = [p[1] for p in ordered]
            color = cmap(n / max(len(ring_times) - 1, 1))
            ax.plot(ring_x, ring_y, color=color, linewidth=1.2, alpha=0.8)
            ax.scatter(ring_x[:-1], ring_y[:-1], color=color, s=14, zorder=3)
            ax.text(ring_x[0], ring_y[0], f"{rt:.1f}s", color=color, fontsize=7)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    save_fig(fig, output_path(args.out, "xy"))

    if ring_times:
        lo = max(t[0], min(ring_times))
        hi = min(t[-1], form_t)
        idx = [k for k, tk in enumerate(t) if lo <= tk <= hi]
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(1, 1, 1, projection="3d")
        all_x, all_y, all_z = [], [], []
        for i in range(1, N_UAV + 1):
            x, y, z = pos[i]
            xs = [x[k] for k in idx]
            ys = [y[k] for k in idx]
            zs = [-z[k] for k in idx]
            all_x += xs
            all_y += ys
            all_z += zs
            ax.plot(xs, ys, zs, linewidth=1.1, alpha=0.6, label=f"UAV{i}")

        cmap = plt.get_cmap("viridis")
        for n, rt in enumerate(ring_times):
            k = nearest_index(t, rt)
            ordered = closure_ring_points(pos, k)
            ring_x = [p[0] for p in ordered]
            ring_y = [p[1] for p in ordered]
            ring_z = [p[2] for p in ordered]
            color = cmap(n / max(len(ring_times) - 1, 1))
            ax.plot(ring_x, ring_y, ring_z, color=color, linewidth=1.8, alpha=0.9)
            ax.scatter(ring_x[:-1], ring_y[:-1], ring_z[:-1], color=color, s=18)
            ax.text(ring_x[0], ring_y[0], ring_z[0], f"{rt:.1f}s", color=color, fontsize=8)
            all_x += ring_x
            all_y += ring_y
            all_z += ring_z

        ax.set_xlabel("X / m")
        ax.set_ylabel("Y / m")
        ax.set_zlabel("Altitude / m")
        ax.legend(
            fontsize=7,
            ncol=2,
            loc="upper right",
            bbox_to_anchor=(0.88, 0.88),
            bbox_transform=ax.transAxes,
            borderaxespad=0.0,
        )
        set_3d_equal(ax, all_x, all_y, all_z)
    save_fig(fig, output_path(args.out, "closure_rings"), bbox_inches="tight", pad_inches=0.02)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(t, err, label="formation error", linewidth=1.7)
    ax.plot(t, spread, label="spread", linewidth=1.2, alpha=0.7)
    if "hold" in stages:
        ax.axvline(t[stages.index("hold")], color="k", linestyle="--", linewidth=1.0, label="hold start")
    ax.set_xlabel("Time from BTS start / s")
    ax.set_ylabel("Distance / m")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_fig(fig, output_path(args.out, "error"))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ("X / m", "Y / m", "Altitude / m")
    for i in range(1, N_UAV + 1):
        x, y, z = pos[i]
        axes[0].plot(t, x, linewidth=1.0, label=f"UAV{i}")
        axes[1].plot(t, y, linewidth=1.0, label=f"UAV{i}")
        axes[2].plot(t, [-zz for zz in z], linewidth=1.0, label=f"UAV{i}")
    for idx_ax, (ax, label) in enumerate(zip(axes, labels)):
        if math.isfinite(negotiated_t):
            ax.axvline(
                negotiated_t,
                color="red",
                linestyle="--",
                linewidth=1.2,
                label=f"T*={negotiated_t:.2f}s" if idx_ax == 0 else None,
            )
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=6)
    axes[2].set_xlabel("Time from BTS start / s")
    save_fig(fig, output_path(args.out, "xyz"))

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    for i in range(1, N_UAV + 1):
        x, y, z = qpos[i]
        ax.plot(x, y, [-zz for zz in z], label=f"q{i}", linewidth=1.7)
        ax.scatter([x[-1]], [y[-1]], [-z[-1]], marker="x", s=45)
    ax.set_xlabel("qX / m")
    ax.set_ylabel("qY / m")
    ax.set_zlabel("Altitude / m")
    ax.legend(fontsize=8)
    save_fig(fig, output_path(args.out, "state3d"))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ("qX / m", "qY / m", "Altitude / m")
    for i in range(1, N_UAV + 1):
        x, y, z = qpos[i]
        axes[0].plot(t, x, linewidth=1.0, label=f"q{i}")
        axes[1].plot(t, y, linewidth=1.0, label=f"q{i}")
        axes[2].plot(t, [-zz for zz in z], linewidth=1.0, label=f"q{i}")
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=6)
    axes[2].set_xlabel("Time from BTS start / s")
    save_fig(fig, output_path(args.out, "state_xyz"))

    time_cols_available = all(f"T{i}" in rows_all[0] for i in range(1, N_UAV + 1))
    if time_cols_available:
        rows_t = [r for r in rows_all if r["stage"] in ("time_consensus", "bts", "hold")]
        if not rows_t:
            rows_t = rows_all
        tt = [float(r["t"]) - t_time0 for r in rows_t]
        fig, ax = plt.subplots(figsize=(8, 5))
        for i in range(1, N_UAV + 1):
            ti = col(rows_t, f"T{i}")
            ax.plot(tt, ti, linewidth=1.6, label=f"T{i}(t)")
        ax.set_xlabel("Time from BTS start / s")
        ax.set_ylabel("Time state / s")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", ncol=4, fontsize=8)
        save_fig(fig, output_path(args.out, "time_consensus"))
    else:
        print("No T1-T6 columns found in CSV. Run the updated controller to generate time-consensus plots.")

    if attitude_available:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i in range(1, N_UAV + 1):
            roll, pitch, yaw = att[i]
            axes[0].plot(t, roll, linewidth=1.0, label=f"UAV{i}")
            axes[1].plot(t, pitch, linewidth=1.0, label=f"UAV{i}")
            axes[2].plot(t, yaw, linewidth=1.0, label=f"UAV{i}")
        axes[0].set_ylabel("Roll / deg")
        axes[1].set_ylabel("Pitch / deg")
        axes[2].set_ylabel("Yaw / deg")
        axes[2].set_xlabel("Time from BTS start / s")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8, ncol=6)
        save_fig(fig, output_path(args.out, "attitude"))
    else:
        print("No attitude columns found in CSV. Run the updated controller to generate attitude plots.")

    print(f"Final formation error: {err[-1]:.3f} m")
    print(f"Final spread: {spread[-1]:.3f} m")
    if "hold" in stages:
        print(f"Hold stage reached at: {t[stages.index('hold')]:.3f} s after BTS start")


if __name__ == "__main__":
    main()
