#!/usr/bin/env python3
"""Standalone scientific figures from saved observations and evaluator truth."""
import argparse, json, math
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
import numpy as np
from fleet_mapping.grid import load_map

p = argparse.ArgumentParser()
p.add_argument("run_dir")
p.add_argument("--output", required=True)
a = p.parse_args()
run = Path(a.run_dir)
out = Path(a.output)
out.mkdir(parents=True, exist_ok=True)
load = lambda name: [
    json.loads(line) for line in (run / name).read_text().splitlines() if line.strip()
]
events = load("events.jsonl")
truth = [t for t in load("truth.jsonl") if len(t["robots"]) == 3]
telemetry = load("telemetry.jsonl")
data, meta = load_map(run / "results/final/shared_map.yaml")
colors = ["#1685d2", "#dc901e", "#8962c4"]
names = ["robot1", "robot2", "robot3"]
start = next(e["sim_time"] for e in events if e["event"] == "explore_start")
end = next(e["sim_time"] for e in events if e["event"] == "formation_request")
series = [t for t in truth if start <= t["sim_time"] <= end]
image = np.where(data < 0, 0, np.where(data == 0, 1, 2))
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(
    image,
    origin="lower",
    extent=[-6, 6, -6, 6],
    cmap=ListedColormap(["#8694a5", "#f5f7fa", "#17212e"]),
    vmin=0,
    vmax=2,
    interpolation="nearest",
)
for n, c in zip(names, colors):
    xy = np.array([t["robots"][n][:2] for t in series])
    ax.plot(xy[:, 0], xy[:, 1], color=c, lw=1.8, label=n + " actual exploration")
    ax.plot(xy[0, 0], xy[0, 1], "o", color=c, ms=6)
for x in [-2, 2]:
    ax.axvline(x, color="#68809c", ls="--", lw=0.9)
ax.set(
    xlim=(-6, 6),
    ylim=(-6, 6),
    xlabel="map X [m]",
    ylabel="map Y [m]",
    title="Shared lidar map and measured exploration paths",
)
ax.set_xticks(range(-6, 7, 2))
ax.set_yticks(range(-6, 7, 2))
ax.grid(alpha=0.18)
ax.legend(loc="lower center", fontsize=8)
fig.tight_layout()
for ext in ["png", "svg"]:
    fig.savefig(out / ("exploration." + ext), dpi=180)
plt.close(fig)
formations = [e for e in events if e["event"] == "formation_ready"][:3]
fig, axes = plt.subplots(1, 3, figsize=(12, 4))
labels = ["Triangle", "Row", "Column"]
for ax, e, label in zip(axes, formations, labels):
    actual = min(truth, key=lambda t: abs(t["sim_time"] - e["sim_time"]))
    estimated = min(telemetry, key=lambda t: abs(t["sim_time"] - e["sim_time"]))
    maxerr = 0
    for i, (n, c) in enumerate(zip(names, colors)):
        target = e["slots"][e["assignment"][i]]
        gt = actual["robots"][n]
        est = estimated["robots"][n]["pose"]
        maxerr = max(maxerr, math.dist(gt[:2], target[:2]))
        ax.add_patch(
            Circle(target[:2], 0.15, fill=False, color="#7fa49b", ls=":", lw=1)
        )
        ax.plot(*target[:2], "x", color="#17212e", ms=7)
        ax.plot(*gt[:2], "o", color=c, label=n, ms=7)
        ax.plot([target[0], gt[0]], [target[1], gt[1]], color=c, lw=1)
        ax.arrow(
            gt[0],
            gt[1],
            0.20 * math.cos(gt[2]),
            0.20 * math.sin(gt[2]),
            color=c,
            width=0.014,
            head_width=0.07,
        )
        ax.plot(*est[:2], "+", color=c, ms=8)
    ax.set(
        xlim=(-1.7, 1.7),
        ylim=(-1.7, 1.7),
        xlabel="X [m]",
        ylabel="Y [m]",
        title=f"{label}\nmax actual error: {maxerr:.3f} m",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
axes[0].legend(fontsize=8, loc="upper right")
fig.suptitle("Three verified formations: x target, circle actual, + SLAM estimate")
fig.tight_layout()
for ext in ["png", "svg"]:
    fig.savefig(out / ("formations." + ext), dpi=180)
print("Saved figures to " + str(out))
