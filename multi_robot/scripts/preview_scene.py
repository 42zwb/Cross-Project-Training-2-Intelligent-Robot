#!/usr/bin/env python3
"""Offline scene illustration. Ground truth never enters the runtime map."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import yaml

from fleet_bringup.scene import generate_scene

p = argparse.ArgumentParser()
p.add_argument("--seeds", type=int, nargs="+", default=[42, 2026, 12345])
p.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "src/fleet_bringup/config/fleet.yaml"))
p.add_argument("--output", required=True)
a = p.parse_args()
cfg = yaml.safe_load(Path(a.config).read_text())
fig, axes = plt.subplots(1, len(a.seeds), figsize=(5 * len(a.seeds), 5), squeeze=False)
records = []
colors = ["#56a8df", "#e5a440", "#9a7dd1"]
for ax, seed in zip(axes[0], a.seeds):
    geometry, record = generate_scene(seed, [r["start"] for r in cfg["robots"]], cfg["scene"]["obstacle_count"])
    records.append(record)
    for robot, color in zip(cfg["robots"], colors):
        x0, y0, x1, y1 = robot["region"]
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor=color, alpha=0.1))
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, fill=False, edgecolor=color, linestyle="--"))
        x, y, _ = robot["start"]
        ax.add_patch(Circle((x, y), 0.18, color=color))
        ax.text(x, y-0.55, robot["name"], ha="center", fontsize=8)
    for name, x, y, dx, dy in geometry:
        ax.add_patch(Rectangle((x-dx/2, y-dy/2), dx, dy, color="#425265"))
    ax.set(xlim=(-7.5,7.5), ylim=(-7.5,7.5), aspect="equal",
           xlabel="X [m]", ylabel="Y [m]", title=f"Seed {seed} | 15 x 15 m | {record['obstacle_count']} obstacles")
    ax.set_xticks(range(-7,8,2)); ax.set_yticks(range(-7,8,2)); ax.grid(alpha=0.15)
fig.suptitle("Seeded scene preview (offline ground truth; robots build their own lidar map)")
fig.tight_layout()
out = Path(a.output); out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, dpi=160)
out.with_suffix(".json").write_text(json.dumps(records, indent=2))
print(out)
