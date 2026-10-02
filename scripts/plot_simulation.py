"""Render the recorded Field2D poses from SimulationFunctionalTest (requires matplotlib)."""

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
report = root / "build/reports/simulation"
with (report / "field-poses.csv").open() as file:
    rows = list(csv.DictReader(file))

names = ["Depot Cycle", "Depot Inside", "Safe", "Trench Depot Points"]
colors = {"blue": "#2563eb", "red": "#dc2626"}
fig, axes = plt.subplots(3, 2, figsize=(13, 13), constrained_layout=True)
fig.suptitle("Recorded Field2D poses · expected vs actual", fontsize=20, fontweight="bold")

def xy(case, prefix):
    return ([float(row[prefix + "_x"]) for row in case],
            [float(row[prefix + "_y"]) for row in case])

def field(ax):
    ax.set(xlim=(-0.3, 16.85), ylim=(-0.3, 8.4), xlabel="Field X (m)", ylabel="Field Y (m)")
    ax.set_aspect("equal")
    ax.plot([0, 16.54, 16.54, 0, 0], [0, 0, 8.07, 8.07, 0], color="#9ca3af", lw=1)
    ax.axvline(8.27, color="#d1d5db", lw=0.8)
    ax.grid(alpha=0.15)

for name, ax in zip(names, axes.flat):
    peaks, finals = [], []
    for alliance in colors:
        case = [row for row in rows if row["scenario"] == name and row["alliance"] == alliance]
        ax.plot(*xy(case, "expected"), "--", color="#374151", lw=2,
                label="Expected" if alliance == "blue" else None)
        ax.plot(*xy(case, "actual"), color=colors[alliance], lw=1.6, label=alliance.title())
        ax.scatter(*[value[-1] for value in xy(case, "actual")], color=colors[alliance], s=25)
        peaks.append(max(float(row["error_m"]) for row in case))
        finals.append(float(case[-1]["error_m"]))
    ax.set_title(f"{name}\nPeak {max(peaks)*100:.1f} cm · endpoint {max(finals)*100:.1f} cm")
    field(ax)
    ax.legend(fontsize=8, loc="upper right")

ax = axes.flat[4]
for alliance in colors:
    case = [row for row in rows if row["scenario"] == "Teleop" and row["alliance"] == alliance]
    ax.plot(*xy(case, "actual"), color=colors[alliance], label=alliance.title())
ax.set_title("Teleop · forward, strafe, rotate, stop, disable")
field(ax)
ax.legend(fontsize=8)

ax = axes.flat[5]
for name in names:
    case = [row for row in rows if row["scenario"] == name and row["alliance"] == "blue"]
    ax.plot([float(row["time"]) for row in case],
            [float(row["error_m"])*100 for row in case], label=name)
ax.axhline(35, color="#9ca3af", ls="--", label="35 cm transient limit")
ax.set(title="Position error through each checked drive phase",
       xlabel="Trajectory time (s)", ylabel="Position error (cm)")
ax.grid(alpha=0.15)
ax.legend(fontsize=8)

fig.savefig(report / "field-poses.png", dpi=150)
print(report / "field-poses.png")
