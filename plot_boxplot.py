import json
import matplotlib.pyplot as plt
import pandas as pd

# -----------------------------
# Load data
# -----------------------------
with open("results.json", "r") as f:
    data = json.load(f)

benchmarks = data["benchmarks"]
records = []

for b in benchmarks:
    name = b["name"]
    # name format is usually "BM_PCL_Transform/500/repetition:0" or similar
    # We want to filter out aggregates like _mean, _median, _stddev if they exist
    if name.endswith("_mean") or name.endswith("_median") or name.endswith("_stddev"):
        continue

    # Extract method name (e.g., BM_PCL_Transform)
    # The name format is "BM_PCL_Transform/500/repetition:0"
    parts = name.split("/")
    method_full = parts[0]

    # Simplify method name for display
    if "BM_PCL_ROS_Transform" in method_full:
        method = "PCL_ROS"
    elif "BM_PCL_Transform" in method_full:
        method = "PCL"
    elif "BM_TF2_Transform" in method_full:
        method = "TF2"
    else:
        method = method_full

    # real_time is the wall clock time per iteration
    time_val = b["real_time"]

    records.append({
        "method": method,
        "time": time_val
    })

df = pd.DataFrame(records)

# -----------------------------
# Plot styling
# -----------------------------
plt.figure(figsize=(7, 5))

# Prepare data for boxplot
methods = ["PCL", "TF2", "PCL_ROS"]
# Filter methods that actually exist in the dataframe
methods = [m for m in methods if m in df["method"].unique()]

plot_data = [df[df["method"] == m]["time"].values for m in methods]

box = plt.boxplot(
    plot_data,
    labels=methods,
    patch_artist=True,
    showmeans=True,
    meanline=True,
    widths=0.5
)

# Colors
colors = ["#4C72B0", "#DD8452", "#55A868"]
# Ensure we have enough colors if more methods are added
if len(methods) > len(colors):
    # Fallback or extend colors
    colors = colors * (len(methods) // len(colors) + 1)

for patch, color in zip(box["boxes"], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

# Style lines
for element in ["whiskers", "caps", "medians", "means"]:
    for line in box[element]:
        line.set_linewidth(2)

# Mean line style
for mean in box["means"]:
    mean.set_color("black")
    mean.set_linestyle("--")

# -----------------------------
# Labels & grid
# -----------------------------
plt.ylabel("Time (us)", fontsize=11)
plt.title("PointCloud Transform Benchmark", fontsize=12)
plt.grid(axis="y", linestyle="--", alpha=0.6)

# -----------------------------
# Tight layout & save
# -----------------------------
plt.tight_layout()
plt.savefig("benchmark_boxplot.png", dpi=150)
plt.show()
