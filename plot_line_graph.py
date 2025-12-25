import json
import matplotlib.pyplot as plt
import pandas as pd
import re

# -----------------------------
# Load data
# -----------------------------
with open("results.json", "r") as f:
    data = json.load(f)

benchmarks = data["benchmarks"]
records = []

for b in benchmarks:
    name = b["name"]

    # Skip aggregate entries
    if name.endswith(("_mean", "_median", "_stddev")):
        continue

    # Example name:
    # BM_PCL_Transform/500/repetition:0
    parts = name.split("/")
    method_full = parts[0]

    # Simplify method name
    if "BM_PCL_ROS_Transform" in method_full:
        method = "PCL_ROS"
    elif "BM_PCL_Transform" in method_full:
        method = "PCL"
    elif "BM_TF2_Transform" in method_full:
        method = "TF2"
    else:
        method = method_full

    # Extract repetition number
    rep_match = re.search(r"repetition:(\d+)", name)
    repetition = int(rep_match.group(1)) if rep_match else 0

    time_val = b["real_time"]

    records.append({
        "method": method,
        "repetition": repetition,
        "time": time_val
    })

df = pd.DataFrame(records)

# Add iteration index per (method, repetition)
df["iteration"] = df.groupby(["method", "repetition"]).cumcount()

# -----------------------------
# Plot
# -----------------------------
plt.figure(figsize=(8, 5))

colors = {
    "PCL": "#4C72B0",
    "TF2": "#DD8452",
    "PCL_ROS": "#55A868"
}

for method in df["method"].unique():
    method_df = df[df["method"] == method]

    for rep, rep_df in method_df.groupby("repetition"):
        plt.plot(
            rep_df["iteration"],
            rep_df["time"],
            color=colors.get(method, "gray"),
            alpha=1.00,
            linewidth=1.5,
            label=method if rep == method_df["repetition"].min() else None
        )

# -----------------------------
# Labels & grid
# -----------------------------
plt.xlabel("Iteration", fontsize=11)
plt.ylabel("Time (us)", fontsize=11)
plt.title("PointCloud Transform Benchmark\nPer-Iteration Runtime", fontsize=12)

plt.grid(True, linestyle="--", alpha=0.6)
plt.legend(title="Method")

# -----------------------------
# Layout & save
# -----------------------------
plt.tight_layout()
plt.savefig("benchmark_lineplot.png", dpi=150)
plt.show()
