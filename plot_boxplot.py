import pandas as pd
import matplotlib.pyplot as plt

# -----------------------------
# Load data
# -----------------------------
df = pd.read_csv("results.csv")

# Ensure consistent ordering
methods = ["PCL", "TF2"]
data = [df[df["method"] == m]["time_ms"].values for m in methods]

# -----------------------------
# Plot styling
# -----------------------------
plt.figure(figsize=(7, 5))

box = plt.boxplot(
    data,
    labels=methods,
    patch_artist=True,
    showmeans=True,
    meanline=True,
    widths=0.5
)

# Colors
colors = ["#4C72B0", "#DD8452"]
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
plt.ylabel("Time (ms)", fontsize=11)
plt.title("PointCloud Transform Benchmark\n(CPU pinned, 5 runs)", fontsize=12)
plt.grid(axis="y", linestyle="--", alpha=0.6)

# -----------------------------
# Tight layout & save
# -----------------------------
plt.tight_layout()
plt.savefig("benchmark_boxplot.png", dpi=150)
plt.show()
