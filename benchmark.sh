#!/usr/bin/env bash
set -e

CORE=3
RUNS=50
OUT=results.csv

NUM_CLOUDS=100
NUM_TRANSFORMS=10

echo "run,method,time_ms" > "$OUT"

for i in $(seq 1 $RUNS); do
  taskset -c $CORE \
    ros2 run pointcloud_transform_benchmark benchmark \
      $NUM_CLOUDS $NUM_TRANSFORMS | \
    awk -v run="$i" -F"," '/^RESULT/ { print run "," $2 "," $3 }' >> "$OUT"
done

echo "Benchmark finished"
echo "Results written to $OUT"
