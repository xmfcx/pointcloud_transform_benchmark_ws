#!/usr/bin/env bash
set -e

CORE=3
OUT=results.json

echo "Running Google Benchmark..."

taskset -c $CORE \
  ros2 run pointcloud_transform_benchmark benchmark \
    --benchmark_format=json \
    --benchmark_out="$OUT" \
    --benchmark_repetitions=50 \
    --benchmark_report_aggregates_only=false

echo "Benchmark finished"
echo "Results written to $OUT"
