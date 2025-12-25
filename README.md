# PointCloud Transformation Benchmark

## Replicate the results

```bash
sudo apt update
sudo apt install -y python3-matplotlib python3-pandas

source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --symlink-install --event-handlers=console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# set CORE to one of your cpu cores, I set to 3, benchmark will only use that specific core
# benchmark_repetitions is set to 50
./benchmark.sh
python3 plot_boxplot.py
python3 plot_line_graph.py

# Plots will be available in the root directory:
# benchmark_boxplot.png and benchmark_lineplot.png
```

All the bench code is in [benchmark.cpp](src/pointcloud_transform_benchmark/src/benchmark.cpp).

## Bench setup

**Inputs:**
- 200.000 points (like vls128) point cloud, random points in `sensor_msgs::msg::PointCloud2` format.
- Random transformation matrix in `geometry_msgs::msg::TransformStamped` format.

**Task:**
- Transform and return in the input format

**Competing methods:**
1. **PCL:** Converts from `sensor_msgs` to `pcl`, transform, converts back to `sensor_msgs`.
2. **TF2:** [Directly transform without unnecessary middle steps.](https://github.com/ros2/geometry2/blob/78ef2e48c31f3a38d67e8198e28cb9596d58c2df/tf2_sensor_msgs/include/tf2_sensor_msgs/tf2_sensor_msgs.hpp#L77-L150)
3. **PCL_ROS:** [Directly transforms the point cloud](https://github.com/ros-perception/perception_pcl/blob/67a5c2ba4c4de3ca21c5cd495812a01ced3fb69a/pcl_ros/src/transforms.cpp#L127-L227) too but does many other things too, ends up being 2 times slower in the benchmark.
   - This is the proposed method in [qiita link](https://qiita.com/yakato_jun/items/091f36308b0662110537).

## Results

**50 runs:**

![benchmark_boxplot.png](assets/benchmark_boxplot.png)
![benchmark_lineplot.png](assets/benchmark_lineplot.png)

As expected, if you want to transform a `sensor_msgs::msg::PointCloud2` with a `geometry_msgs::msg::TransformStamped`, it is faster to do it directly.

Converting to `pcl::PointCloud<pcl::PointXYZ>`, transforming, converting back to `sensor_msgs::msg::PointCloud2` will obviously be more costly.

Of course [`pcl::transformPointCloud(const Eigen::Matrix4f & transform, const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out)`](https://github.com/ros-perception/perception_pcl/blob/67a5c2ba4c4de3ca21c5cd495812a01ced3fb69a/pcl_ros/src/transforms.cpp#L127-L227) is an option too. But it is still slower than direct tf2 way.

I don't know what was going on <https://qiita.com/yakato_jun/items/091f36308b0662110537> here but `tf2::doTransform()` is considerably faster than `pcl::transformPointCloud()`.
