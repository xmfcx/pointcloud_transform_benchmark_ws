#include <chrono>
#include <random>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using Clock = std::chrono::steady_clock;

/* ------------------------------------------------------------
 * Utility: Random generators
 * ------------------------------------------------------------ */
float randomFloat(float min, float max) {
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

geometry_msgs::msg::TransformStamped randomTransform() {
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.translation.x = randomFloat(-50.f, 50.f);
    tf.transform.translation.y = randomFloat(-50.f, 50.f);
    tf.transform.translation.z = randomFloat(-10.f, 10.f);

    tf2::Quaternion q;
    q.setRPY(
        randomFloat(-M_PI, M_PI),
        randomFloat(-M_PI, M_PI),
        randomFloat(-M_PI, M_PI));
    q.normalize();

    tf.transform.rotation = tf2::toMsg(q);
    return tf;
}

/* ------------------------------------------------------------
 * Generate random PointCloud2
 * ------------------------------------------------------------ */
sensor_msgs::msg::PointCloud2 createRandomCloud(std::size_t num_points) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;
    cloud.width = num_points;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (std::size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        // Random direction + radius
        float theta = randomFloat(0.f, 2.f * M_PI);
        float phi = randomFloat(0.f, M_PI);
        float r = randomFloat(0.f, 300.f);

        *iter_x = r * std::sin(phi) * std::cos(theta);
        *iter_y = r * std::sin(phi) * std::sin(theta);
        *iter_z = r * std::cos(phi);
    }

    return cloud;
}

/* ------------------------------------------------------------
 * Benchmark: PCL transform
 * ------------------------------------------------------------ */
void benchmarkPCL(
    const std::vector<sensor_msgs::msg::PointCloud2> &clouds,
    const std::vector<geometry_msgs::msg::TransformStamped> &transforms) {
    auto start = Clock::now();

    for (const auto &cloud: clouds) {
        pcl::PointCloud<pcl::PointXYZ> pcl_in;
        pcl::PointCloud<pcl::PointXYZ> pcl_out;
        pcl::fromROSMsg(cloud, pcl_in);

        for (const auto &tf: transforms) {
            Eigen::Matrix4f mat =
                    tf2::transformToEigen(tf.transform).matrix().cast<float>();

            pcl::transformPointCloud(pcl_in, pcl_out, mat);

            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(pcl_out, out);
        }
    }

    auto end = Clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "RESULT,PCL," << ms << std::endl;
}

/* ------------------------------------------------------------
 * Benchmark: tf2::doTransform
 * ------------------------------------------------------------ */
void benchmarkTF2(
    const std::vector<sensor_msgs::msg::PointCloud2> &clouds,
    const std::vector<geometry_msgs::msg::TransformStamped> &transforms) {
    auto start = Clock::now();

    for (const auto &cloud: clouds) {
        for (const auto &tf: transforms) {
            sensor_msgs::msg::PointCloud2 out;
            tf2::doTransform(cloud, out, tf);
        }
    }

    auto end = Clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "RESULT,TF2," << ms << std::endl;
}

/* ------------------------------------------------------------
 * Main
 * ------------------------------------------------------------ */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::size_t num_clouds = 1000;
    std::size_t num_transforms = 100;

    if (argc >= 2) {
        num_clouds = static_cast<std::size_t>(std::stoul(argv[1]));
    }
    if (argc >= 3) {
        num_transforms = static_cast<std::size_t>(std::stoul(argv[2]));
    }

    std::vector<sensor_msgs::msg::PointCloud2> clouds;
    clouds.reserve(num_clouds);

    for (std::size_t i = 0; i < num_clouds; ++i) {
        std::size_t size = static_cast<std::size_t>(randomFloat(500.f, 50000.f));
        clouds.emplace_back(createRandomCloud(size));
    }

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(num_transforms);
    for (std::size_t i = 0; i < num_transforms; ++i) {
        transforms.emplace_back(randomTransform());
    }

    std::cout << "Benchmarking "
              << num_clouds << " clouds × "
              << num_transforms << " transforms\n";

    benchmarkPCL(clouds, transforms);
    benchmarkTF2(clouds, transforms);

    rclcpp::shutdown();
    return 0;
}
