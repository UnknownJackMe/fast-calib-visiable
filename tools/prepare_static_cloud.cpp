#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
struct Options
{
  std::string bag;
  std::string topic = "/livox/lidar";
  std::string output_full;
  std::string output_preview;
  double full_leaf = 0.005;
  double preview_leaf = 0.03;
  double min_range = 0.3;
  double max_range = 8.0;
  double min_x = -std::numeric_limits<double>::infinity();
  double max_x = std::numeric_limits<double>::infinity();
  double min_y = -std::numeric_limits<double>::infinity();
  double max_y = std::numeric_limits<double>::infinity();
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
};

void printUsage(const char *program)
{
  std::cerr
      << "用法: " << program << " --bag BAG_DIR --output-full FILE --output-preview FILE [选项]\n"
      << "选项:\n"
      << "  --topic TOPIC             LiDAR PointCloud2 topic，默认 /livox/lidar\n"
      << "  --full-leaf METERS        高分辨率体素大小，默认 0.005\n"
      << "  --preview-leaf METERS     RViz 预览体素大小，默认 0.03\n"
      << "  --min-range METERS        最小量程，默认 0.3\n"
      << "  --max-range METERS        最大量程，默认 8.0\n"
      << "  --min-x/--max-x METERS    可选的宽松 X 范围\n"
      << "  --min-y/--max-y METERS    可选的宽松 Y 范围\n"
      << "  --min-z/--max-z METERS    可选的宽松 Z 范围\n";
}

Options parseOptions(int argc, char **argv)
{
  Options options;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto requireValue = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc)
      {
        throw std::runtime_error(name + " 缺少参数值");
      }
      return argv[++i];
    };

    if (arg == "--bag")
      options.bag = requireValue(arg);
    else if (arg == "--topic")
      options.topic = requireValue(arg);
    else if (arg == "--output-full")
      options.output_full = requireValue(arg);
    else if (arg == "--output-preview")
      options.output_preview = requireValue(arg);
    else if (arg == "--full-leaf")
      options.full_leaf = std::stod(requireValue(arg));
    else if (arg == "--preview-leaf")
      options.preview_leaf = std::stod(requireValue(arg));
    else if (arg == "--min-range")
      options.min_range = std::stod(requireValue(arg));
    else if (arg == "--max-range")
      options.max_range = std::stod(requireValue(arg));
    else if (arg == "--min-x")
      options.min_x = std::stod(requireValue(arg));
    else if (arg == "--max-x")
      options.max_x = std::stod(requireValue(arg));
    else if (arg == "--min-y")
      options.min_y = std::stod(requireValue(arg));
    else if (arg == "--max-y")
      options.max_y = std::stod(requireValue(arg));
    else if (arg == "--min-z")
      options.min_z = std::stod(requireValue(arg));
    else if (arg == "--max-z")
      options.max_z = std::stod(requireValue(arg));
    else if (arg == "--help" || arg == "-h")
    {
      printUsage(argv[0]);
      std::exit(0);
    }
    else
      throw std::runtime_error("未知参数: " + arg);
  }

  if (options.bag.empty() || options.output_full.empty() || options.output_preview.empty())
    throw std::runtime_error("必须提供 --bag、--output-full 和 --output-preview");
  if (!(options.full_leaf > 0.0) || !(options.preview_leaf >= options.full_leaf))
    throw std::runtime_error("体素大小无效，preview-leaf 必须大于等于 full-leaf");
  if (!(options.min_range >= 0.0) || !(options.max_range > options.min_range))
    throw std::runtime_error("量程范围无效");
  if (!(options.max_x > options.min_x) || !(options.max_y > options.min_y) ||
      !(options.max_z > options.min_z))
    throw std::runtime_error("宽松 XYZ 范围无效");
  return options;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, float leaf)
{
  auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  return output;
}

void ensureParent(const std::string &path)
{
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const Options options = parseOptions(argc, argv);
    auto accumulated = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    rosbag2_cpp::Reader reader;
    reader.open(options.bag);
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

    std::size_t frame_count = 0;
    std::size_t raw_point_count = 0;
    const double min_range_sq = options.min_range * options.min_range;
    const double max_range_sq = options.max_range * options.max_range;

    std::cout << "正在读取 LiDAR bag: " << options.bag << std::endl;
    std::cout << "点云 topic: " << options.topic << std::endl;

    while (reader.has_next())
    {
      auto bag_message = reader.read_next();
      if (bag_message->topic_name != options.topic)
        continue;

      auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
      serialization.deserialize_message(&serialized, ros_msg.get());

      pcl::PointCloud<pcl::PointXYZ> frame;
      pcl::fromROSMsg(*ros_msg, frame);
      raw_point_count += frame.size();
      ++frame_count;

      for (const auto &point : frame)
      {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
          continue;
        const double range_sq =
            static_cast<double>(point.x) * point.x +
            static_cast<double>(point.y) * point.y +
            static_cast<double>(point.z) * point.z;
        if (range_sq < min_range_sq || range_sq > max_range_sq)
          continue;
        if (point.x < options.min_x || point.x > options.max_x ||
            point.y < options.min_y || point.y > options.max_y ||
            point.z < options.min_z || point.z > options.max_z)
          continue;
        accumulated->push_back(point);
      }
    }

    if (frame_count == 0 || accumulated->empty())
      throw std::runtime_error("bag 中没有读取到指定 PointCloud2 topic 的有效点");

    accumulated->width = static_cast<std::uint32_t>(accumulated->size());
    accumulated->height = 1;
    accumulated->is_dense = true;

    std::cout << "累计帧数: " << frame_count << std::endl;
    std::cout << "原始点数: " << raw_point_count << std::endl;
    std::cout << "量程过滤后点数: " << accumulated->size() << std::endl;
    std::cout << "正在生成高分辨率静态点云..." << std::endl;

    auto full = voxelize(accumulated, static_cast<float>(options.full_leaf));
    accumulated.reset();
    std::cout << "高分辨率点数: " << full->size() << std::endl;

    std::cout << "正在生成 RViz 低分辨率预览点云..." << std::endl;
    auto preview = voxelize(full, static_cast<float>(options.preview_leaf));
    std::cout << "预览点数: " << preview->size() << std::endl;

    ensureParent(options.output_full);
    ensureParent(options.output_preview);
    if (pcl::io::savePLYFileASCII(options.output_full, *full) < 0)
      throw std::runtime_error("保存高分辨率点云失败: " + options.output_full);
    if (pcl::io::savePLYFileASCII(options.output_preview, *preview) < 0)
      throw std::runtime_error("保存预览点云失败: " + options.output_preview);

    std::cout << "高分辨率静态点云: " << options.output_full << std::endl;
    std::cout << "RViz 预览点云: " << options.output_preview << std::endl;
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "生成静态点云失败: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
