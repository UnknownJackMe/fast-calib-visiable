#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <regex>
#include <string>
#include <vector>

#include "common_lib.h"
#include "../src/data_preprocess.hpp"
#include "../src/qr_detect.hpp"

std::vector<pcl::PointXYZ> defaultManualCenters()
{
  return {
      pcl::PointXYZ(3.57454f, 0.07096f, 0.11873f),
      pcl::PointXYZ(3.55390f, -0.43180f, 0.14098f),
      pcl::PointXYZ(3.54801f, 0.06100f, -0.27500f),
      pcl::PointXYZ(3.52758f, -0.45020f, -0.25500f),
  };
}

std::vector<pcl::PointXYZ> loadManualCentersFromYaml(const std::string &path)
{
  if (path.empty())
  {
    return defaultManualCenters();
  }

  std::ifstream input(path);
  if (!input.is_open())
  {
    throw std::runtime_error("Failed to open manual LiDAR centers YAML: " + path);
  }

  std::regex coord_re(R"(^\s*([xyz]):\s*([-+0-9.eE]+)\s*$)");
  std::smatch match;
  std::string line;
  bool have_x = false;
  bool have_y = false;
  bool have_z = false;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::vector<pcl::PointXYZ> centers;

  while (std::getline(input, line))
  {
    if (!std::regex_match(line, match, coord_re))
    {
      continue;
    }

    const auto key = match[1].str();
    const auto value = std::stof(match[2].str());
    if (key == "x")
    {
      x = value;
      have_x = true;
    }
    else if (key == "y")
    {
      y = value;
      have_y = true;
    }
    else if (key == "z")
    {
      z = value;
      have_z = true;
    }

    if (have_x && have_y && have_z)
    {
      centers.push_back(pcl::PointXYZ(x, y, z));
      have_x = have_y = have_z = false;
    }
  }

  if (centers.size() != 4)
  {
    throw std::runtime_error("Expected 4 manual LiDAR centers in " + path + ", got " + std::to_string(centers.size()));
  }

  return centers;
}

bool isValidatedRefinedCentersYaml(const std::string &path)
{
  if (path.empty())
  {
    return false;
  }
  std::ifstream input(path);
  if (!input.is_open())
  {
    return false;
  }
  bool refined_kind = false;
  bool passed = false;
  std::string line;
  // Only accept the top-level contract fields. Per-hole entries also contain
  // a status key, so allowing indentation here could accidentally accept a
  // top-level failed refinement when one individual hole passed.
  std::regex kind_re(R"(^kind:\s*refined_lidar_hole_centers\s*$)");
  std::regex status_re(R"(^status:\s*pass\s*$)");
  while (std::getline(input, line))
  {
    refined_kind = refined_kind || std::regex_match(line, kind_re);
    passed = passed || std::regex_match(line, status_re);
  }
  return refined_kind && passed;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fast_calib");

  Params params = loadParameters(node);
  node->declare_parameter("manual_lidar_centers_path", std::string(""));
  node->declare_parameter("allow_unvalidated_manual_centers", false);
  std::string manual_lidar_centers_path;
  bool allow_unvalidated_manual_centers = false;
  node->get_parameter_or("manual_lidar_centers_path", manual_lidar_centers_path, std::string(""));
  node->get_parameter_or("allow_unvalidated_manual_centers", allow_unvalidated_manual_centers, false);

  if (!allow_unvalidated_manual_centers &&
      !isValidatedRefinedCentersYaml(manual_lidar_centers_path))
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Refusing unvalidated LiDAR centers. Expected kind=refined_lidar_hole_centers "
        "and status=pass in %s. Set allow_unvalidated_manual_centers:=true only for "
        "explicit legacy replay.",
        manual_lidar_centers_path.c_str());
    rclcpp::shutdown();
    return 5;
  }

  DataPreprocessPtr dataPreprocessPtr;
  dataPreprocessPtr.reset(new DataPreprocess(params));

  if (dataPreprocessPtr->img_input_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Input image is empty");
    rclcpp::shutdown();
    return 2;
  }

  QRDetectPtr qrDetectPtr;
  qrDetectPtr.reset(new QRDetect(node, params));

  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  qr_center_cloud->reserve(4);
  qrDetectPtr->detect_qr(dataPreprocessPtr->img_input_, qr_center_cloud);
  if (qr_center_cloud->size() != 4)
  {
    RCLCPP_ERROR(node->get_logger(), "Expected 4 QR/camera centers, got %zu", qr_center_cloud->size());
    cv::imwrite(params.output_path + "/qr_detect_manual_lidar.png", qrDetectPtr->imageCopy_);
    rclcpp::shutdown();
    return 3;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  lidar_center_cloud->reserve(4);
  try
  {
    const auto manual_centers = loadManualCentersFromYaml(manual_lidar_centers_path);
    for (const auto &center : manual_centers)
    {
      lidar_center_cloud->push_back(center);
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 4;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  sortPatternCenters(qr_center_cloud, qr_centers, "camera");
  sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

  RCLCPP_INFO(node->get_logger(), "Sorted camera centers:");
  for (const auto &p : qr_centers->points)
  {
    std::cout << "  " << p.x << " " << p.y << " " << p.z << std::endl;
  }
  RCLCPP_INFO(node->get_logger(), "Sorted refined LiDAR centers:");
  for (const auto &p : lidar_centers->points)
  {
    std::cout << "  " << p.x << " " << p.y << " " << p.z << std::endl;
  }

  Eigen::Matrix4f transformation;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
  svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  aligned_lidar_centers->reserve(lidar_centers->size());
  alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

  double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
  RCLCPP_INFO(node->get_logger(), "[Refined LiDAR centers] RMSE: %.6f m", rmse);
  RCLCPP_INFO(node->get_logger(), "[Refined LiDAR centers] T_cam_lidar:");
  std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  projectPointCloudToImage(dataPreprocessPtr->cloud_input_, transformation, qrDetectPtr->cameraMatrix_,
                           qrDetectPtr->distCoeffs_, dataPreprocessPtr->img_input_, colored_cloud);
  saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

  rclcpp::shutdown();
  return 0;
}
