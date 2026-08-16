#ifndef FAST_CALIB__BOARD_POLYGON_SELECTION_TOOL_HPP_
#define FAST_CALIB__BOARD_POLYGON_SELECTION_TOOL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Ogre
{
class ManualObject;
class SceneNode;
class Viewport;
}

namespace rviz_rendering
{
class RenderWindow;
}

namespace fast_calib
{
class BoardPolygonSelectionTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  BoardPolygonSelectionTool();
  ~BoardPolygonSelectionTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
  int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;

private:
  struct ScreenPoint
  {
    int x;
    int y;
  };

  void initializeOverlay();
  void updateOverlay();
  void clearPolygon();
  void finishPolygon();
  void onPreviewCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message);
  void selectVisiblePoints(rviz_rendering::RenderWindow *window);
  bool pointInsidePolygon(double x, double y) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_publisher_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr preview_cloud_;
  std::string preview_frame_id_;
  std::vector<ScreenPoint> polygon_;
  bool drawing_;
  rviz_rendering::RenderWindow *render_window_;

  Ogre::ManualObject *overlay_;
  Ogre::SceneNode *overlay_node_;
  std::string material_name_;
};
}  // namespace fast_calib

#endif  // FAST_CALIB__BOARD_POLYGON_SELECTION_TOOL_HPP_
