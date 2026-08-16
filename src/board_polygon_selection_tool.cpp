// Lightweight FAST-Calib board polygon Tool.
// Interaction/projection design reference: RMonica/rviz2_cloud_annotation
// (BSD-3-Clause). No annotation, label propagation, or custom view controller
// code is vendored; see docs/THIRD_PARTY_NOTICES.md.

#include "fast_calib/board_polygon_selection_tool.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRenderQueue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreViewport.h>

#include <QKeyEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace fast_calib
{
BoardPolygonSelectionTool::BoardPolygonSelectionTool()
    : preview_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      drawing_(false),
      render_window_(nullptr),
      overlay_(nullptr),
      overlay_node_(nullptr)
{
  shortcut_key_ = 'p';
  access_all_keys_ = true;
}

BoardPolygonSelectionTool::~BoardPolygonSelectionTool()
{
  if (scene_manager_ && overlay_node_)
  {
    overlay_node_->detachAllObjects();
    scene_manager_->destroySceneNode(overlay_node_);
  }
  if (scene_manager_ && overlay_)
    scene_manager_->destroyManualObject(overlay_);
  if (!material_name_.empty())
    Ogre::MaterialManager::getSingleton().remove(material_name_);
}

void BoardPolygonSelectionTool::onInitialize()
{
  setName(QString::fromUtf8("标定板多边形选择"));
  setDescription(QString::fromUtf8(
      "按住左键拖动套索，松开左键完成；不需要点中具体点云。Esc 清除。"));
  setCursor(Qt::CrossCursor);

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  cloud_subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/static_cloud_preview", qos,
      std::bind(&BoardPolygonSelectionTool::onPreviewCloud, this, std::placeholders::_1));
  selected_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/board_polygon_selected_points", qos);
  initializeOverlay();

  RCLCPP_INFO(
      node_->get_logger(),
      "标定板套索工具已加载：按住左键拖动，松开完成；不需要点中具体点云。" );
}

void BoardPolygonSelectionTool::activate()
{
  setCursor(Qt::CrossCursor);
  if (overlay_node_)
    overlay_node_->setVisible(drawing_);
}

void BoardPolygonSelectionTool::deactivate()
{
  clearPolygon();
}

void BoardPolygonSelectionTool::initializeOverlay()
{
  std::ostringstream name;
  name << "FastCalibBoardPolygon_" << std::hex << reinterpret_cast<std::uintptr_t>(this);
  material_name_ = name.str() + "_material";

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  overlay_ = scene_manager_->createManualObject(name.str() + "_line");
  overlay_->setDynamic(true);
  overlay_->setUseIdentityProjection(true);
  overlay_->setUseIdentityView(true);
  overlay_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  Ogre::AxisAlignedBox infinite_box;
  infinite_box.setInfinite();
  overlay_->setBoundingBox(infinite_box);

  overlay_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  overlay_node_->attachObject(overlay_);
  overlay_node_->setVisible(false);
}

void BoardPolygonSelectionTool::onPreviewCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*message, *cloud);
  preview_cloud_ = cloud;
  preview_frame_id_ = message->header.frame_id;
  RCLCPP_INFO(node_->get_logger(), "已加载 RViz 预览点云，共 %zu 个点。", cloud->size());
}

void BoardPolygonSelectionTool::updateOverlay()
{
  if (!overlay_ || !overlay_node_ || !render_window_ || polygon_.empty())
    return;

  Ogre::Viewport *viewport =
      rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window_);
  const double width = std::max<std::uint32_t>(1, viewport->getActualWidth());
  const double height = std::max<std::uint32_t>(1, viewport->getActualHeight());

  overlay_->clear();
  overlay_->begin(material_name_, Ogre::RenderOperation::OT_LINE_STRIP);
  for (const auto &point : polygon_)
  {
    const double x = 2.0 * static_cast<double>(point.x) / width - 1.0;
    const double y = 1.0 - 2.0 * static_cast<double>(point.y) / height;
    overlay_->position(x, y, -1.0);
    overlay_->colour(1.0f, 0.85f, 0.05f, 1.0f);
  }
  if (polygon_.size() >= 3 && !drawing_)
  {
    const auto &point = polygon_.front();
    overlay_->position(
        2.0 * static_cast<double>(point.x) / width - 1.0,
        1.0 - 2.0 * static_cast<double>(point.y) / height,
        -1.0);
    overlay_->colour(1.0f, 0.85f, 0.05f, 1.0f);
  }
  overlay_->end();
  overlay_node_->setVisible(true);
}

void BoardPolygonSelectionTool::clearPolygon()
{
  polygon_.clear();
  drawing_ = false;
  render_window_ = nullptr;
  if (overlay_)
    overlay_->clear();
  if (overlay_node_)
    overlay_node_->setVisible(false);
}

int BoardPolygonSelectionTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
{
  if (event.leftDown())
  {
    polygon_.clear();
    drawing_ = true;
    render_window_ = event.panel->getRenderWindow();
    polygon_.push_back({event.x, event.y});
    updateOverlay();
    return Render;
  }

  if (drawing_)
  {
    if (event.rightDown())
    {
      clearPolygon();
      RCLCPP_INFO(node_->get_logger(), "已取消当前套索。" );
      return Render;
    }

    const auto &last = polygon_.back();
    const int dx = event.x - last.x;
    const int dy = event.y - last.y;
    if ((event.left() || event.leftUp()) && dx * dx + dy * dy >= 16)
      polygon_.push_back({event.x, event.y});
    updateOverlay();

    if (event.leftUp())
    {
      finishPolygon();
      return Render;
    }
    return Render;
  }

  return 0;
}

int BoardPolygonSelectionTool::processKeyEvent(
    QKeyEvent *event, rviz_common::RenderPanel *)
{
  if (event->key() == Qt::Key_Escape)
  {
    clearPolygon();
    RCLCPP_INFO(node_->get_logger(), "已清除标定板多边形。" );
    return Render;
  }
  if (event->key() == Qt::Key_Backspace && drawing_ && polygon_.size() > 2)
  {
    polygon_.pop_back();
    updateOverlay();
    return Render;
  }
  if ((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) && drawing_)
  {
    finishPolygon();
    return Render;
  }
  return 0;
}

void BoardPolygonSelectionTool::finishPolygon()
{
  if (polygon_.size() < 3)
  {
    RCLCPP_WARN(node_->get_logger(), "多边形至少需要 3 个顶点。" );
    clearPolygon();
    return;
  }

  drawing_ = false;
  updateOverlay();
  selectVisiblePoints(render_window_);
}

bool BoardPolygonSelectionTool::pointInsidePolygon(double x, double y) const
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon_.size() - 1; i < polygon_.size(); j = i++)
  {
    const double xi = polygon_[i].x;
    const double yi = polygon_[i].y;
    const double xj = polygon_[j].x;
    const double yj = polygon_[j].y;
    const bool crosses = ((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);
    if (crosses)
      inside = !inside;
  }
  return inside;
}

void BoardPolygonSelectionTool::selectVisiblePoints(rviz_rendering::RenderWindow *window)
{
  if (!window || !preview_cloud_ || preview_cloud_->empty())
  {
    RCLCPP_WARN(node_->get_logger(), "尚未收到 /static_cloud_preview，无法选择。" );
    return;
  }
  if (!preview_frame_id_.empty() &&
      preview_frame_id_ != context_->getFixedFrame().toStdString())
  {
    RCLCPP_ERROR(
        node_->get_logger(),
        "预览点云坐标系 %s 与 RViz Fixed Frame %s 不一致。",
        preview_frame_id_.c_str(), context_->getFixedFrame().toStdString().c_str());
    return;
  }

  Ogre::Viewport *viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
  Ogre::Camera *camera = viewport->getCamera();
  const int width = static_cast<int>(viewport->getActualWidth());
  const int height = static_cast<int>(viewport->getActualHeight());
  if (width <= 0 || height <= 0)
    return;

  int min_x = width - 1;
  int max_x = 0;
  int min_y = height - 1;
  int max_y = 0;
  for (const auto &point : polygon_)
  {
    min_x = std::min(min_x, std::clamp(point.x, 0, width - 1));
    max_x = std::max(max_x, std::clamp(point.x, 0, width - 1));
    min_y = std::min(min_y, std::clamp(point.y, 0, height - 1));
    max_y = std::max(max_y, std::clamp(point.y, 0, height - 1));
  }
  const int box_width = max_x - min_x + 1;
  const int box_height = max_y - min_y + 1;
  if (box_width <= 1 || box_height <= 1)
    return;

  const Ogre::Matrix4 projection = camera->getProjectionMatrix();
  const Ogre::Vector3 camera_position = camera->getDerivedPosition();
  const Ogre::Quaternion inverse_orientation = camera->getDerivedOrientation().Inverse();

  struct ProjectedPoint
  {
    int x;
    int y;
    float depth;
    std::size_t index;
  };
  std::vector<ProjectedPoint> projected;
  projected.reserve(preview_cloud_->size());
  std::vector<float> depth_buffer(
      static_cast<std::size_t>(box_width) * box_height,
      std::numeric_limits<float>::infinity());

  for (std::size_t index = 0; index < preview_cloud_->size(); ++index)
  {
    const auto &point = preview_cloud_->points[index];
    const Ogre::Vector3 relative(point.x, point.y, point.z);
    const Ogre::Vector3 camera_point = inverse_orientation * (relative - camera_position);
    const float depth = -camera_point.z;
    if (!(depth > 0.0f))
      continue;

    const Ogre::Vector4 clip = projection * Ogre::Vector4(
        camera_point.x, camera_point.y, camera_point.z, 1.0f);
    if (std::abs(clip.w) < 1e-6f)
      continue;
    const float ndc_x = clip.x / clip.w;
    const float ndc_y = clip.y / clip.w;
    const int pixel_x = static_cast<int>(std::lround((ndc_x + 1.0f) * 0.5f * width));
    const int pixel_y = static_cast<int>(std::lround((1.0f - ndc_y) * 0.5f * height));
    if (pixel_x < min_x || pixel_x > max_x || pixel_y < min_y || pixel_y > max_y)
      continue;

    constexpr int point_radius_pixels = 2;
    for (int dy = -point_radius_pixels; dy <= point_radius_pixels; ++dy)
    {
      for (int dx = -point_radius_pixels; dx <= point_radius_pixels; ++dx)
      {
        const int sample_x = pixel_x + dx;
        const int sample_y = pixel_y + dy;
        if (sample_x < min_x || sample_x > max_x || sample_y < min_y || sample_y > max_y)
          continue;
        const std::size_t buffer_index =
            static_cast<std::size_t>(sample_y - min_y) * box_width + (sample_x - min_x);
        depth_buffer[buffer_index] = std::min(depth_buffer[buffer_index], depth);
      }
    }
    projected.push_back({pixel_x, pixel_y, depth, index});
  }

  auto selected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  selected->reserve(projected.size());
  constexpr float visibility_tolerance = 0.06f;
  for (const auto &item : projected)
  {
    if (!pointInsidePolygon(item.x, item.y))
      continue;
    const std::size_t buffer_index =
        static_cast<std::size_t>(item.y - min_y) * box_width + (item.x - min_x);
    if (item.depth <= depth_buffer[buffer_index] + visibility_tolerance)
      selected->push_back(preview_cloud_->points[item.index]);
  }

  selected->width = static_cast<std::uint32_t>(selected->size());
  selected->height = 1;
  selected->is_dense = true;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*selected, message);
  message.header.frame_id = preview_frame_id_;
  message.header.stamp = node_->now();
  selected_publisher_->publish(message);

  RCLCPP_INFO(
      node_->get_logger(),
      "多边形选择完成：共选择 %zu 个可见预览点，已发布到 /board_polygon_selected_points。",
      selected->size());
}
}  // namespace fast_calib

PLUGINLIB_EXPORT_CLASS(fast_calib::BoardPolygonSelectionTool, rviz_common::Tool)
