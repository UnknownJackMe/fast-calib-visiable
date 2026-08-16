# 四球粗定位到 LiDAR 孔心自动精修：完整实施计划

状态：v1 核心流程已实现并完成双场景回归
日期：2026-08-16

## 实施结果摘要

截至 2026-08-16，Phase 1 到 Phase 4 的核心功能已经实现：

- rough seed 与 refined center 文件语义分离；
- 基于板面最大空圆和局部边界点的单孔拟合；
- 四孔候选联合几何选择；
- refined YAML、质量报告和调试点云；
- RViz 粗 seed 与绿色 refined center 同屏显示；
- 精修失败重试、成功确认后才计算外参；
- C++ 外参入口默认拒绝未验证 rough seed。

回归结果：

```text
recalib_202260816_02
  rough seed direct RMSE: 0.025906 m
  refined RMSE:           0.002541 m
  maximum seed offset:    0.1732 m

final_success_20260617
  historical manual RMSE: about 0.004935 m
  refined regression RMSE: 0.002671 m
```

剩余增强项主要是失败孔的红色局部调试 marker、更多角度/距离数据回归和阈值长期定版。

## 1. 背景与问题定义

当前 RViz2 交互流程允许用户在累计静态点云中拖动四个小球。现有实现把小球保存的位置直接当作 LiDAR 四孔中心，并直接参与相机-LiDAR 外参求解。

这不符合交互设计的原始目标：

- 人使用鼠标只能给出孔的大致位置；
- RDP、点云稀疏、视角和球体遮挡都会产生厘米级误差；
- 小球应当是局部搜索种子，不是最终测量值；
- 最终孔心必须由程序在种子附近的真实点云中自动精修得到。

当前场景 `recalib_202260816_02` 可以直接说明问题：

- 粗球直接计算的外参 RMSE 为 `0.025906 m`；
- 两列上下距离分别约为 `0.349 m` 和 `0.371 m`；
- 标定板真实上下孔距为 `0.400 m`；
- 上一次人工仔细确认的参考结果 RMSE 约为 `0.004935 m`。

因此后续流程必须严格区分：

```text
rough seeds       用户粗略拖动的小球位置
refined centers   程序从局部点云拟合出的孔心
```

**粗种子不得静默回退为最终孔心。**

## 2. 目标

实现以下完整流程：

```text
累计静态点云
  -> RViz 四球粗定位
  -> 保存四个 rough seeds
  -> 每个 seed 附近提取局部点云
  -> 局部板面估计与坐标展开
  -> 孔边缘候选提取
  -> 圆孔中心和半径鲁棒拟合
  -> 四孔矩形几何验证
  -> 输出 refined centers 和质量报告
  -> 使用 refined centers 计算外参
```

用户只需要把小球拖到正确孔附近，不需要像素级或毫米级手工操作。

## 3. 非目标

本阶段不处理：

- 动态标定或运动中的标定板；
- 相机-LiDAR 时间偏移估计；
- 任意形状标定板；
- 不知道孔距、孔径的无先验检测；
- 通过粗球强行构造完美矩形后直接求外参；
- 在局部拟合失败时自动使用粗球结果冒充成功。

## 4. 用户最终工作流

### 4.1 数据准备

用户可以使用：

- 现场单张相机 PNG + LiDAR bag；或
- 相机和 LiDAR 合并 rosbag。

程序生成：

```text
calib_data/<scene>/image.png
calib_data/<scene>/lidar_bag/
output/<scene>/filtered_cloud.ply
```

### 4.2 粗球交互

RViz2 中显示四个带名称的球：

```text
upper +Y
upper -Y
lower +Y
lower -Y
```

用户要求：

- 每个球放到对应孔附近；
- 球落入孔的局部搜索范围即可；
- 不要求球心与真实孔心重合；
- 球不能放到错误孔、板边缘或其他空洞附近。

### 4.3 自动精修

保存粗球后，程序自动运行局部精修，并在 RViz 中同时显示：

- 粗种子：半透明或小尺寸标记；
- 精修孔心：实心、高对比度标记；
- 局部搜索区域；
- 成功/失败颜色；
- 拟合圆环或孔轮廓；
- 每个孔的拟合残差和半径。

### 4.4 用户确认

如果四个孔全部通过验证，用户确认后计算外参。

如果任何一个孔失败：

1. RViz 标红失败孔；
2. 用户只调整失败孔的粗球；
3. 重新运行局部精修；
4. 不重新录制，不重新处理其他成功孔。

## 5. 数据文件与命名

### 5.1 粗种子文件

新名称：

```text
output/<scene>/manual_lidar_hole_seeds.yaml
```

格式：

```yaml
frame_id: livox_frame
source_cloud: output/<scene>/filtered_cloud.ply
centers:
- name: upper +Y
  x: 2.40
  y: 0.22
  z: 1.15
- name: upper -Y
  x: 2.41
  y: -0.28
  z: 1.18
- name: lower +Y
  x: 2.40
  y: 0.22
  z: 0.78
- name: lower -Y
  x: 2.42
  y: -0.28
  z: 0.80
```

兼容策略：

- 旧 `manual_lidar_holes.yaml` 可以作为 seed 文件读取；
- 新流程不再把它解释为精确中心；
- 文档和日志中统一使用 `seed`，避免语义混淆。

### 5.2 精修孔心文件

```text
output/<scene>/refined_lidar_holes.yaml
```

建议格式：

```yaml
frame_id: livox_frame
algorithm: local_plane_circle_refinement_v1
centers:
- name: upper +Y
  seed: {x: 2.40, y: 0.22, z: 1.15}
  refined: {x: 2.397, y: 0.247, z: 1.176}
  seed_distance_m: 0.038
  fitted_radius_m: 0.116
  radial_rmse_m: 0.006
  inlier_count: 184
  status: pass
```

### 5.3 质量报告

```text
output/<scene>/hole_refinement_report.yaml
```

报告包含：

- 每孔局部点数；
- 板面内点数和残差；
- 边缘候选点数；
- 圆拟合内点数；
- 拟合半径；
- 径向 RMSE；
- seed 到 refined center 的距离；
- 四孔边长、对角线和平面度；
- 总体 pass/warn/fail；
- 失败原因和建议调整方向。

### 5.4 调试点云

每个孔保存独立中间结果：

```text
output/<scene>/refinement_debug/upper_pos_y_roi.ply
output/<scene>/refinement_debug/upper_pos_y_plane.ply
output/<scene>/refinement_debug/upper_pos_y_edges.ply
output/<scene>/refinement_debug/upper_pos_y_circle_inliers.ply
```

这样即使标定板已经撤走，也可以离线复现问题。

## 6. 算法设计

## 6.1 输入点云选择

优先使用：

```text
output/<scene>/filtered_cloud.ply
```

必要时也读取：

```text
aligned_cloud.ply
plane_cloud.ply
edge_cloud.ply
```

第一版以 `filtered_cloud.ply` 为主，避免依赖现有全局圆孔检测是否成功。

## 6.2 局部 ROI 提取

以每个粗 seed 为中心，提取各向异性 3D ROI。

建议初始参数：

```yaml
seed_search_radius_x: 0.12
seed_search_radius_y: 0.18
seed_search_radius_z: 0.18
```

原因：

- X 主要是板面法向方向，窗口应更窄；
- Y/Z 是板面内方向，允许鼠标粗定位误差；
- 参数必须可配置，不能写死当前场景距离和方向。

如果局部点数不足，可以进行一次受限扩窗：

```text
1.0x -> 1.5x
```

不得无限扩展，以免抓到相邻孔或板边缘。

## 6.3 局部板面估计

每个 ROI 独立拟合局部平面：

- PCL RANSAC plane；
- 限制距离阈值；
- 计算平面法向和点到平面残差；
- 法向方向统一朝向 LiDAR 或使用全局板面法向消除正负号。

建议初始参数：

```yaml
local_plane_distance_threshold: 0.015
local_plane_min_inliers: 100
local_plane_max_rmse: 0.012
```

平面拟合失败时该孔直接 fail，不使用 seed 替代。

## 6.4 建立局部二维坐标系

根据局部平面法向建立正交基：

```text
u, v：板面内坐标
n：板面法向
```

将 ROI 点投影到 `(u, v)` 平面，使后续圆拟合不依赖标定板在 LiDAR 坐标系中的姿态。

保存：

- 3D 到 2D 变换；
- 平面原点；
- `u/v/n` 基向量；
- 投影前后的对应关系。

## 6.5 孔边缘候选提取

第一版采用组合方法，而不是只依赖一种边缘检测：

1. 去除明显偏离板面的噪点；
2. 在 seed 投影点附近建立二维搜索圆盘；
3. 计算局部点密度或邻域边界；
4. 提取孔周围的边界点；
5. 可结合现有 `edge_cloud.ply` 作为附加候选，但不作为唯一输入。

建议候选条件：

- 距 seed 的板面内距离小于局部搜索半径；
- 半径候选围绕标称孔半径 `0.120 m`；
- 排除 ROI 外边界和标定板外轮廓；
- 保留孔内壁/孔缘产生的稳定回波。

## 6.6 圆拟合

使用两级拟合：

### 第一级：RANSAC Circle2D

- 从边缘候选中拟合二维圆；
- 半径限制在标称半径附近；
- 获得初始圆心、半径和内点。

### 第二级：加权最小二乘精修

- 使用 RANSAC 内点；
- 最小化径向残差；
- 对远离平面或反射异常点降权；
- 输出精修圆心和协方差/稳定性指标。

建议初始参数：

```yaml
expected_hole_radius: 0.120
min_fitted_radius: 0.075
max_fitted_radius: 0.155
circle_ransac_distance_threshold: 0.012
circle_min_inliers: 30
circle_max_radial_rmse: 0.015
```

这些阈值需要用当前数据集回归后再定版。

## 6.7 恢复三维孔心

将二维圆心通过局部平面坐标变换恢复到 LiDAR 三维坐标系。

最终三维孔心应位于拟合板面上，而不是直接沿用 seed 的 X/Y/Z。

## 6.8 四孔联合几何验证

单孔拟合完成后，对四孔整体做验证。

已知几何：

```text
宽度：0.500 m
高度：0.400 m
对角线：sqrt(0.5^2 + 0.4^2) ≈ 0.640312 m
```

验证内容：

- 上排宽度；
- 下排宽度；
- +Y 侧高度；
- -Y 侧高度；
- 两条对角线；
- 四点共面误差；
- 相邻边夹角；
- 是否出现重复孔或标签交换。

几何先验主要用于：

- 拒绝错误候选；
- 在多个局部圆候选之间选择最佳组合；
- 输出质量评分。

第一版不应把四点强制修改成一个完美矩形后再输出，因为这会隐藏真实拟合误差。

## 6.9 多候选选择

如果一个 seed ROI 内拟合出多个圆候选，综合评分：

```text
score =
  seed 距离项
  + 半径误差项
  + 径向残差项
  + 平面残差项
  + 四孔整体几何项
```

必须保证四个 seed 最终映射到四个不同候选。

## 7. 参数设计

在 `config/qr_params.yaml` 中新增独立区块：

```yaml
# Rough-seed local LiDAR hole refinement
hole_refinement_enabled: true
seed_search_radius_x: 0.12
seed_search_radius_y: 0.18
seed_search_radius_z: 0.18
seed_search_expand_factor: 1.5
local_plane_distance_threshold: 0.015
local_plane_min_inliers: 100
local_plane_max_rmse: 0.012
expected_hole_radius: 0.120
min_fitted_radius: 0.075
max_fitted_radius: 0.155
circle_ransac_distance_threshold: 0.012
circle_min_inliers: 30
circle_max_radial_rmse: 0.015
max_seed_to_refined_distance: 0.20
max_side_length_error: 0.035
max_diagonal_error: 0.045
max_four_point_planarity_error: 0.015
```

参数原则：

- 所有距离单位统一为米；
- 默认值必须适用于当前 MID360 数据；
- 每个阈值在报告中显示；
- 不通过时输出具体阈值和实际值。

## 8. 程序结构计划

## 8.1 新增核心模块

建议新增：

```text
src/lidar_hole_refiner.hpp
tools/refine_lidar_hole_seeds.cpp
```

核心类职责：

```cpp
class LidarHoleRefiner {
 public:
  RefinementResult refine(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
      const std::vector<NamedSeed>& seeds);
};
```

不要把新逻辑继续堆进 `manual_lidar_centers_calib.cpp`。

## 8.2 修改交互编辑器

`interactive_lidar_hole_editor.py`：

- 输出文件语义改为 seed；
- 服务名称兼容旧名称，同时新增清晰名称：

```text
/save_lidar_hole_seeds
```

- 球标签明确显示 `rough seed`；
- 保存时只承诺“大致位置”。

## 8.3 修改工作流脚本

新顺序：

```text
保存 seeds
  -> refine_lidar_hole_seeds
  -> 检查 refinement report
  -> 全部 pass 才运行 extrinsic calibration
```

失败时：

- 保持 RViz 和 editor 运行；
- 不结束整个场景；
- 提示用户调整失败 seed；
- 支持重新触发精修。

## 8.4 修改外参工具

现有 `manual_lidar_centers_calib` 应改名或增加更明确入口，例如：

```text
refined_lidar_centers_calib
```

输入必须是：

```text
refined_lidar_holes.yaml
```

如果输入文件标记为 `rough seeds`，程序应拒绝运行，而不是继续计算。

为了兼容历史结果，可以保留显式参数：

```text
allow_unvalidated_manual_centers:=false
```

默认必须为 false。

## 9. RViz 可视化计划

精修完成后额外发布：

```text
/rough_lidar_hole_seeds
/refined_lidar_hole_centers
/lidar_hole_refinement_debug
```

颜色建议：

- 粗 seed：灰色或半透明；
- 精修成功：绿色；
- warning：黄色；
- fail：红色；
- 圆拟合内点：青色；
- 被拒绝点：暗红色。

每个孔的文字标签显示：

```text
upper +Y
r=0.116 m
rmse=0.006 m
inliers=184
PASS
```

RDP 模式继续使用低频静态发布，不播放实时 bag。

## 10. 质量门限与状态

每个孔状态：

### PASS

- 局部平面拟合成功；
- 圆内点数量足够；
- 半径在允许范围；
- 径向 RMSE 小于阈值；
- refined center 未偏离 seed 搜索范围。

### WARN

- 单孔拟合基本通过；
- 但整体矩形几何接近阈值；
- 允许用户查看后手动确认，不自动计算最终外参。

### FAIL

- 找不到局部平面；
- 找不到圆；
- 半径异常；
- 候选与相邻孔重复；
- 整体几何明显不符。

总体规则：

```text
4 PASS       -> 允许自动进入外参计算
存在 WARN    -> 等待用户确认
存在 FAIL    -> 禁止外参计算，返回 RViz 调整
```

## 11. 外参结果验收

最终外参不仅检查是否生成文件，还要检查：

- 四孔刚体配准 RMSE；
- refined centers 的矩形几何；
- 彩色点云投影视觉效果；
- 外参相对历史/机械安装是否出现不合理跳变。

建议初始门限：

```text
目标 RMSE：<= 0.008 m
理想 RMSE：<= 0.005 m
警告范围：0.008 m 到 0.015 m
拒绝：> 0.015 m
```

最终阈值以多组真实场景回归结果为准。

## 12. 测试计划

## 12.1 单元测试

构造合成数据：

- 带噪平面；
- 四个已知圆孔；
- 缺点、离群点和不均匀采样；
- seed 偏移 2、5、10、15 cm；
- 相邻板边缘和假圆候选。

验证：

- 精修中心误差；
- 半径误差；
- 候选唯一性；
- 失败场景是否正确拒绝。

## 12.2 当前数据回归

核心回归场景：

```text
recalib_202260816_02
final_success_20260617
```

对 `recalib_202260816_02`：

- 输入当前厘米级粗 seed；
- 验证程序能在每个 seed 周围找到对应孔；
- refined centers 的宽高接近 `0.500 × 0.400 m`；
- 外参 RMSE 显著低于当前 `0.025906 m`；
- 目标先达到 `< 0.010 m`，再优化到接近 `0.005 m`。

对 `final_success_20260617`：

- 使用历史人工确认孔心作为参考；
- 比较精修中心与参考点；
- 确保新流程不劣化已成功场景。

## 12.3 失败测试

- seed 放到错误孔；
- 两个 seed 放到同一个孔；
- seed 放到板边缘；
- 只有三个孔可见；
- 点云太稀；
- ROI 太小；
- 半径参数错误；
- 标定板在录制中移动。

程序必须给出可读失败原因，不能崩溃或输出伪成功结果。

## 13. 分阶段实施

### Phase 1：数据契约和只读诊断（已完成）

- 定义 seed/refined/report 文件格式；
- 读取现有 seed YAML；
- 提取每孔 ROI；
- 保存调试点云；
- 不改现有外参流程。

验收：四个 ROI 与用户指定孔对应。

### Phase 2：单孔局部精修（已完成）

- 局部平面拟合；
- 二维投影；
- 边缘提取；
- RANSAC 圆拟合；
- 每孔独立报告。

验收：当前数据四个孔至少三个稳定通过，失败孔原因明确。

### Phase 3：四孔联合验证（已完成）

- 多候选组合；
- 防止重复匹配；
- 宽高、对角线、平面度检查；
- 输出总体 pass/warn/fail。

验收：错误 seed 不会生成伪成功四孔。

### Phase 4：工作流集成（核心流程已完成）

- RViz seed 保存后自动运行精修；
- 发布 refined markers；失败孔红色调试 marker 作为后续增强；
- 只有验证通过才计算外参；
- 失败时允许只调整单个 seed。

验收：用户无需手工精确拖球。

### Phase 5：回归、阈值定版与文档（双场景与文档已完成）

- 回归历史和当前数据；
- 固化默认阈值；
- 更新 README/README_zh；
- 记录典型成功与失败截图（待补充更多失败样本）；
- 添加命令级测试说明和合成回归测试。

## 14. 风险与应对

### 点云孔边缘不是完整圆

应对：允许部分圆弧拟合，使用 RANSAC + 几何先验，不要求 360° 完整边缘。

### MID360 入射角导致半径偏差

应对：先拟合板面并投影到二维，避免在原始三维坐标中直接拟合圆。

### seed ROI 包含板外边缘

应对：限制圆心与 seed 距离，结合半径和四孔整体几何评分。

### 点云累计包含轻微运动

应对：平面残差、圆残差和局部厚度超限时 fail，并提示重新录制。

### 四孔先验掩盖错误检测

应对：先独立拟合，再用几何做验证和候选选择；不把粗结果强行变成完美矩形。

### 参数过度绑定当前场景

应对：使用板面局部坐标和相对尺度；至少用两个不同距离/角度场景回归。

## 15. 完成定义

该任务只有同时满足以下条件才算完成：

- 用户粗球偏差达到 10–15 cm 时仍能找到正确孔；
- 粗 seed 与 refined center 在文件和 RViz 中明确区分；
- 四个 refined centers 全部来自点云拟合；
- 任一孔失败时禁止生成最终外参；
- 当前场景 RMSE 从 `0.025906 m` 显著下降到 `< 0.010 m`；
- 历史成功场景结果不劣于原有水平；
- 合并 bag 离线流程和现场 PNG + LiDAR 流程都能使用；
- RDP 下只使用低频静态点云；
- 生成完整质量报告和可复查调试点云；
- 中英文文档和实际命令一致。

## 16. 后续增强

v1 核心流程已经完成。后续工作不应再回退到“粗球直接计算外参”，建议按以下顺序增强：

1. 增加错误孔、重复孔、板边缘和点云过稀等失败回归样本；
2. 在 RViz 中为失败孔发布红色局部调试 marker；
3. 用更多距离、入射角和光照条件的数据长期收敛默认阈值；
4. 补充典型成功/失败截图和现场验收记录；
5. 如需重放历史手工中心，只能显式设置 `allow_unvalidated_manual_centers:=true`。
