# FAST-Calib 标定记录索引

记录时间：2026-06-17

本目录记录这次 MID360 + Hikvision 相机标定过程中遇到的问题、设备配置、最终可用数据和复现步骤。后续重新标定时，优先看 `quick_start.md`，遇到问题再查 `pitfalls_and_solutions.md`。

## 文件说明

- `quick_start.md`：下次重新标定的最短流程。
- `interactive_workflow.md`：RViz 可视化拖球标定流程。
- `rough_seed_refinement_plan.md`：四球粗定位到自动孔心精修的设计、实施阶段和验收标准。
- `device_config.md`：设备、网络、ROS、相机内参、标定板参数和关键路径。
- `pitfalls_and_solutions.md`：这次遇到的坑、现象、原因和解决方案。
- `final_result_20260617.md`：这次最终采用的四孔外参结果。

## 本次最终采用结果

最终结果目录：

```text
/home/vision/FAST-Calib/output/final_success_20260617
```

外参文件：

```text
/home/vision/FAST-Calib/output/final_success_20260617/calib_result.txt
```

四孔配准 RMSE：

```text
0.004935 m
```

本次结果是基于人工确认的四个 LiDAR 孔中心计算的，不依赖自动 LiDAR 圆洞检测。
