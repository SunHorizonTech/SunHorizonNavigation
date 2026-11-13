# 保留包与清理边界

18 个包不等于 18 个包都必须在线启动。下列“必需”针对当前融合导航架构，不代表所有机器人或所有配置。

| 包 | 分类 | 保留理由 |
|---|---|---|
| `sunhorizon_navigation_runtime` | 核心运行链 | 组合传感器、感知、定位与 Nav2 运行 |
| `sunhorizon_navigation_core` | 核心运行链 | 状态机、定位和安全门等共享逻辑 |
| `sunhorizon_navigation_interfaces` | 核心运行链 | 组件之间的消息、服务、Action 合同 |
| `sunhorizon_livo_bringup` | 核心运行链 | 现用 LIVO 直接适配、测量时效与传播保护；包名虽是 bringup，含正式库 |
| `sunhorizon_3d_localizer` | 核心运行链 | NDT 地图定位与质量/时效门 |
| `sunhorizon_localization_manager` | 核心运行链 | 定位状态与地图坐标链管理 |
| `sunhorizon_camera_runtime` | 当前 RGB-D 配置必需 | 相机采集与来源/时序证据 |
| `sunhorizon_airy_adapter` | 当前 Airy 配置必需 | 点云、IMU 规范化和直接适配 |
| `sunhorizon_rgbd_adapter` | 当前 RGB-D 配置必需 | 深度投影、自体过滤、空间证据 |
| `sunhorizon_perception_runtime` | 当前融合配置必需 | 机身运动几何、地面与障碍分类 |
| `sunhorizon_obstacle_coverage` | 当前安全链必需 | 空间覆盖、可见性与有效性判断 |
| `sunhorizon_velocity_supervisor` | 当前安全链必需 | 最终速度发布、超时置零与锁停 |
| `sunhorizon_platform_adapter_casbot02` | 当前机器人接入必需 | 平台状态、关节/机身证据与厂家接口；换机器人需替换 |
| `sunhorizon_navigation_server` | 对外服务层 | 导航和地图业务无关接口；不是底层估计器必需 |
| `sunhorizon_navigation_diagnostics` | 辅助工具 | 就绪、时间对齐、标定诊断；不属于实时控制必需 |
| `sunhorizon_sensor_rate_probe` | 辅助工具 | 传感器频率观测；不属于实时控制必需 |
| `sunhorizon_map_projector` | 地图工作流工具 | 点云地图转二维栅格；已有地图时不是在线运行必需 |
| `sunhorizon_map_annotation_rviz` | 交互工具 | RViz 地图标注；无界面运行不需要 |

## 已删除的主要内容

- 备用路线：AMCL、GLIM、RTAB-Map、视觉定位及 backend_extras 聚合包。
- 旧组合入口：navigation_bringup、lidar_navigation_bringup、nav2_bringup 的模拟/影子/现场配置路线。Nav2 正式集成仍在 navigation_runtime。
- motion_qualification：现场短距离、偏航、速度标定验收包。
- 包内残留：GLIM tracking adapter、4 个 attended 运动试验 launch、静止资格验收脚本和无正式调用的 stationary_evidence_gate、旧多后端配置测试、投影 baseline/benchmark/parity 对比目录与工作区 benchmark。
- 非公开资料：现场配置、模型、录包、内部文档、部署工具和私有依赖。

## 仍然保留但不是旧实验的内容

- 确定性单元测试及核心录制数据回放工具是验证手段，不按 test/replay 字样删除。
- attended_geometry 被 perception_runtime 调用，attended_travel_guard 被最终速度监督器调用，均是现用保护逻辑。
- 标定、地图生成、诊断属于辅助工具，不是已废弃代码，也不是在线运行必需。

本轮以构建定义、源代码调用和现有主线依赖核对；没有把仅因命名包含 shadow/attended 就仍被正式路径调用的保护代码删掉。完整 ROS 构建及真机尚未验证。
