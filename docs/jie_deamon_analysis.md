# jie_deamon 开源项目分析与功能移植

## 项目概述

[Gitee: s-robot/jie_deamon](https://gitee.com/s-robot/jie_deamon)  
[GitHub: 6-robot/jie_deamon](https://github.com/6-robot/jie_deamon)

智元机器狗后端服务节点，基于 ROS 2 Humble + C++17，集成 LiDAR 目标追踪、Web 可视化、Android App 通讯。

## 架构对比

| | jie_deamon | 本项目 (mapless_nav) |
|------|-----------|---------------------|
| 语言 | C++17 | Python + C++ |
| 目标跟踪 | LiDAR 人腿跟随 (/scan) | YOLO+ByteTrack 视觉跟踪 |
| 避障方式 | APF 势场法 | obstacle_threshold 停障 |
| 位置平滑 | 2D 卡尔曼滤波 | ByteTrack 内置关联 |
| 可视化 | Web HTTP+WS（8080/8890） | RViz |
| 远程控制 | Android UDP + Web 摇杆 | ROS2 服务/话题 |
| 底盘控制 | 全向 (vx, vy, wz) | 差速 (vx, wz) |
| 动作指令 | /d1_cmd (stand/sit/liedown...) | 无 |

## 可移植模块

### 1. APF 势场避障（lidar_tracker.hpp）

**算法核心：**

```
机器人与目标之间形成吸引力 F_att = k_att * (target - robot)
障碍物产生排斥力 F_rep = k_rep * (1/d - 1/d0) / d^2 * dir
合速度 = F_att * velocity_scale + F_rep

三个距离阈值：
  APF_INFLUENCE_DIST = 0.25m  排斥力生效范围
  APF_SLOWDOWN_DIST  = 0.25m  减速区（速度×slowdown_factor）
  APF_EMERGENCY_DIST = 0.20m  急停区（速度=0）
```

**状态：✅ 已移植** → `src/lidar_follow/apf_controller.py`

### 2. 2D 卡尔曼滤波（kalman_filter.hpp）

**算法：** 4 状态 CV 模型 [x, y, vx, vy]，无外部矩阵库依赖

```
状态转移: x' = x + vx*dt, y' = y + vy*dt
观测:     z = [x, y]
预测:     F*P*F^T + Q（展开计算，4×4 矩阵）
更新:     K = P*H^T * inv(H*P*H^T + R)，x = x + K*(z - Hx)
```

**状态：✅ 已移植** → `src/lidar_follow/kalman_filter_2d.py`

### 3. LiDAR 人腿跟随（lidar_tracker.hpp）

**算法：**

```
1. 扫描 /scan 中距离 target 中心 TARGET_RADIUS(0.3m) 内的点
2. 计算这些点的质心作为新的 target 位置
3. 用卡尔曼滤波平滑 target 位置
4. 计算跟随速度：
   - vx = (target_x - FOLLOW_DIST(0.4m)) * LINEAR_SCALE(0.5)
   - wz = atan2(target_y, target_x) * ANGULAR_SCALE(1.0)
   - vy = -通道宽度差 * LATERAL_SCALE(1.0)
5. 融合 APF 避障排斥力
6. 机器人框架点排除（ROBOT_FRAME）
```

**状态：✅ 已移植** → `src/lidar_follow/lidar_tracker.py`

### 4. Web 可视化（web_server.hpp）

纯 C++ HTTP + WebSocket 服务器，零外部依赖。提供：
- `/scan` 点云数据实时推送（WebSocket JSON）
- 虚拟摇杆控制
- 跟随模式切换
- 机器狗姿态控制

**状态：📋 可选** → 已有 RViz 可视化，暂不移植

## 移植模块目录结构

```
src/lidar_follow/
├── README.md               # 模块说明
├── apf_controller.py       # APF 势场避障控制器
├── kalman_filter_2d.py     # 2D 卡尔曼滤波器
├── lidar_tracker.py        # LiDAR 人腿跟随追踪器
└── test_apf.py             # APF 单元测试
```

## 关键参数对标

| 参数 | jie_deamon (C++) | lidar_follow (Python) |
|------|-----------------|----------------------|
| FOLLOW_DIST | 0.4m | 0.4m |
| TARGET_RADIUS | 0.3m | 0.3m |
| MAX_LINEAR_SPEED | 1.0 m/s | 0.5 m/s |
| MAX_ANGULAR_SPEED | 1.0 rad/s | 1.0 rad/s |
| APF_INFLUENCE_DIST | 0.25m | 0.25m |
| APF_EMERGENCY_DIST | 0.20m | 0.20m |
| APF_REPULSE_GAIN | 0.01 | 0.01 |
| ROBOT_FRAME | 前0.15 后0.35 左右0.15 | 前0.15 后0.35 左右0.15 |

## 与现有系统集成

```
原有管线:
  /target_tracker/primary_target (视觉) → following_controller_node → /cmd_vel

新增管线（互补）:
  /scan (L2 LiDAR) → lidar_tracker.py (APF+Kálmán) → /cmd_vel
                                                          ↓
                                    可与视觉跟踪结果融合或独立运行
```

当视觉跟踪失效（暗光/遮挡），LiDAR 跟随仍可工作。两者可组合：
- 视觉提供目标身份识别（谁是人）
- APF 提供平滑避障追踪（怎么跟）
