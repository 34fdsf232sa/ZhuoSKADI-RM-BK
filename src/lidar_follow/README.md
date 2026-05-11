# LiDAR Follow — APF + Kalman LiDAR Person Tracker

Python 移植自 [jie_deamon](https://gitee.com/s-robot/jie_deamon) 开源项目。

## 模块

| 文件 | 功能 | 来源 |
|------|------|------|
| `kalman_filter_2d.py` | 2D 匀速卡尔曼滤波器（4 状态） | `kalman_filter.hpp` |
| `apf_controller.py` | 人工势场避障控制器 | `lidar_tracker.hpp` APF 逻辑 |
| `lidar_tracker.py` | LiDAR 人腿跟踪（Kalman+APF 组合） | `lidar_tracker.hpp` |
| `test_apf.py` | APF 单元测试 + 可视化 | - |

## 核心算法

### APF 势场法

```
吸引力（目标）:    v_att = k_att * (target - robot)
排斥力（障碍物）:  F_rep = k_rep * (1/d - 1/d₀) / d² * dir

三级距离响应：
  d < 0.20m → 急停 (v=0)
  d < 0.25m → 减速 (v *= factor)
  d < 0.25m → 排斥力生效
```

### 卡尔曼滤波

```
模型: CV (Constant Velocity)
状态: [x, y, vx, vy]
预测: x' = F*x,  P' = F*P*Fᵀ + Q
更新: K = P*Hᵀ * inv(H*P*Hᵀ + R), x += K*(z - Hx)
```

### 目标跟踪

```
1. 扫描 /scan 中距当前 target 中心 TARGET_RADIUS(0.3m) 内的点
2. 质心 → 卡尔曼滤波 → 新 target 位置
3. 跟随速度: vx = (target_x - FOLLOW_DIST) * scale
             wz = atan2(target_y, target_x) * scale
4. 融合 APF 排斥力 → 最终 cmd_vel
```

## 使用

```python
from lidar_follow.lidar_tracker import LidarTracker

tracker = LidarTracker()
tracker.set_target(0.4, 0.0)  # 前方 0.4m

def publish_velocity(vx, wz):
    # 发布到 /cmd_vel

tracker.set_velocity_callback(publish_velocity)

# 在每个 /scan 回调中:
vx, wz, info = tracker.process_scan(msg.ranges, msg.angle_min, msg.angle_increment)
```

## 测试

```bash
cd src/lidar_follow
python3 test_apf.py
```
