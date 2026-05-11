# mapless_nav_v2 — COD 架构风格独立版本

参考 COD RM2026 导航项目架构，重构为模块化、双模式启动的独立导航系统。

## 与 COD 架构对比

| 组件 | COD 实现 | mapless_nav_v2 实现 |
|------|---------|-------------------|
| 启动模式 | `multiplenav_launch` / `singlenav_launch` | `patrol_nav.launch` / `simple_nav.launch` |
| 核心 bringup | `navigation_launch.py` (7 节点) | `bringup.launch.py` (相同 7 节点) |
| 自点云过滤 | `cpp_lidar_filter` (C++ 独立包) | 已集成到 `pointcloud_fusion_node` (PCL CropBox) |
| 点云转 LaserScan | `pointcloud_to_laserscan` (C++ 包) | `pointcloud_to_scan_node.py` (Python) |
| 里程计 | `small_point_lio` | `real_odom_node.py` (l2_imu/external/fake) |
| 航点系统 | RViz `waypoint_editor` + CSV | `waypoint_patrol_node.py` + `wps/*.csv` |
| 返航 | 无独立节点 | `return_home_node.py` |
| 底盘适配 | `fake_vel_transform` (全向) | 无需（差速底盘） |
| 行为树 | 自定义 BT XML | 自定义 BT XML（带重规划+恢复） |
| 代价图 | STVL (强度体素层) | VoxelLayer + ObstacleLayer |
| 控制器 | GoalApproachController → MPPI | RegulatedPurePursuitController |
| 规划器 | SmacPlanner2D | NavfnPlanner |
| 视觉跟踪 | 无 | YOLO+ByteTrack (target_tracker_node) |

## 架构图

```
                    patrol_nav.launch.py / simple_nav.launch.py
                                   |
              +------------------+-+------------------+
              |                  |                    |
        传感器处理层          里程计层            Nav2 导航栈
              |                  |                    |
    pointcloud_fusion_node  real_odom_node    bringup.launch.py
    (自过滤+融合)          (fake/l2/ext)    +------------------+
              |                  |           | controller_server|
    pointcloud_to_scan_node     |           | smoother_server  |
    (/fused_pc -> /scan)        |           | planner_server   |
              |                  |           | behavior_server  |
         [代价图输入]       odom->base_link  | bt_navigator     |
              |                  |           | waypoint_follower|
              +------------------+           | velocity_smoother|
                     |                       +------------------+
              高层导航服务                          |
        return_home_node  waypoint_patrol_node     |
              |                  |                 |
         /return_home      /patrol/start      /cmd_vel
        NavigateToPose    FollowWaypoints         |
              |                  |                 |
              +------------------+-----------------+
                                 |
                            [底盘控制]
```

## 启动模式

### Patrol Mode (巡逻模式)
```bash
ros2 launch mapless_nav_v2 patrol_nav.launch.py
ros2 launch mapless_nav_v2 patrol_nav.launch.py enable_patrol:=true odom_mode:=l2_imu
```

### Simple Mode (单目标导航)
```bash
ros2 launch mapless_nav_v2 simple_nav.launch.py
ros2 launch mapless_nav_v2 simple_nav.launch.py odom_mode:=external
```

## 行为树

- `navigate_to_pose_w_replanning_and_recovery.xml` — 单目标 + 重规划 + 恢复
- `navigate_through_poses_w_replanning_and_recovery.xml` — 多航点 + RemovePassedGoals + 恢复

## 航点格式 (CSV)

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command
0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,start
1,1.0,0.0,0.0,0.0,0.0,0.0,1.0,patrol
2,0.0,0.0,0.0,0.0,0.0,0.0,1.0,end
```

- `command`: start/patrol/end（与 COD 兼容）
- 存储在 `wps/` 目录
- 可通过 `waypoint_loader.py` 读写

## 与 COD 的主要差异

1. **差速底盘** — 无需 fake_vel_transform
2. **无地图模式** — 代价图使用 rolling_window，无 static_layer
3. **多传感器融合** — L2 360° + Mid-70 + Berxel 深度
4. **视觉跟踪** — YOLO+ByteTrack 目标跟踪（COD 无此功能）
5. **简洁实现** — Python 节点优先，降低复杂度
