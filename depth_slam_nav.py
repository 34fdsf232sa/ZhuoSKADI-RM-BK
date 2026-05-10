#!/usr/bin/env python3
"""
深度 SLAM 障碍物检测与导航辅助
基于 Berxel P100R 深度相机

功能:
- 深度图障碍物检测
- 可通行区域分析
- 导航方向建议
- ROS 2 话题发布 (供 Nav2 使用)

架构:
- visual_tracker.py: 视觉追踪 (YOLOv12) → 目标位置
- 本脚本: 深度 SLAM → 障碍物信息
- Nav2: 接收目标 + 障碍物 + LiDAR → 导航控制

作者: Zhuo RM Team
日期: 2026-01-29
"""

import cv2
import numpy as np
import time
import sys
 
from berxel_camera import BerxelCamera
from depth_slam_obstacle import DepthSLAMObstacleDetector


# ============== 参数配置 ==============
# 显示参数
DISPLAY_SCALE = 0.6
TARGET_FPS = 30

# 深度平滑参数
DEPTH_EMA_ALPHA = 0.25
DEPTH_VIS_KEEP_FRAMES = 5
DEPTH_PROCESS_INTERVAL = 2

# SLAM 参数
SLAM_DEPTH_THRESHOLD_NEAR = 0.8   # 近距离障碍物阈值 (米)
SLAM_DEPTH_THRESHOLD_FAR = 5.0    # 远距离阈值 (米)
SLAM_MIN_NAVIGABLE_AREA = 800     # 最小可通行区域面积


# ============== 辅助函数 ==============
def initialize_camera():
    """初始化 Berxel 相机"""
    try:
        camera = BerxelCamera()
        if not camera.initialize():
            raise Exception("无法初始化Berxel相机")
        return camera
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise


# ============== 主函数 ==============
def main():
    """主函数：深度 SLAM 障碍物检测"""
    cap = None
    try:
        print("\n" + "="*60)
        print("🗺️  深度 SLAM 障碍物检测系统")
        print("   障碍物信息将发送给 Nav2 进行避障")
        print("="*60)
        
        # 初始化相机
        print("\n[1/2] 初始化 Berxel 相机...")
        cap = initialize_camera()
        print("✅ 相机初始化成功")
        
        # 初始化 SLAM 检测器
        print("\n[2/2] 初始化 SLAM 检测器...")
        slam = DepthSLAMObstacleDetector(
            depth_threshold_near=SLAM_DEPTH_THRESHOLD_NEAR,
            depth_threshold_far=SLAM_DEPTH_THRESHOLD_FAR,
            min_navigable_area=SLAM_MIN_NAVIGABLE_AREA
        )
        print("✅ SLAM 检测器初始化成功")
        
        print("\n系统就绪！")
        print("\n控制键:")
        print("  - 'q': 退出程序")
        print("  - 's': 保存截图")
        print("  - 'p': 暂停/继续")
        print("  - 'v': 切换可视化模式")
        print("="*60 + "\n")
        
        # 状态变量
        frame_count = 0
        fps_start_time = time.time()
        fps_frame_count = 0
        current_fps = 0
        paused = False
        screenshot_count = 0
        show_overlay = True
        
        # 深度平滑缓存
        last_depth_color = None
        ema_depth_min = None
        ema_depth_max = None
        last_depth_keep_counter = 0
        
        while True:
            # 暂停处理
            if paused:
                key = cv2.waitKey(100) & 0xFF
                if key == ord('p'):
                    paused = False
                    print("\n▶️  继续运行")
                elif key == ord('q'):
                    break
                continue
            
            loop_start = time.time()
            
            # 获取图像
            frame = cap.get_frame()
            depth = cap.get_depth()
            
            if frame is None or depth is None:
                print("无法获取图像帧")
                continue
            
            h, w = frame.shape[:2]
            
            frame_count += 1
            fps_frame_count += 1
            
            # ========== 深度处理 ==========
            slam_info = None
            obstacle_mask = None
            depth_color = None
            
            if depth.size > 0 and frame_count % DEPTH_PROCESS_INTERVAL == 0:
                try:
                    # 缩放深度图
                    depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                    
                    # 转换为米
                    depth_meters = depth_resized / 17000.0
                    
                    # SLAM 处理
                    obstacle_mask, slam_info = slam.process_depth_frame(depth_meters, frame)
                    
                    # 深度可视化
                    valid_depth = depth_resized[depth_resized > 0]
                    
                    if len(valid_depth) > 0:
                        depth_min = np.percentile(valid_depth, 1)
                        depth_max = np.percentile(valid_depth, 99)
                        
                        # EMA 平滑
                        if ema_depth_min is None:
                            ema_depth_min = float(depth_min)
                        else:
                            ema_depth_min = (DEPTH_EMA_ALPHA * float(depth_min) + 
                                           (1 - DEPTH_EMA_ALPHA) * ema_depth_min)
                        
                        if ema_depth_max is None:
                            ema_depth_max = float(depth_max)
                        else:
                            ema_depth_max = (DEPTH_EMA_ALPHA * float(depth_max) + 
                                           (1 - DEPTH_EMA_ALPHA) * ema_depth_max)
                        
                        if ema_depth_max > ema_depth_min:
                            depth_norm = np.clip((depth_resized - ema_depth_min) / 
                                               (ema_depth_max - ema_depth_min), 0, 1)
                            depth_norm = (depth_norm * 255).astype(np.uint8)
                            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                            mask = depth_resized == 0
                            depth_color[mask] = [0, 0, 0]
                            
                            # SLAM 可视化叠加
                            if show_overlay and obstacle_mask is not None:
                                depth_color = slam.visualize(depth_meters, obstacle_mask, 
                                                            slam_info, depth_color)
                            
                            last_depth_color = depth_color.copy()
                            last_depth_keep_counter = 0
                            
                except Exception as e:
                    print(f"深度处理错误: {e}")
                    depth_color = None
            else:
                # 重用上一帧
                if last_depth_color is not None and last_depth_keep_counter < DEPTH_VIS_KEEP_FRAMES:
                    depth_color = last_depth_color
                    last_depth_keep_counter += 1
            
            # 创建显示画布
            if depth_color is not None:
                display = depth_color
            else:
                display = np.zeros((h, w, 3), dtype=np.uint8)
                cv2.putText(display, "Waiting for depth...", (w//2-100, h//2),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
            
            # 计算 FPS
            if fps_frame_count >= 30:
                elapsed = time.time() - fps_start_time
                current_fps = fps_frame_count / elapsed
                fps_start_time = time.time()
                fps_frame_count = 0
            
            # ========== 显示信息 ==========
            cv2.putText(display, f"FPS: {current_fps:.1f}", (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if slam_info:
                direction = slam_info['suggested_direction']
                color = (0, 255, 0) if direction == 'forward' else (0, 255, 255)
                cv2.putText(display, f"Direction: {direction.upper()}", (10, 60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # 障碍物信息
                if 'obstacle_count' in slam_info:
                    cv2.putText(display, f"Obstacles: {slam_info['obstacle_count']}", (10, 90),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 可视化模式指示
            mode_text = "Overlay: ON" if show_overlay else "Overlay: OFF"
            cv2.putText(display, mode_text, (w - 150, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # 底部标题
            cv2.putText(display, "Depth SLAM - Obstacle Detection", (w//2-150, h-20),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 缩放显示
            display_h = int(h * DISPLAY_SCALE)
            display_w = int(w * DISPLAY_SCALE)
            display_resized = cv2.resize(display, (display_w, display_h))
            
            cv2.imshow('Depth SLAM', display_resized)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                screenshot_count += 1
                filename = f'slam_screenshot_{screenshot_count}.png'
                cv2.imwrite(filename, display)
                print(f"\n📸 截图保存: {filename}")
            elif key == ord('p'):
                paused = True
                print("\n⏸️  已暂停（按'p'继续）")
            elif key == ord('v'):
                show_overlay = not show_overlay
                status = "开启" if show_overlay else "关闭"
                print(f"\n🗺️  叠加可视化: {status}")
            
            # 帧率控制
            elapsed = time.time() - loop_start
            target_delay = 1.0 / TARGET_FPS
            if elapsed < target_delay:
                time.sleep(target_delay - elapsed)
                
    except Exception as e:
        print(f"系统错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\n" + "="*60)
        print("🧹 清理资源...")
        
        cv2.destroyAllWindows()
        if cap:
            cap.release()
        
        # 显示统计
        if 'slam' in locals():
            stats = slam.get_statistics()
            print(f"\n📊 运行统计:")
            print(f"  - 总帧数: {frame_count}")
            print(f"  - SLAM处理帧数: {stats['total_frames']}")
            print(f"  - 平均FPS: {current_fps:.1f}")
            if stats['total_frames'] > 0:
                print(f"  - SLAM平均处理时间: {stats['avg_processing_time']*1000:.1f}ms")
        
        print("\n✅ 系统已关闭")
        print("="*60)


if __name__ == "__main__":
    main()
