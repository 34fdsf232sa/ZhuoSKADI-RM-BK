#!/usr/bin/env python3
"""
人员检测 + SLAM导航集成系统
结合YOLOv8人员检测与深度SLAM避障

功能:
- 实时人员检测与跟踪
- 深度SLAM障碍物检测
- 导航决策输出
- 统一可视化界面

作者: Zhuo RM Team
日期: 2025-10-17
"""

import cv2
import numpy as np
import time
import onnxruntime as ort
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import threading
import struct

from berxel_camera import BerxelCamera
from depth_slam_obstacle import DepthSLAMObstacleDetector

# 在启动时打印环境信息
def _print_runtime_env():
    try:
        import onnxruntime as ort
        print('onnxruntime version:', ort.__version__)
        print('Available providers:', ort.get_available_providers())
    except Exception as e:
        print('onnxruntime not fully available:', e)

_print_runtime_env()

# 加载 ONNX Runtime 会话
print("Initializing ONNX Runtime...")
available_providers = ort.get_available_providers()
print(f"Available ONNX Runtime providers: {available_providers}")

if 'ROCMExecutionProvider' in available_providers:
    providers = ['ROCMExecutionProvider', 'CPUExecutionProvider']
    print("✅ Using ROCMExecutionProvider (AMD GPU acceleration)")
elif 'CUDAExecutionProvider' in available_providers:
    providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
    print("⚠️ CUDA found but you have AMD 780M - using CPU instead")
    providers = ['CPUExecutionProvider']
else:
    providers = ['CPUExecutionProvider']
    print("ℹ️ Using CPUExecutionProvider (optimized for x86)")

# 加载 ONNX 模型
onnx_model_path = 'yolov8n.onnx'
session = ort.InferenceSession(onnx_model_path, providers=providers)

# 获取输入输出信息
input_name = session.get_inputs()[0].name
output_names = [o.name for o in session.get_outputs()]
print(f"Model input: {input_name}, outputs: {output_names}")

# YOLO 推理参数配置
YOLO_CONF_THRESHOLD = 0.40
YOLO_IOU_THRESHOLD = 0.45
YOLO_MAX_DETECTIONS = 100

# 性能优化参数
SKIP_FRAMES = 1
YOLO_INPUT_SIZE = 416
DISPLAY_SCALE = 0.5
TARGET_FPS = 30
DEPTH_PROCESS_INTERVAL = 3

# 深度平滑参数
DEPTH_EMA_ALPHA = 0.25
DEPTH_VIS_KEEP_FRAMES = 5

# SLAM参数
SLAM_DEPTH_THRESHOLD_NEAR = 0.8  # 近距离障碍物阈值(米)
SLAM_DEPTH_THRESHOLD_FAR = 5.0   # 远距离阈值(米)

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.lidar_callback,
            10)
        self.lidar_data = None
        self.lock = threading.Lock()
        self.point_count = 0
        print("LiDAR Monitor initialized, waiting for data...")

    def lidar_callback(self, msg):
        with self.lock:
            self.lidar_data = msg
            self.point_count = msg.width * msg.height
            # print(f"Received LiDAR data: {self.point_count} points")

    def get_latest_cloud(self):
        with self.lock:
            return self.lidar_data, self.point_count

def start_lidar_monitor():
    rclpy.init()
    lidar_monitor = LidarMonitor()
    thread = threading.Thread(target=rclpy.spin, args=(lidar_monitor,), daemon=True)
    thread.start()
    return lidar_monitor

def initialize_camera():
    try:
        camera = BerxelCamera()
        if not camera.initialize():
            raise Exception("无法初始化Berxel相机")
        return camera
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise


def is_valid_person(box, confidence, frame_height, frame_width):
    """检查检测框是否符合人形特征"""
    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1
    
    if width <= 0 or height <= 0:
        return False
    
    aspect_ratio = height / width
    
    if aspect_ratio < 0.3 or aspect_ratio > 8.0:
        return False
    
    area = width * height
    frame_area = frame_height * frame_width
    area_ratio = area / frame_area
    
    if area_ratio < 0.002 or area_ratio > 0.98:
        return False
    
    if width < 40 or height < 60:
        return False
    
    if width > 1900 or height > 1100:
        return False
        
    if confidence < 0.45:
        return False
    
    return True


def run_onnx_inference(frame, session, input_name, input_size=416):
    """使用 ONNX Runtime 运行 YOLOv8 推理"""
    img = cv2.resize(frame, (input_size, input_size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)
    
    outputs = session.run(None, {input_name: img})
    predictions = outputs[0][0].T
    
    boxes = predictions[:, :4]
    scores = predictions[:, 4:]
    
    class_ids = np.argmax(scores, axis=1)
    confidences = np.max(scores, axis=1)
    
    mask = confidences > YOLO_CONF_THRESHOLD
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]
    
    person_mask = class_ids == 0
    boxes = boxes[person_mask]
    confidences = confidences[person_mask]
    
    x_center, y_center, width, height = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = x_center - width / 2
    y1 = y_center - height / 2
    x2 = x_center + width / 2
    y2 = y_center + height / 2
    
    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(),
            confidences.tolist(),
            YOLO_CONF_THRESHOLD,
            YOLO_IOU_THRESHOLD
        )
        
        if len(indices) > 0:
            if isinstance(indices, tuple):
                indices = list(indices)
            if isinstance(indices, list) and len(indices) > 0:
                indices = np.array(indices).flatten()
            x1 = x1[indices]
            y1 = y1[indices]
            x2 = x2[indices]
            y2 = y2[indices]
            confidences = confidences[indices]
            
            detections = np.column_stack([x1, y1, x2, y2, confidences])
            return detections
    
    return np.array([])


def get_depth_at_point(depth_map, x, y, window_size=5):
    """获取指定点的深度值"""
    if depth_map is None:
        return None
    
    h, w = depth_map.shape[:2]
    
    half_win = window_size // 2
    y_min = max(0, y - half_win)
    y_max = min(h, y + half_win + 1)
    x_min = max(0, x - half_win)
    x_max = min(w, x + half_win + 1)
    
    region = depth_map[y_min:y_max, x_min:x_max]
    valid_depths = region[region > 0]
    
    if len(valid_depths) == 0:
        return None
    
    depth_value = np.median(valid_depths)
    
    if depth_value < 3000 or depth_value > 150000:
        return None
    
    return float(depth_value)


def pointcloud2_to_birdview(cloud_msg, width=400, height=400, resolution=0.02, debug_counter=[0]):
    """将 PointCloud2 转换为鸟瞰图"""
    if cloud_msg is None:
        birdview = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(birdview, "No LiDAR Data", (10, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return birdview
    
    # 创建鸟瞰图画布
    birdview = np.zeros((height, width, 3), dtype=np.uint8)
    
    try:
        # 解析 PointCloud2 数据
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        
        debug_counter[0] += 1
        
        # 根据 point_step 自动识别格式
        points = []
        total_points = len(data) // point_step
        
        for i in range(0, min(len(data), total_points * point_step), point_step):
            if i + 12 <= len(data):  # 至少需要 x,y,z
                try:
                    x = struct.unpack('f', bytes(data[i:i+4]))[0]
                    y = struct.unpack('f', bytes(data[i+4:i+8]))[0]
                    z = struct.unpack('f', bytes(data[i+8:i+12]))[0]
                    
                    # 过滤无效点和超出范围的点
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(x) or np.isinf(y) or np.isinf(z)):
                        if abs(x) < 20 and abs(y) < 20 and z > -5 and z < 5:  # 扩大范围
                            points.append((x, y, z))
                except:
                    continue
        
        # 每 30 帧打印一次调试信息
        if debug_counter[0] % 30 == 0:
            print(f"LiDAR Debug: Total {total_points} points, Valid {len(points)} points, Step {point_step}")
        
        # 绘制点云到鸟瞰图
        for x, y, z in points:
            # 转换到图像坐标 (中心在图像下方，前方向上)
            img_x = int(width / 2 + y / resolution)  # y对应图像x
            img_y = int(height - x / resolution - 20)  # x对应图像y (前方向上)
            
            if 0 <= img_x < width and 0 <= img_y < height:
                # 根据高度着色
                if z < -0.2:
                    color = (0, 0, 255)  # 低于地面 - 红色
                elif z > 0.5:
                    color = (255, 255, 0)  # 高于半米 - 黄色
                else:
                    color = (0, 255, 0)  # 正常 - 绿色
                cv2.circle(birdview, (img_x, img_y), 2, color, -1)
        
        # 绘制机器人位置（图像下方中心）
        robot_x, robot_y = width//2, height - 10
        cv2.circle(birdview, (robot_x, robot_y), 5, (255, 0, 255), -1)
        cv2.circle(birdview, (robot_x, robot_y), 10, (255, 0, 255), 1)
        
        # 绘制方向指示线
        cv2.arrowedLine(birdview, (robot_x, robot_y), (robot_x, robot_y-30), (255, 0, 255), 2)
        
        # 添加文本信息
        cv2.putText(birdview, f"Points: {len(points)}", (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(birdview, "LiDAR BirdView", (10, height-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(birdview, f"{resolution*width:.1f}m x {resolution*height:.1f}m", (width-150, height-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
    except Exception as e:
        if debug_counter[0] % 30 == 1:
            print(f"点云转换错误: {e}")
            import traceback
            traceback.print_exc()
        cv2.putText(birdview, "PointCloud Error", (10, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    return birdview


def main():
    """主函数：集成人员检测与SLAM导航"""
    cap = None
    try:
        print("\n" + "="*70)
        print("🤖 人员检测 + SLAM导航集成系统")
        print("="*70)
        
        # 初始化相机
        print("\n[1/3] 初始化Berxel相机...")
        cap = initialize_camera()
        print("✅ 相机初始化成功")

        # 初始化LiDAR监听
        print("\n[1.5/3] 初始化LiDAR监听...")
        lidar_monitor = None
        try:
            lidar_monitor = start_lidar_monitor()
            print("✅ LiDAR监听已启动")
        except Exception as e:
            print(f"⚠️ LiDAR监听启动失败: {e}")
        
        # 初始化SLAM检测器
        print("\n[2/3] 初始化SLAM避障系统...")
        slam = DepthSLAMObstacleDetector(
            depth_threshold_near=SLAM_DEPTH_THRESHOLD_NEAR,
            depth_threshold_far=SLAM_DEPTH_THRESHOLD_FAR,
            min_navigable_area=800
        )
        print("✅ SLAM系统初始化成功")
        
        print("\n[3/3] 系统就绪！")
        print("\n控制键:")
        print("  - 'q': 退出程序")
        print("  - 's': 保存截图")
        print("  - 'p': 暂停/继续")
        print("  - 'd': 切换SLAM显示")
        print("="*70 + "\n")
        
        # 性能统计变量
        frame_count = 0
        last_detections = []
        fps_start_time = time.time()
        fps_frame_count = 0
        current_fps = 0
        
        # SLAM控制变量
        show_slam = True
        paused = False
        screenshot_count = 0
        
        # 深度平滑缓存
        last_depth_color = None
        last_depth_raw = None
        ema_depth_min = None
        ema_depth_max = None
        last_depth_keep_counter = 0
        
        while True:
            if paused:
                key = cv2.waitKey(100) & 0xFF
                if key == ord('p'):
                    paused = False
                    print("\n▶️  继续运行")
                elif key == ord('q'):
                    break
                continue
            
            loop_start = time.time()
            
            # 获取彩色图像和深度图
            frame = cap.get_frame()
            depth = cap.get_depth()
            
            if frame is None:
                print("无法获取图像帧")
                continue
            
            h, w = frame.shape[:2]
            
            # 创建显示画布（左：检测 | 中：深度/SLAM | 右：点云）
            display = np.zeros((h, w*3, 3), dtype=np.uint8)
            display[:, :w] = frame.copy()
            
            frame_count += 1
            fps_frame_count += 1
            
            # ========== 深度图处理与SLAM ==========
            depth_resized = None
            slam_info = None
            obstacle_mask = None
            
            if depth is not None and depth.size > 0 and frame_count % DEPTH_PROCESS_INTERVAL == 0:
                try:
                    # 缩放深度图到彩色图尺寸
                    depth_resized = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
                    
                    # 转换为米（P100R需要除以17）
                    depth_meters = depth_resized / 17000.0
                    
                    # SLAM处理
                    if show_slam:
                        obstacle_mask, slam_info = slam.process_depth_frame(depth_meters, frame)
                    
                    # 深度可视化（带平滑）
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
                            
                            # SLAM可视化叠加
                            if show_slam and obstacle_mask is not None:
                                depth_color = slam.visualize(depth_meters, obstacle_mask, 
                                                            slam_info, depth_color)
                            
                            display[:, w:w*2] = depth_color
                            last_depth_color = depth_color.copy()
                            last_depth_raw = depth_resized.copy()
                            last_depth_keep_counter = 0
                    else:
                        display[:, w:w*2] = 0
                        
                except Exception as e:
                    print(f"深度/SLAM处理错误: {e}")
                    display[:, w:] = 0
            else:
                # 重用上一帧深度可视化
                if last_depth_color is not None and last_depth_keep_counter < DEPTH_VIS_KEEP_FRAMES:
                    display[:, w:w*2] = last_depth_color
                    last_depth_keep_counter += 1
                    depth_resized = last_depth_raw
                else:
                    display[:, w:w*2] = 0
            
            # ========== LiDAR 点云显示 ==========
            if lidar_monitor is not None:
                cloud_msg, point_count = lidar_monitor.get_latest_cloud()
                lidar_birdview = pointcloud2_to_birdview(cloud_msg, width=w, height=h)
                display[:, w*2:w*3] = lidar_birdview
            else:
                display[:, w*2:w*3] = 0
                cv2.putText(display, "No LiDAR", (w*2+10, h//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            
            # ========== YOLO人员检测 ==========
            try:
                if frame_count % (SKIP_FRAMES + 1) == 0:
                    detections = run_onnx_inference(frame, session, input_name, YOLO_INPUT_SIZE)
                    
                    scale_x = w / YOLO_INPUT_SIZE
                    scale_y = h / YOLO_INPUT_SIZE
                    
                    last_detections = []
                    for det in detections:
                        x1 = int(det[0] * scale_x)
                        y1 = int(det[1] * scale_y)
                        x2 = int(det[2] * scale_x)
                        y2 = int(det[3] * scale_y)
                        confidence = float(det[4])
                        
                        if is_valid_person((x1, y1, x2, y2), confidence, h, w):
                            last_detections.append((x1, y1, x2, y2, confidence))
                
                # 绘制检测结果
                for detection in last_detections:
                    x1, y1, x2, y2, confidence = detection
                    try:
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # 获取深度值
                        depth_source = depth_resized if depth_resized is not None else last_depth_raw
                        depth_value = get_depth_at_point(depth_source, center_x, center_y, window_size=7)
                        
                        label = f"Person {confidence:.2f}"
                        if depth_value is not None:
                            distance_m = depth_value / 17000.0
                            label += f" {distance_m:.2f}m"
                        else:
                            label += " (no depth)"
                        
                        # 在左侧绘制
                        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display, label, (x1, y1 - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.drawMarker(display, (center_x, center_y),
                                     (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                        
                        # 在中间列也绘制
                        cv2.rectangle(display, (w+x1, y1), (w+x2, y2), (0, 255, 0), 2)
                        cv2.drawMarker(display, (w+center_x, center_y),
                                     (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                    except Exception as e:
                        print(f"处理检测框时发生错误: {e}")
                        continue
                
                # 计算FPS
                if fps_frame_count >= 30:
                    elapsed = time.time() - fps_start_time
                    current_fps = fps_frame_count / elapsed
                    fps_start_time = time.time()
                    fps_frame_count = 0
                
                # 显示系统信息
                info_y = 30
                cv2.putText(display, f"FPS: {current_fps:.1f}", (10, info_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                if show_slam and slam_info:
                    info_y += 35
                    direction = slam_info['suggested_direction']
                    cv2.putText(display, f"Nav: {direction.upper()}", (10, info_y),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    info_y += 30
                    cv2.putText(display, f"People: {len(last_detections)}", (10, info_y),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 显示提示
                cv2.putText(display, "Detection", (w//2-80, h-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(display, "SLAM Navigation" if show_slam else "Depth Only", 
                          (w + w//2-100, h-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(display, "LiDAR PointCloud", 
                          (w*2 + w//2-100, h-20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # 缩放显示
                display_h = int(h * DISPLAY_SCALE)
                display_w = int(w * 3 * DISPLAY_SCALE)
                display_resized = cv2.resize(display, (display_w, display_h))
                
                cv2.imshow('Person Detection + SLAM Navigation', display_resized)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    screenshot_count += 1
                    filename = f'screenshot_{screenshot_count}.png'
                    cv2.imwrite(filename, display)
                    print(f"\n📸 截图保存: {filename}")
                elif key == ord('p'):
                    paused = True
                    print("\n⏸️  已暂停（按'p'继续）")
                elif key == ord('d'):
                    show_slam = not show_slam
                    status = "开启" if show_slam else "关闭"
                    print(f"\n🗺️  SLAM显示: {status}")
                
                # 帧率控制
                elapsed = time.time() - loop_start
                target_delay = 1.0 / TARGET_FPS
                if elapsed < target_delay:
                    time.sleep(target_delay - elapsed)
                    
            except Exception as e:
                print(f"主循环错误: {e}")
                continue
                
    except Exception as e:
        print(f"系统错误: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # 清理资源
        print("\n" + "="*70)
        print("🧹 清理资源...")
        
        cv2.destroyAllWindows()
        if cap:
            cap.release()
        
        # 显示统计信息
        if 'slam' in locals():
            stats = slam.get_statistics()
            print(f"\n📊 运行统计:")
            print(f"  - 总帧数: {frame_count}")
            print(f"  - SLAM处理帧数: {stats['total_frames']}")
            print(f"  - 平均FPS: {current_fps:.1f}")
            if stats['total_frames'] > 0:
                print(f"  - SLAM平均处理时间: {stats['avg_processing_time']*1000:.1f}ms")
        
        print("\n✅ 系统已关闭")
        print("="*70)


if __name__ == "__main__":
    main()
