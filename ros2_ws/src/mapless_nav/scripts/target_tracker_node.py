#!/usr/bin/env python3
"""
Target Tracker Node with YOLO Detection and ByteTrack

This node combines YOLO object detection with ByteTrack multi-object tracking
to provide robust person tracking for the mapless navigation system.

Features:
- YOLO detection via ONNX Runtime (supports ROCm/CUDA/CPU)
- ByteTrack multi-object tracking
- 3D position estimation using depth information
- Track persistence and re-identification
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

import os
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Try to import ONNX Runtime
try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("WARNING: onnxruntime not installed, detection will be disabled")


@dataclass
class Detection:
    """Single detection result"""
    bbox: np.ndarray  # [x1, y1, x2, y2]
    confidence: float
    class_id: int
    class_name: str


@dataclass
class Track:
    """Tracked object with history"""
    track_id: int
    bbox: np.ndarray
    confidence: float
    class_id: int
    class_name: str
    position_3d: Optional[np.ndarray] = None  # [x, y, z] in camera frame
    velocity: Optional[np.ndarray] = None  # [vx, vy, vz]
    age: int = 0
    hits: int = 0
    time_since_update: int = 0
    history: deque = None
    
    def __post_init__(self):
        if self.history is None:
            self.history = deque(maxlen=30)


class ByteTracker:
    """
    Simplified ByteTrack implementation for multi-object tracking
    
    Based on: https://arxiv.org/abs/2110.06864
    """
    
    def __init__(self, 
                 track_thresh: float = 0.5,
                 track_buffer: int = 30,
                 match_thresh: float = 0.8,
                 min_box_area: int = 100):
        self.track_thresh = track_thresh
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh
        self.min_box_area = min_box_area
        
        self.tracks: List[Track] = []
        self.lost_tracks: List[Track] = []
        self.next_id = 1
        self.frame_id = 0
    
    def update(self, detections: List[Detection]) -> List[Track]:
        """Update tracks with new detections"""
        self.frame_id += 1
        
        # Filter small boxes
        detections = [d for d in detections 
                     if (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]) > self.min_box_area]
        
        # Split detections by confidence
        high_dets = [d for d in detections if d.confidence >= self.track_thresh]
        low_dets = [d for d in detections if d.confidence < self.track_thresh]
        
        # Update existing tracks
        unmatched_tracks = []
        for track in self.tracks:
            track.time_since_update += 1
            unmatched_tracks.append(track)
        
        # Match high confidence detections with tracks
        matched_tracks, matched_dets, unmatched_tracks, unmatched_high_dets = \
            self._match(unmatched_tracks, high_dets)
        
        # Update matched tracks
        for track, det in zip(matched_tracks, matched_dets):
            track.bbox = det.bbox
            track.confidence = det.confidence
            track.time_since_update = 0
            track.hits += 1
            track.age += 1
            track.history.append(det.bbox.copy())
        
        # Try to match remaining tracks with low confidence detections
        matched_tracks2, matched_dets2, unmatched_tracks, _ = \
            self._match(unmatched_tracks, low_dets)
        
        for track, det in zip(matched_tracks2, matched_dets2):
            track.bbox = det.bbox
            track.confidence = det.confidence
            track.time_since_update = 0
            track.age += 1
            track.history.append(det.bbox.copy())
        
        # Create new tracks for unmatched high confidence detections
        for det in unmatched_high_dets:
            new_track = Track(
                track_id=self.next_id,
                bbox=det.bbox,
                confidence=det.confidence,
                class_id=det.class_id,
                class_name=det.class_name,
                hits=1,
                age=1
            )
            self.next_id += 1
            self.tracks.append(new_track)
        
        # Move lost tracks
        for track in unmatched_tracks:
            if track.time_since_update > self.track_buffer:
                self.tracks.remove(track)
                self.lost_tracks.append(track)
        
        # Clean old lost tracks
        self.lost_tracks = [t for t in self.lost_tracks 
                           if self.frame_id - t.age < self.track_buffer * 2]
        
        # Return active tracks
        return [t for t in self.tracks if t.hits >= 3 or t.time_since_update == 0]
    
    def _match(self, tracks: List[Track], detections: List[Detection]) \
            -> Tuple[List[Track], List[Detection], List[Track], List[Detection]]:
        """Match tracks with detections using IoU"""
        if not tracks or not detections:
            return [], [], tracks, detections
        
        # Compute IoU matrix
        iou_matrix = np.zeros((len(tracks), len(detections)))
        for i, track in enumerate(tracks):
            for j, det in enumerate(detections):
                iou_matrix[i, j] = self._iou(track.bbox, det.bbox)
        
        # Greedy matching
        matched_tracks = []
        matched_dets = []
        unmatched_tracks = list(tracks)
        unmatched_dets = list(detections)
        
        while True:
            if iou_matrix.size == 0:
                break
            
            max_iou = iou_matrix.max()
            if max_iou < 1 - self.match_thresh:
                break
            
            idx = np.unravel_index(iou_matrix.argmax(), iou_matrix.shape)
            track_idx, det_idx = idx
            
            matched_tracks.append(unmatched_tracks[track_idx])
            matched_dets.append(unmatched_dets[det_idx])
            
            # Remove matched from matrix
            iou_matrix = np.delete(iou_matrix, track_idx, axis=0)
            iou_matrix = np.delete(iou_matrix, det_idx, axis=1)
            
            unmatched_tracks.pop(track_idx)
            unmatched_dets.pop(det_idx)
        
        return matched_tracks, matched_dets, unmatched_tracks, unmatched_dets
    
    @staticmethod
    def _iou(box1: np.ndarray, box2: np.ndarray) -> float:
        """Compute IoU between two boxes"""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        
        return inter / (area1 + area2 - inter + 1e-6)


class YOLODetector:
    """YOLO detector using ONNX Runtime"""
    
    # COCO class names
    COCO_CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
        'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
        'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
        'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
        'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
        'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
        'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
        'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
        'toothbrush'
    ]
    
    def __init__(self, 
                 model_path: str,
                 input_size: int = 640,
                 conf_thresh: float = 0.25,
                 nms_thresh: float = 0.45,
                 target_classes: List[int] = None):
        """
        Initialize YOLO detector
        
        Args:
            model_path: Path to ONNX model
            input_size: Model input size
            conf_thresh: Confidence threshold
            nms_thresh: NMS threshold
            target_classes: List of class IDs to detect (None = all)
        """
        self.input_size = input_size
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.target_classes = target_classes or [0]  # Default: person only
        
        if not ONNX_AVAILABLE:
            raise RuntimeError("ONNX Runtime not available")
        
        # Select best available provider
        available = ort.get_available_providers()

        providers = []
        if 'ROCMExecutionProvider' in available and os.access('/dev/kfd', os.R_OK | os.W_OK):
            providers.append('ROCMExecutionProvider')
        if 'CUDAExecutionProvider' in available and any(
                os.access(f'/dev/nvidia{i}', os.R_OK) for i in range(4)
                if os.path.exists(f'/dev/nvidia{i}')):
            providers.append('CUDAExecutionProvider')
        providers.append('CPUExecutionProvider')

        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        
        # Get actual input size from model
        input_shape = self.session.get_inputs()[0].shape
        if len(input_shape) == 4:
            self.input_size = input_shape[2]  # Assuming NCHW format
        
        print(f"YOLO initialized with providers: {self.session.get_providers()}")
        print(f"Input size: {self.input_size}")
    
    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run detection on image
        
        Args:
            image: BGR image (H, W, 3)
            
        Returns:
            List of Detection objects
        """
        h, w = image.shape[:2]
        
        # Preprocess
        input_tensor, ratio, pad = self._preprocess(image)
        
        # Inference
        outputs = self.session.run(None, {self.input_name: input_tensor})
        
        # Postprocess
        detections = self._postprocess(outputs[0], ratio, pad, (h, w))
        
        return detections
    
    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """Preprocess image for YOLO"""
        h, w = image.shape[:2]
        
        # Calculate resize ratio
        ratio = min(self.input_size / h, self.input_size / w)
        new_h, new_w = int(h * ratio), int(w * ratio)
        
        # Resize
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Pad to square
        pad_h = (self.input_size - new_h) // 2
        pad_w = (self.input_size - new_w) // 2
        
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        padded[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
        
        # Convert to tensor
        tensor = padded.astype(np.float32) / 255.0
        tensor = tensor.transpose(2, 0, 1)  # HWC -> CHW
        tensor = np.expand_dims(tensor, 0)  # Add batch dim
        
        return tensor, ratio, (pad_w, pad_h)
    
    def _postprocess(self, output: np.ndarray, ratio: float, 
                     pad: Tuple[int, int], orig_size: Tuple[int, int]) -> List[Detection]:
        """Postprocess YOLO output"""
        # Output shape: (1, num_classes + 4, num_boxes) for YOLOv8
        # or (1, num_boxes, num_classes + 4) for older versions
        
        if output.shape[1] < output.shape[2]:
            output = output.transpose(0, 2, 1)  # (1, boxes, features)
        
        output = output[0]  # Remove batch dim
        
        # Split into boxes and scores
        if output.shape[1] == 84:  # YOLOv8 format (4 + 80 classes)
            boxes = output[:, :4]
            scores = output[:, 4:]
        else:
            boxes = output[:, :4]
            scores = output[:, 4:5] * output[:, 5:]  # obj_conf * class_conf
        
        # Get best class for each box
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        
        # Filter by confidence and target classes
        mask = confidences > self.conf_thresh
        if self.target_classes:
            class_mask = np.isin(class_ids, self.target_classes)
            mask = mask & class_mask
        
        boxes = boxes[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]
        
        if len(boxes) == 0:
            return []
        
        # Convert from center format to corner format
        # [cx, cy, w, h] -> [x1, y1, x2, y2]
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        
        # Remove padding and rescale
        pad_w, pad_h = pad
        x1 = (x1 - pad_w) / ratio
        y1 = (y1 - pad_h) / ratio
        x2 = (x2 - pad_w) / ratio
        y2 = (y2 - pad_h) / ratio
        
        # Clip to image bounds
        h, w = orig_size
        x1 = np.clip(x1, 0, w)
        y1 = np.clip(y1, 0, h)
        x2 = np.clip(x2, 0, w)
        y2 = np.clip(y2, 0, h)
        
        boxes = np.stack([x1, y1, x2, y2], axis=1)
        
        # NMS
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), 
            confidences.tolist(),
            self.conf_thresh, 
            self.nms_thresh
        )
        
        if len(indices) == 0:
            return []
        
        indices = indices.flatten()
        
        # Create Detection objects
        detections = []
        for i in indices:
            det = Detection(
                bbox=boxes[i],
                confidence=float(confidences[i]),
                class_id=int(class_ids[i]),
                class_name=self.COCO_CLASSES[class_ids[i]] if class_ids[i] < len(self.COCO_CLASSES) else "unknown"
            )
            detections.append(det)
        
        return detections


class TargetTrackerNode(Node):
    """ROS2 Node for YOLO detection and ByteTrack tracking"""
    
    def __init__(self):
        super().__init__('target_tracker_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '/home/zhuo-skadi/Documents/ros2-robt/yolo12n.onnx')
        self.declare_parameter('input_size', 416)
        self.declare_parameter('conf_thresh', 0.3)
        self.declare_parameter('nms_thresh', 0.45)
        self.declare_parameter('target_classes', [0])  # person
        
        self.declare_parameter('image_topic', '/berxel/color/image_raw')
        self.declare_parameter('depth_topic', '/berxel/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/berxel/depth/camera_info')
        
        self.declare_parameter('track_thresh', 0.5)
        self.declare_parameter('track_buffer', 30)
        self.declare_parameter('match_thresh', 0.8)
        
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('publish_markers', True)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        input_size = self.get_parameter('input_size').value
        conf_thresh = self.get_parameter('conf_thresh').value
        nms_thresh = self.get_parameter('nms_thresh').value
        target_classes = self.get_parameter('target_classes').value
        
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        track_thresh = self.get_parameter('track_thresh').value
        track_buffer = self.get_parameter('track_buffer').value
        match_thresh = self.get_parameter('match_thresh').value
        
        self.publish_visualization = self.get_parameter('publish_visualization').value
        self.publish_markers = self.get_parameter('publish_markers').value
        
        # Initialize detector
        try:
            self.detector = YOLODetector(
                model_path=model_path,
                input_size=input_size,
                conf_thresh=conf_thresh,
                nms_thresh=nms_thresh,
                target_classes=target_classes
            )
            self.detector_enabled = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize detector: {e}")
            self.detector_enabled = False
        
        # Initialize tracker
        self.tracker = ByteTracker(
            track_thresh=track_thresh,
            track_buffer=track_buffer,
            match_thresh=match_thresh
        )
        
        # Camera intrinsics
        self.fx = 460.0
        self.fy = 460.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Latest frames
        self.latest_color = None
        self.latest_depth = None
        
        # QoS Profile
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image, image_topic, self.color_callback, sensor_qos)
        
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, sensor_qos)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, sensor_qos)
        
        # Publishers
        self.tracks_pub = self.create_publisher(
            PoseArray, '/target_tracker/tracks', 10)
        
        self.primary_target_pub = self.create_publisher(
            PoseStamped, '/target_tracker/primary_target', 10)
        
        if self.publish_visualization:
            self.viz_pub = self.create_publisher(
                Image, '/target_tracker/visualization', 10)
        
        if self.publish_markers:
            self.markers_pub = self.create_publisher(
                MarkerArray, '/target_tracker/markers', 10)
        
        # Processing timer
        self.timer = self.create_timer(0.05, self.process_frame)  # 20Hz
        
        # Statistics
        self.frame_count = 0
        self.fps_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info("Target Tracker Node initialized")
        self.get_logger().info(f"  Image topic: {image_topic}")
        self.get_logger().info(f"  Depth topic: {depth_topic}")
        self.get_logger().info(f"  Detector enabled: {self.detector_enabled}")
    
    def color_callback(self, msg: Image):
        """Store latest color frame"""
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")
    
    def depth_callback(self, msg: Image):
        """Store latest depth frame"""
        try:
            if msg.encoding == '16UC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics"""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True
    
    def process_frame(self):
        """Main processing loop"""
        if self.latest_color is None or not self.detector_enabled:
            return
        
        frame = self.latest_color.copy()
        
        # Run detection
        detections = self.detector.detect(frame)
        
        # Run tracking
        tracks = self.tracker.update(detections)
        
        # Estimate 3D positions
        if self.latest_depth is not None:
            for track in tracks:
                track.position_3d = self.estimate_3d_position(track.bbox)
        
        # Publish results
        self.publish_tracks(tracks)
        
        # Publish primary target (closest person)
        if tracks:
            primary = self.select_primary_target(tracks)
            if primary:
                self.publish_primary_target(primary)
        
        # Publish visualization
        if self.publish_visualization:
            self.publish_viz(frame, tracks)
        
        # Publish markers
        if self.publish_markers:
            self.publish_track_markers(tracks)
        
        # Update FPS
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            now = time.time()
            self.fps = 30.0 / (now - self.fps_time)
            self.fps_time = now
            self.get_logger().debug(f"FPS: {self.fps:.1f}, Tracks: {len(tracks)}")
    
    def estimate_3d_position(self, bbox: np.ndarray) -> Optional[np.ndarray]:
        """Estimate 3D position from bbox and depth"""
        if self.latest_depth is None:
            return None
        
        # Get bbox center
        cx = int((bbox[0] + bbox[2]) / 2)
        cy = int((bbox[1] + bbox[3]) / 2)
        
        # Get depth at center (with small window averaging)
        h, w = self.latest_depth.shape[:2]
        cx = np.clip(cx, 5, w - 5)
        cy = np.clip(cy, 5, h - 5)
        
        window = self.latest_depth[cy-5:cy+5, cx-5:cx+5]
        
        # Handle different depth formats
        if self.latest_depth.dtype == np.uint16:
            valid = window[window > 0]
            if len(valid) == 0:
                return None
            depth = np.median(valid) * 0.001  # mm to meters
        else:
            valid = window[np.isfinite(window) & (window > 0)]
            if len(valid) == 0:
                return None
            depth = np.median(valid)
        
        if depth < 0.3 or depth > 10.0:
            return None
        
        # Deproject to 3D
        x = (cx - self.cx) * depth / self.fx
        y = (cy - self.cy) * depth / self.fy
        z = depth
        
        return np.array([x, y, z])
    
    def select_primary_target(self, tracks: List[Track]) -> Optional[Track]:
        """Select primary target (closest person)"""
        valid_tracks = [t for t in tracks if t.position_3d is not None 
                       and t.class_name == 'person']
        
        if not valid_tracks:
            return None
        
        # Select closest
        closest = min(valid_tracks, key=lambda t: t.position_3d[2])
        return closest
    
    def publish_tracks(self, tracks: List[Track]):
        """Publish all tracks as PoseArray"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'berxel_depth_optical_frame'
        
        for track in tracks:
            if track.position_3d is not None:
                from geometry_msgs.msg import Pose
                pose = Pose()
                pose.position.x = float(track.position_3d[0])
                pose.position.y = float(track.position_3d[1])
                pose.position.z = float(track.position_3d[2])
                pose.orientation.w = 1.0
                msg.poses.append(pose)
        
        self.tracks_pub.publish(msg)
    
    def publish_primary_target(self, track: Track):
        """Publish primary target"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'berxel_depth_optical_frame'
        
        msg.pose.position.x = float(track.position_3d[0])
        msg.pose.position.y = float(track.position_3d[1])
        msg.pose.position.z = float(track.position_3d[2])
        msg.pose.orientation.w = 1.0
        
        self.primary_target_pub.publish(msg)
    
    def publish_viz(self, frame: np.ndarray, tracks: List[Track]):
        """Publish visualization image"""
        viz = frame.copy()
        
        for track in tracks:
            # Draw bbox
            x1, y1, x2, y2 = track.bbox.astype(int)
            color = (0, 255, 0) if track.class_name == 'person' else (255, 0, 0)
            cv2.rectangle(viz, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"ID:{track.track_id} {track.class_name}"
            if track.position_3d is not None:
                label += f" {track.position_3d[2]:.2f}m"
            cv2.putText(viz, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw FPS
        cv2.putText(viz, f"FPS: {self.fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        msg = self.bridge.cv2_to_imgmsg(viz, 'bgr8')
        self.viz_pub.publish(msg)
    
    def publish_track_markers(self, tracks: List[Track]):
        """Publish RViz markers for tracks"""
        markers = MarkerArray()
        
        for i, track in enumerate(tracks):
            if track.position_3d is None:
                continue
            
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'berxel_depth_optical_frame'
            marker.ns = 'tracks'
            marker.id = track.track_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(track.position_3d[0])
            marker.pose.position.y = float(track.position_3d[1])
            marker.pose.position.z = float(track.position_3d[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.7
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 200ms
            
            markers.markers.append(marker)
            
            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'track_labels'
            text_marker.id = track.track_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(track.position_3d[0])
            text_marker.pose.position.y = float(track.position_3d[1])
            text_marker.pose.position.z = float(track.position_3d[2]) + 1.0
            
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"ID:{track.track_id}"
            text_marker.lifetime = marker.lifetime
            
            markers.markers.append(text_marker)
        
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
