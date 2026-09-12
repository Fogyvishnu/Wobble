#!/usr/bin/env python3
"""
Project Wobble: Real-Time YOLOv8 Deep Learning Perception & Object Tracking Node
Author: Antigravity Agent for Project Wobble

Description:
    Runs real-time deep learning object detection on camera streams (/camera/image_raw).
    Features:
      1. YOLOv8-Nano inference with automated fallback (Ultralytics -> ONNXRuntime -> OpenCV DNN).
      2. 3D Spatial Geometry & Depth Estimation using pinhole camera model.
      3. Cybernetic HUD visual overlay with class bounding boxes, tracking reticles, and live telemetry.
      4. JSON object telemetry publisher (/camera/yolo_detections).
      5. Autonomous target-following & person-tracking controller with active squat interaction.
      6. Forward collision avoidance warnings (/wobble/collision_warning).
"""

import os
import sys
import time
import json
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Bool
from cv_bridge import CvBridge

# Reference real-world heights (meters) for spatial depth estimation
CLASS_REAL_HEIGHTS = {
    'person': 1.70,
    'bicycle': 1.05,
    'car': 1.50,
    'motorcycle': 1.10,
    'airplane': 4.00,
    'bus': 3.20,
    'train': 3.80,
    'truck': 2.80,
    'boat': 2.00,
    'traffic light': 0.80,
    'fire hydrant': 0.70,
    'stop sign': 0.75,
    'bench': 0.80,
    'bird': 0.20,
    'cat': 0.25,
    'dog': 0.50,
    'horse': 1.60,
    'sheep': 0.80,
    'cow': 1.40,
    'backpack': 0.45,
    'umbrella': 0.70,
    'handbag': 0.35,
    'suitcase': 0.65,
    'sports ball': 0.22,
    'bottle': 0.26,
    'cup': 0.12,
    'chair': 0.85,
    'couch': 0.85,
    'potted plant': 0.50,
    'bed': 0.70,
    'tv': 0.60,
    'laptop': 0.25,
    'cell phone': 0.15,
}
DEFAULT_OBJECT_HEIGHT = 0.50  # Generic obstacle height (meters)


class WobbleYOLODetector(Node):
    """Real-Time YOLOv8 Perception & Cybernetic HUD Node for Wobble."""

    def __init__(self):
        super().__init__('wobble_yolo_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/camera/yolo_annotated')
        self.declare_parameter('detections_topic', '/camera/yolo_detections')
        self.declare_parameter('enable_tracking', False)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('target_distance', 1.20)  # Desired standoff distance (meters)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('horizontal_fov_deg', 80.0)

        # Retrieve parameter values
        self.conf_thresh = float(self.get_parameter('conf_threshold').value)
        self.iou_thresh = float(self.get_parameter('iou_threshold').value)
        self.cam_topic = str(self.get_parameter('camera_topic').value)
        self.annotated_topic = str(self.get_parameter('annotated_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.enable_tracking = bool(self.get_parameter('enable_tracking').value)
        self.target_class = str(self.get_parameter('target_class').value).lower()
        self.target_dist = float(self.get_parameter('target_distance').value)
        self.max_lin_spd = float(self.get_parameter('max_linear_speed').value)
        self.max_ang_spd = float(self.get_parameter('max_angular_speed').value)
        self.hfov_rad = math.radians(float(self.get_parameter('horizontal_fov_deg').value))

        # CV Bridge
        self.bridge = CvBridge()

        # Model Loader
        self.backend = None
        self.model = None
        self.init_model()

        # Publishers
        self.pub_annotated = self.create_publisher(Image, self.annotated_topic, 10)
        self.pub_detections = self.create_publisher(String, self.detections_topic, 10)
        self.pub_warning = self.create_publisher(Bool, '/wobble/collision_warning', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_squat = self.create_publisher(Float64, '/cmd_squat', 10)

        # Subscribers
        self.sub_cam = self.create_subscription(
            Image, self.cam_topic, self.image_callback, 10)
        self.sub_toggle_track = self.create_subscription(
            Bool, '/wobble/toggle_yolo_tracking', self.toggle_tracking_callback, 10)

        # FPS & Profiling
        self.frame_count = 0
        self.fps = 0.0
        self.last_time = time.time()
        self.last_bearing_err = 0.0

        self.get_logger().info(
            f"WobbleYOLODetector ready. Backend: [{self.backend}]. Listening on: {self.cam_topic}"
        )

    def init_model(self):
        """Initializes YOLOv8 with automatic fallback across backends."""
        custom_path = self.get_parameter('model_path').value

        # Search locations for pre-downloaded weights
        candidate_paths = []
        if custom_path:
            candidate_paths.append(custom_path)
        
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        candidate_paths.extend([
            os.path.join(ws_dir, "src", "wobble_control", "models", "yolov8n.pt"),
            os.path.join(ws_dir, "src", "wobble_control", "models", "yolov8n.onnx"),
            os.path.join(ws_dir, "yolov8n.pt"),
            os.path.join(ws_dir, "yolov8n.onnx"),
            "yolov8n.pt",
            "yolov8n.onnx"
        ])

        # Backend 1: Ultralytics PyTorch
        try:
            from ultralytics import YOLO
            for p in candidate_paths:
                if p.endswith('.pt') and os.path.exists(p):
                    self.get_logger().info(f"Loading Ultralytics YOLOv8 PyTorch model from: {p}")
                    self.model = YOLO(p)
                    self.backend = 'ultralytics_pt'
                    return
            # Fallback to automatic download via ultralytics
            self.get_logger().info("Loading default 'yolov8n.pt' via Ultralytics...")
            self.model = YOLO('yolov8n.pt')
            self.backend = 'ultralytics_pt'
            return
        except Exception as e:
            self.get_logger().warn(f"Ultralytics PyTorch backend unavailable ({e}). Falling back to ONNX...")

        # Backend 2: ONNXRuntime
        try:
            import onnxruntime as ort
            for p in candidate_paths:
                if p.endswith('.onnx') and os.path.exists(p):
                    self.get_logger().info(f"Loading ONNXRuntime session from: {p}")
                    self.model = ort.InferenceSession(p)
                    self.backend = 'onnxruntime'
                    return
        except Exception as e:
            self.get_logger().warn(f"ONNXRuntime backend unavailable ({e}). Falling back to OpenCV DNN...")

        # Backend 3: OpenCV DNN
        try:
            for p in candidate_paths:
                if p.endswith('.onnx') and os.path.exists(p):
                    self.get_logger().info(f"Loading OpenCV DNN net from: {p}")
                    self.model = cv2.dnn.readNetFromONNX(p)
                    self.backend = 'opencv_dnn'
                    return
        except Exception as e:
            self.get_logger().error(f"Failed to load any YOLOv8 backend: {e}")

    def toggle_tracking_callback(self, msg: Bool):
        self.enable_tracking = msg.data
        mode = "ENABLED" if self.enable_tracking else "DISABLED"
        self.get_logger().info(f"YOLO Autonomous Target Tracking [{mode}]")

    def image_callback(self, msg: Image):
        """Processes incoming camera frame with YOLOv8."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        h, w = frame.shape[:2]
        now = time.time()
        self.frame_count += 1
        dt = now - self.last_time
        if dt >= 1.0:
            self.fps = self.frame_count / dt
            self.frame_count = 0
            self.last_time = now

        # Pinhole Camera Geometry
        # f = (W / 2) / tan(HFOV / 2)
        fx = (w / 2.0) / math.tan(self.hfov_rad / 2.0)
        fy = fx  # Square pixels assumption
        cx = w / 2.0
        cy = h / 2.0

        detections = []

        # Execute Inference based on Backend
        if self.backend == 'ultralytics_pt':
            detections = self.infer_ultralytics(frame, fx, fy, cx, cy)
        elif self.backend == 'onnxruntime':
            detections = self.infer_onnxruntime(frame, fx, fy, cx, cy)
        elif self.backend == 'opencv_dnn':
            detections = self.infer_opencv_dnn(frame, fx, fy, cx, cy)

        # Check collision risk and tracking
        collision_warning = False
        target_obj = None

        for det in detections:
            # Collision danger check: distance < 0.9m and centered within +/- 18 degrees
            if det['distance_m'] < 0.90 and abs(det['bearing_deg']) < 18.0:
                collision_warning = True

            # Track selected target class (prefer highest confidence or closest)
            if self.enable_tracking and det['class'] == self.target_class:
                if target_obj is None or det['distance_m'] < target_obj['distance_m']:
                    target_obj = det

        # Publish collision warning
        warn_msg = Bool()
        warn_msg.data = collision_warning
        self.pub_warning.publish(warn_msg)

        # Autonomous Tracking Controller
        if self.enable_tracking and target_obj is not None:
            self.execute_tracking_control(target_obj)

        # Publish JSON Detections Telemetry
        json_payload = {
            "timestamp": now,
            "fps": round(self.fps, 1),
            "backend": self.backend,
            "count": len(detections),
            "collision_warning": collision_warning,
            "detections": detections
        }
        json_msg = String()
        json_msg.data = json.dumps(json_payload)
        self.pub_detections.publish(json_msg)

        # Render Cybernetic Visual HUD
        annotated_frame = self.render_cybernetic_hud(
            frame, detections, collision_warning, target_obj
        )

        # Publish Annotated Image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            out_msg.header = msg.header
            self.pub_annotated.publish(out_msg)
        except Exception:
            pass

    def infer_ultralytics(self, frame, fx, fy, cx, cy):
        """Inference using Ultralytics PyTorch API."""
        results = self.model(frame, conf=self.conf_thresh, iou=self.iou_thresh, verbose=False)
        detections = []
        h, w = frame.shape[:2]

        for box in results[0].boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy

            box_w = max(1, x2 - x1)
            box_h = max(1, y2 - y1)
            u_c = (x1 + x2) / 2.0
            v_c = (y1 + y2) / 2.0

            # Distance via pinhole projection: Z = (fy * H_real) / h_px
            real_h = CLASS_REAL_HEIGHTS.get(cls_name, DEFAULT_OBJECT_HEIGHT)
            distance = (fy * real_h) / float(box_h)
            distance = max(0.20, min(30.0, distance))

            # Bearing angle in horizontal plane
            bearing_rad = math.atan2(u_c - cx, fx)
            bearing_deg = math.degrees(bearing_rad)

            detections.append({
                "class": cls_name,
                "class_id": cls_id,
                "conf": round(conf, 2),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "centroid": [round(u_c, 1), round(v_c, 1)],
                "distance_m": round(distance, 2),
                "bearing_deg": round(bearing_deg, 1),
                "bearing_rad": round(bearing_rad, 4),
            })
        return detections

    def infer_opencv_dnn(self, frame, fx, fy, cx, cy):
        """Inference using OpenCV DNN module on ONNX export."""
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 480), swapRB=True, crop=False)
        self.model.setInput(blob)
        preds = self.model.forward()  # Shape: (1, 84, N)
        
        # Squeeze & Transpose to (N, 84)
        preds = np.squeeze(preds, axis=0).T
        boxes = []
        confidences = []
        class_ids = []

        x_scale = w / 640.0
        y_scale = h / 480.0

        for row in preds:
            scores = row[4:]
            class_id = np.argmax(scores)
            conf = scores[class_id]
            if conf >= self.conf_thresh:
                xc, yc, bw, bh = row[0], row[1], row[2], row[3]
                x1 = int((xc - bw / 2.0) * x_scale)
                y1 = int((yc - bh / 2.0) * y_scale)
                box_w = int(bw * x_scale)
                box_h = int(bh * y_scale)
                boxes.append([x1, y1, box_w, box_h])
                confidences.append(float(conf))
                class_ids.append(int(class_id))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.iou_thresh)
        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                x1, y1, bw, bh = boxes[idx]
                x2 = x1 + bw
                y2 = y1 + bh
                conf = confidences[idx]
                cls_id = class_ids[idx]
                cls_name = list(CLASS_REAL_HEIGHTS.keys())[cls_id] if cls_id < len(CLASS_REAL_HEIGHTS) else f"obj_{cls_id}"

                u_c = (x1 + x2) / 2.0
                v_c = (y1 + y2) / 2.0
                real_h = CLASS_REAL_HEIGHTS.get(cls_name, DEFAULT_OBJECT_HEIGHT)
                distance = (fy * real_h) / max(1.0, float(bh))
                distance = max(0.20, min(30.0, distance))

                bearing_rad = math.atan2(u_c - cx, fx)
                bearing_deg = math.degrees(bearing_rad)

                detections.append({
                    "class": cls_name,
                    "class_id": cls_id,
                    "conf": round(conf, 2),
                    "bbox": [x1, y1, x2, y2],
                    "centroid": [round(u_c, 1), round(v_c, 1)],
                    "distance_m": round(distance, 2),
                    "bearing_deg": round(bearing_deg, 1),
                    "bearing_rad": round(bearing_rad, 4),
                })
        return detections

    def infer_onnxruntime(self, frame, fx, fy, cx, cy):
        """Inference using ONNXRuntime session."""
        h, w = frame.shape[:2]
        # Preprocess to (1, 3, 480, 640)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (640, 480))
        img_input = img_resized.transpose(2, 0, 1).astype(np.float32) / 255.0
        img_input = np.expand_dims(img_input, axis=0)

        input_name = self.model.get_inputs()[0].name
        preds = self.model.run(None, {input_name: img_input})[0]
        preds = np.squeeze(preds, axis=0).T  # (N, 84)

        boxes = []
        confidences = []
        class_ids = []

        x_scale = w / 640.0
        y_scale = h / 480.0

        for row in preds:
            scores = row[4:]
            class_id = np.argmax(scores)
            conf = scores[class_id]
            if conf >= self.conf_thresh:
                xc, yc, bw, bh = row[0], row[1], row[2], row[3]
                x1 = int((xc - bw / 2.0) * x_scale)
                y1 = int((yc - bh / 2.0) * y_scale)
                box_w = int(bw * x_scale)
                box_h = int(bh * y_scale)
                boxes.append([x1, y1, box_w, box_h])
                confidences.append(float(conf))
                class_ids.append(int(class_id))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.iou_thresh)
        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                x1, y1, bw, bh = boxes[idx]
                x2 = x1 + bw
                y2 = y1 + bh
                conf = confidences[idx]
                cls_id = class_ids[idx]
                cls_name = list(CLASS_REAL_HEIGHTS.keys())[cls_id] if cls_id < len(CLASS_REAL_HEIGHTS) else f"obj_{cls_id}"

                u_c = (x1 + x2) / 2.0
                v_c = (y1 + y2) / 2.0
                real_h = CLASS_REAL_HEIGHTS.get(cls_name, DEFAULT_OBJECT_HEIGHT)
                distance = (fy * real_h) / max(1.0, float(bh))
                distance = max(0.20, min(30.0, distance))

                bearing_rad = math.atan2(u_c - cx, fx)
                bearing_deg = math.degrees(bearing_rad)

                detections.append({
                    "class": cls_name,
                    "class_id": cls_id,
                    "conf": round(conf, 2),
                    "bbox": [x1, y1, x2, y2],
                    "centroid": [round(u_c, 1), round(v_c, 1)],
                    "distance_m": round(distance, 2),
                    "bearing_deg": round(bearing_deg, 1),
                    "bearing_rad": round(bearing_rad, 4),
                })
        return detections

    def execute_tracking_control(self, target):
        """Active closed-loop target tracking and person follower controller."""
        bearing_rad = target['bearing_rad']
        dist = target['distance_m']

        # Steering: Proportional-Derivative to center target in frame
        kp_yaw = 1.6
        kd_yaw = 0.3
        d_err = bearing_rad - self.last_bearing_err
        omega_z = - (kp_yaw * bearing_rad + kd_yaw * d_err)
        self.last_bearing_err = bearing_rad

        # Distance: Proportional to maintain desired standoff distance
        dist_err = dist - self.target_dist
        kp_dist = 0.50
        v_x = kp_dist * dist_err

        # Saturation limits for balance safety
        v_x = max(-self.max_lin_spd, min(self.max_lin_spd, v_x))
        omega_z = max(-self.max_ang_spd, min(self.max_ang_spd, omega_z))

        # Publish Command Velocity
        cmd = Twist()
        cmd.linear.x = float(v_x)
        cmd.angular.z = float(omega_z)
        self.pub_cmd_vel.publish(cmd)

        # Dynamic Squat Interaction:
        # If target vertical center is in the bottom third of the image (low object/sitting person), squat down!
        h_norm = target['centroid'][1] / 480.0
        squat_cmd = Float64()
        if h_norm > 0.65:
            squat_cmd.data = -0.38  # Crouch down to inspect
        else:
            squat_cmd.data = 0.0    # Stand upright
        self.pub_squat.publish(squat_cmd)

    def render_cybernetic_hud(self, frame, detections, collision_warning, target_obj):
        """Draws high-contrast cybernetic robotics visual HUD overlay."""
        vis = frame.copy()
        h, w = vis.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Target Bounding Boxes
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cls_name = det['class']
            conf = det['conf']
            dist = det['distance_m']
            bearing = det['bearing_deg']

            # Dynamic color coding
            is_target = (target_obj is not None and det == target_obj)
            if collision_warning and dist < 0.9:
                box_color = (0, 0, 255)      # Red alert
            elif is_target:
                box_color = (0, 255, 128)    # Neon green active lock
            elif cls_name == 'person':
                box_color = (255, 230, 0)    # Cyan
            elif cls_name in ['stop sign', 'traffic light']:
                box_color = (0, 0, 255)      # Red
            else:
                box_color = (0, 200, 255)    # Gold/Yellow

            # Glowing Box corners
            cv2.rectangle(vis, (x1, y1), (x2, y2), box_color, 2)
            corner_len = min(16, (x2 - x1) // 4)
            # Glowing Box corner brackets
            cv2.line(vis, (x1, y1), (x1 + corner_len, y1), box_color, 3)
            cv2.line(vis, (x1, y1), (x1, y1 + corner_len), box_color, 3)
            cv2.line(vis, (x2, y1), (x2 - corner_len, y1), box_color, 3)
            cv2.line(vis, (x2, y1), (x2, y1 + corner_len), box_color, 3)
            cv2.line(vis, (x1, y2), (x1 + corner_len, y2), box_color, 3)
            cv2.line(vis, (x1, y2), (x1, y2 - corner_len), box_color, 3)
            cv2.line(vis, (x2, y2), (x2 - corner_len, y2), box_color, 3)
            cv2.line(vis, (x2, y2), (x2, y2 - corner_len), box_color, 3)

            # Centroid crosshair
            u_c, v_c = int(det['centroid'][0]), int(det['centroid'][1])
            cv2.drawMarker(vis, (u_c, v_c), box_color, cv2.MARKER_CROSS, 10, 1)

            # Label Badge
            label = f"{cls_name.upper()} {int(conf*100)}% | {dist:.1f}m"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(vis, (x1, max(0, y1 - th - 8)), (x1 + tw + 8, y1), (15, 20, 25), -1)
            cv2.rectangle(vis, (x1, max(0, y1 - th - 8)), (x1 + tw + 8, y1), box_color, 1)
            cv2.putText(vis, label, (x1 + 4, max(th, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)

        # 2. Central Optical Sight Reticle
        reticle_color = (0, 220, 255)
        cv2.line(vis, (cx - 24, cy), (cx - 8, cy), reticle_color, 1)
        cv2.line(vis, (cx + 8, cy), (cx + 24, cy), reticle_color, 1)
        cv2.line(vis, (cx, cy - 24), (cx, cy - 8), reticle_color, 1)
        cv2.line(vis, (cx, cy + 8), (cx, cy + 24), reticle_color, 1)
        cv2.circle(vis, (cx, cy), 16, reticle_color, 1)

        # 3. Top Header Bar
        cv2.rectangle(vis, (0, 0), (w, 34), (15, 18, 22), -1)
        cv2.line(vis, (0, 34), (w, 34), (0, 200, 255), 2)
        cv2.putText(vis, "WOBBLE | YOLOV8-NANO PERCEPTION", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 235, 255), 1, cv2.LINE_AA)
        
        mode_str = "TRACKING" if self.enable_tracking else "SCANNING"
        mode_col = (0, 255, 150) if self.enable_tracking else (180, 200, 220)
        cv2.putText(vis, f"[{mode_str}]", (w - 115, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, mode_col, 1, cv2.LINE_AA)

        # 4. Bottom Telemetry Bar
        cv2.rectangle(vis, (0, h - 30), (w, h), (15, 18, 22), -1)
        cv2.line(vis, (0, h - 30), (w, h - 30), (0, 200, 255), 1)

        if collision_warning:
            status_text = "[ALERT] COLLISION PROXIMITY! OBSTACLE IN PATH"
            status_color = (0, 50, 255)
        elif target_obj:
            status_text = f"TARGET: {target_obj['class'].upper()} | DIST: {target_obj['distance_m']:.2f}m | BRG: {target_obj['bearing_deg']:+4.1f} deg"
            status_color = (0, 255, 128)
        else:
            status_text = f"Detections: {len(detections)} | FPS: {self.fps:4.1f} | Backend: {self.backend}"
            status_color = (210, 220, 230)

        cv2.putText(vis, status_text, (12, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, status_color, 1, cv2.LINE_AA)

        return vis


def main(args=None):
    rclpy.init(args=args)
    node = WobbleYOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
