#!/usr/bin/env python3
import os
import time
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from interfaces.srv import KeyVisual   # srv 需包含: ok,cx,cy,inliers,dx,dy,z

SQUARE_SIZE_M = 0.07  # 主視覺邊長(公尺) 7cm

class Table(Node):
    """
    一次性按需偵測：
      - 收到 /table(start=true) 才建立影像/深度訂閱
      - 只處理第一幀有效結果 → 回傳 → 關閉訂閱
      - 尺寸估距優先，深度作為備援
    回傳 (dx,dy) 為相機座標系所需位移（公尺），將主視覺移至影像中心。
    """
    def __init__(self):
        super().__init__('table_node')

        # ---------- 狀態 ----------
        self.bridge = CvBridge()
        self.active = False                  # 是否正在一次性偵測
        self._once_event = threading.Event() # 本次偵測完成事件
        self._result = None                  # 本次服務回傳的暫存結果

        self.cv_image = None
        self.location = None          # (u,v) in pixels (color image)
        self.ok = False
        self.inliers = 0
        self.last_corners = None      # (4,2) float32 for size-based Z

        # 內參 / 影像尺寸 / 深度
        self.fx = self.fy = self.cx = self.cy = None
        self.color_shape = None   # (h,w)
        self.depth_img = None
        self.depth_shape = None
        self.depth_is_meters = False  # 32FC1(m) or 16UC1(mm)

        # ---------- 參數 ----------
        # 話題
        self.declare_parameter('color_image_topic', '/latuuu_camera/latuuu_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/latuuu_camera/latuuu_camera/color/camera_info')
        self.declare_parameter('aligned_depth_topic', '/latuuu_camera/aligned_depth_to_color/image_raw')
        # 尺寸估距優先區間（m）：在此區間內只用尺寸估距，超出才嘗試深度
        self.declare_parameter('prefer_size_range_min_m', 0.0)
        self.declare_parameter('prefer_size_range_max_m', 0.30)
        # 有效距離範圍（m）
        self.declare_parameter('min_valid_z_m', 0.08)
        self.declare_parameter('max_valid_z_m', 2.50)
        # 一次性偵測逾時（秒）
        self.declare_parameter('detection_timeout_sec', 3.0)
        # 是否在偵測完成時發佈可視化影像
        self.declare_parameter('publish_viz', True)

        self.color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.depth_topic       = self.get_parameter('aligned_depth_topic').get_parameter_value().string_value

        self.prefer_min_z = float(self.get_parameter('prefer_size_range_min_m').value)
        self.prefer_max_z = float(self.get_parameter('prefer_size_range_max_m').value)
        self.min_valid_z  = float(self.get_parameter('min_valid_z_m').value)
        self.max_valid_z  = float(self.get_parameter('max_valid_z_m').value)
        self.timeout_sec  = float(self.get_parameter('detection_timeout_sec').value)
        self.publish_viz  = bool(self.get_parameter('publish_viz').value)

        # ---------- QoS ----------
        self.img_qos = QoSProfile(depth=10)
        self.img_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.img_qos.history = QoSHistoryPolicy.KEEP_LAST

        # ---------- 訂閱/發佈 ----------
        # 常駐訂閱 camera_info（頻率低、成本小）
        self.info_sub  = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
        # 影像/深度訂閱「按需建立、用完即關」
        self.img_sub = None
        self.depth_sub = None

        self.img_publisher = self.create_publisher(Image, 'table_detection', 10)

        # ---------- Service ----------
        self.server = self.create_service(KeyVisual, 'table', self.table_callback)

        # ---------- 模板/特徵 ----------
        template_path = self._resolve_template_path()
        self.get_logger().info(f"Using template: {template_path}")
        self._template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        if self._template is None:
            raise RuntimeError(f"讀取模板失敗：{template_path}")

        self._orb = cv2.ORB_create(1500)
        self._tKp, self._tDesc = self._orb.detectAndCompute(self._template, None)
        self._hT, self._wT = self._template.shape[:2]
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.get_logger().info("table_node up (on-demand, size-first)")

    # ---------- 路徑處理 ----------
    def _resolve_template_path(self) -> str:
        candidates = []
        try:
            pkg_share = get_package_share_directory('opencv_ros')
            candidates.append(os.path.join(pkg_share, 'asset', 'template.png'))  # 只嘗試 asset/
        except Exception:
            pass
        here = Path(__file__).resolve().parent
        candidates += [str(here / 'asset' / 'template.png'), 'template.png']
        for p in candidates:
            if os.path.exists(p):
                return p
        raise RuntimeError("找不到 template.png, 請放在 share/opencv_ros/asset/ 或與執行檔同層。")

    # ---------- 動態訂閱開關 ----------
    def _start_subscriptions(self):
        if self.img_sub is None:
            self.img_sub = self.create_subscription(
                Image, self.color_image_topic, self.image_callback, self.img_qos
            )
        if self.depth_sub is None:
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self.depth_cb, 10
            )

    def _stop_subscriptions(self):
        if self.img_sub is not None:
            self.destroy_subscription(self.img_sub)
            self.img_sub = None
        if self.depth_sub is not None:
            self.destroy_subscription(self.depth_sub)
            self.depth_sub = None

    # ---------- Callbacks ----------
    def info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]; self.fy = msg.k[4]
        self.cx = msg.k[2]; self.cy = msg.k[5]

    def depth_cb(self, msg: Image):
        if not self.active:
            return
        d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_img = d
        self.depth_shape = d.shape[:2]
        self.depth_is_meters = (d.dtype == np.float32 or d.dtype == np.float64)

    def image_callback(self, msg: Image):
        if not self.active:
            return  # 不在任務中，完全不做事
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_shape = frame.shape[:2]

            vis, center, corners, ok = self.analyze(frame)
            self.cv_image = vis
            self.location = center
            self.ok = ok
            if corners is not None:
                self.last_corners = corners.astype(np.float32)

            # 只要這一幀可用，就計算一次 (dx,dy,z) 並結束
            result_dict = self._compute_offset_once()
            if result_dict is not None:
                if self.publish_viz and self.cv_image is not None:
                    try:
                        self.img_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8'))
                    except Exception:
                        pass
                self._result = result_dict
                self._once_event.set()   # 通知 service：完成了

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')
            self._result = None
            self._once_event.set()

    # ---------- Service ----------
    def table_callback(self, request, response):
        try:
            if not request.start:
                # no-op 回應
                response.ok = False
                response.cx = response.cy = -1.0
                response.inliers = 0
                response.dx = response.dy = 0.0
                response.z = -1.0
                self.get_logger().info('Table detection stopped (no-op).')
                return response

            # 準備一次性偵測
            self._once_event.clear()
            self._result = None
            self.active = True
            self._start_subscriptions()
            self.get_logger().info('One-shot detection started.')

            # 等待結果（逾時保護）
            done = self._once_event.wait(timeout=self.timeout_sec)

            # 收尾：無論成功/失敗都關訂閱
            self.active = False
            self._stop_subscriptions()

            # 組回傳
            if not done or self._result is None:
                self.get_logger().warn('One-shot detection timeout or no result.')
                response.ok = False
                response.cx = response.cy = -1.0
                response.inliers = 0
                response.dx = response.dy = 0.0
                response.z = -1.0
                return response

            res = self._result
            response.ok = res['ok']
            response.cx = res['cx']
            response.cy = res['cy']
            response.inliers = res['inliers']
            response.dx = res['dx']
            response.dy = res['dy']
            response.z  = res['z']
            return response

        except Exception as e:
            self.get_logger().error(f'service error: {e}')
            response.ok = False
            response.cx = response.cy = -1.0
            response.inliers = 0
            response.dx = response.dy = 0.0
            response.z = -1.0
            return response

    # ---------- 視覺核心 ----------
    def analyze(self, frame):
        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ORB 特徵
        kps, desc = self._orb.detectAndCompute(gray, None)
        if desc is None or len(kps) < 8 or self._tDesc is None:
            return vis, None, None, False

        # KNN + Lowe ratio
        matches = self._bf.knnMatch(self._tDesc, desc, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]
        if len(good) < 12:
            return self._fallback_white_rect(frame)

        # 單應性
        src = np.float32([self._tKp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kps[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or inliers is None or inliers.sum() < 10:
            return self._fallback_white_rect(frame)

        # 投影四角
        corners_t = np.float32([[0, 0], [self._wT, 0], [self._wT, self._hT], [0, self._hT]]).reshape(-1, 1, 2)
        corners_i = cv2.perspectiveTransform(corners_t, H).reshape(-1, 2)

        # 疊圖＋中心
        cv2.polylines(vis, [corners_i.astype(int)], True, (0, 255, 0), 2, cv2.LINE_AA)
        cx, cy = corners_i.mean(axis=0)
        cv2.circle(vis, (int(cx), int(cy)), 6, (0, 0, 255), -1)

        self.inliers = int(inliers.sum())
        return vis, (float(cx), float(cy)), corners_i, True

    def _fallback_white_rect(self, frame):
        vis = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 190), (179, 60, 255))
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_score = None, 0.0
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 1000:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                rect = cv2.minAreaRect(approx)
                w, h = rect[1]
                if w * h <= 0:
                    continue
                rect_area = w * h
                fill_ratio = area / rect_area
                aspect = max(w, h) / max(1.0, min(w, h))
                score = fill_ratio - 0.1 * abs(aspect - 1.0)
                if score > best_score:
                    best_score, best = score, approx

        if best is not None:
            corners = best.reshape(-1, 2).astype(np.float32)
            cx, cy = corners.mean(axis=0)
            cv2.polylines(vis, [corners.astype(int)], True, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.circle(vis, (int(cx), int(cy)), 6, (0, 165, 255), -1)
            self.inliers = 0
            return vis, (float(cx), float(cy)), corners, True

        return vis, None, None, False

    # ---------- 偏移計算（一次性） ----------
    def _compute_offset_once(self):
        out = {
            'ok': False,
            'cx': float(self.location[0]) if self.location is not None else -1.0,
            'cy': float(self.location[1]) if self.location is not None else -1.0,
            'inliers': int(self.inliers) if self.inliers is not None else 0,
            'dx': 0.0, 'dy': 0.0, 'z': -1.0
        }
        if self.location is None:
            return out

        # 需要內參
        if not all(v is not None for v in [self.fx, self.fy, self.cx, self.cy]):
            return out

        # ---- 尺寸估距優先 ----
        Z = None
        if self.last_corners is not None:
            w_px = self._square_edge_px(self.last_corners)
            if w_px > 5:
                Z_size = (self.fx * SQUARE_SIZE_M) / w_px
                if self.prefer_min_z <= Z_size <= self.prefer_max_z:
                    Z = Z_size
                    self.get_logger().info_once("Using size-based range as primary estimator.")

        # ---- 深度備援（或尺寸超出偏好區間）----
        if Z is None and self.depth_img is not None:
            Z = self.depth_at(self.location[0], self.location[1])
            if Z is None and self.last_corners is not None:
                # 深度仍無 → 尺寸再救一次（即使超出偏好區間）
                w_px = self._square_edge_px(self.last_corners)
                if w_px > 5:
                    Z = (self.fx * SQUARE_SIZE_M) / w_px
                    self.get_logger().warn("Depth invalid → fallback to size-based Z outside prefer range.")

        # 最終檢查
        if Z is None or not (self.min_valid_z <= Z <= self.max_valid_z):
            return out

        # 像素→相機座標
        u, v = self.location
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        dx_cam = -X
        dy_cam = -Y

        out.update({'ok': True, 'dx': float(dx_cam), 'dy': float(dy_cam), 'z': float(Z)})
        self.get_logger().info(f"One-shot OK: (u,v)=({u:.1f},{v:.1f}), Z={Z:.3f} m, move (dx,dy)=({dx_cam:.3f},{dy_cam:.3f}) m")
        return out

    # ---------- 工具 ----------
    def _square_edge_px(self, corners: np.ndarray) -> float:
        """回傳四邊形四條邊長的平均（像素）"""
        c = corners.reshape(-1, 2).astype(np.float32)
        d = lambda i, j: float(np.linalg.norm(c[i] - c[j]))
        return 0.25 * (d(0, 1) + d(1, 2) + d(2, 3) + d(3, 0))

    def depth_at(self, u_color, v_color, ksize_start=5):
        """深度備援：若深度/解析度不可用會回 None。單位輸出 m。"""
        if self.depth_img is None or self.depth_shape is None or self.color_shape is None:
            return None
        h_d, w_d = self.depth_shape
        h_c, w_c = self.color_shape
        # 彩色→深度座標縮放
        if w_d != w_c or h_d != h_c:
            u = int(np.clip(u_color * (w_d / w_c), 0, w_d - 1))
            v = int(np.clip(v_color * (h_d / h_c), 0, h_d - 1))
        else:
            u = int(np.clip(u_color, 0, w_d - 1))
            v = int(np.clip(v_color, 0, h_d - 1))

        # 擴窗搜尋有效深度：5→7→9→11
        for ksize in (ksize_start, 7, 9, 11):
            r = ksize // 2
            y0, y1 = max(0, v - r), min(h_d, v + r + 1)
            x0, x1 = max(0, u - r), min(w_d, u + r + 1)
            patch = self.depth_img[y0:y1, x0:x1].astype(np.float32)
            vals = patch.flatten()
            vals = vals[vals > 0]
            if vals.size >= max(3, (ksize * ksize) // 4):
                if self.depth_is_meters:
                    return float(np.median(vals))
                else:
                    return float(np.median(vals) / 1000.0)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Table()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
