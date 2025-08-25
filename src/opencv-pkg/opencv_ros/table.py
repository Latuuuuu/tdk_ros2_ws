import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

# ⬇︎ 如果你暫時還不能改 srv，先保留 Menu；建議改成 TableDetect 後把 import 換掉
# from interfaces.srv import TableDetect as Table
from interfaces.srv import KeyVisual  

class Table(Node):
    def __init__(self):
        super().__init__('table_node')

        # ---- state ----
        self.bridge = CvBridge()
        self.start = False
        self.last_log_time = 0.0
        self.cv_image = None
        self.location = None          # (cx, cy) in pixels
        self.ok = False
        self.inliers = 0

        # ---- camera topics ----
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.img_sub = self.create_subscription(
            Image, "/latuuu_camera/latuuu_camera/color/image_raw",
            self.image_callback, qos
        )
        self.img_publisher = self.create_publisher(Image, 'table_detection', 10)

        # ---- service ----
        self.server = self.create_service(KeyVisual, 'table', self.table_callback)

        # ---- feature / template ----
        candidates = []
        try:
            pkg_share = get_package_share_directory('opencv_ros')
            # 只嘗試 asset/
            candidates.append(os.path.join(pkg_share, 'asset', 'template.png'))
        except Exception:
            pass

        # source tree 下開發時（未安裝）
        here = Path(__file__).resolve().parent
        candidates += [
            str(here / 'asset' / 'template.png'),
            'template.png',
        ]

        template_path = None
        for p in candidates:
            if os.path.exists(p):
                template_path = p
                break

        if template_path is None:
            raise RuntimeError("找不到 template.png,請確認它已被安裝到 share/opencv_ros/asset/ 或放在執行目錄。")

        self.get_logger().info(f"Using template: {template_path}")
        self._template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        if self._template is None:
            raise RuntimeError(f"讀取模板失敗：{template_path}")

        self._orb = cv2.ORB_create(1500)
        self._tKp, self._tDesc = self._orb.detectAndCompute(self._template, None)
        self._hT, self._wT = self._template.shape[:2]
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.get_logger().info("table_node up")

    # ---------------- Callbacks ----------------

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 只分析一次，並保存結果+可視化影像
            vis, center, _, ok = self.analyze(frame)
            self.cv_image = vis
            self.location = center
            self.ok = ok

            # 可選：在 start 期間才發佈疊圖，減少頻寬
            if self.start and self.cv_image is not None:
                self.img_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8'))

            # 偶爾 log 一下
            now = time.time()
            if not self.start and now - self.last_log_time > 2.0:
                self.get_logger().info('Idle: detection running, but not publishing.')
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    def table_callback(self, request, response):
        try:
            self.start = bool(request.start)

            if self.start:
                # 立即回最新結果，不阻塞
                if self.ok and self.location is not None:
                    cx, cy = self.location
                    response.ok = True
                    response.cx = float(cx)
                    response.cy = float(cy)
                    response.inliers = int(self.inliers)
                    self.get_logger().info(f'Table detection OK: center=({cx:.1f},{cy:.1f})')
                else:
                    self.get_logger().warning('Table detection NOT ready.')
            else:
                response.ok = False
                self.get_logger().info('Table detection stopped.')
            return response
        except Exception as e:
            self.get_logger().error(f'service error: {e}')
            response.ok = False
            response.cx = response.cy = -1.0
            response.inliers = 0
            return response

    # ---------------- Vision core ----------------

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

        # 畫圖＋中心
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


def main(args=None):
    rclpy.init(args=args)
    node = Table()
    # 若你真的要在 service 中 busy-wait，請改用 MultiThreadedExecutor
    # from rclpy.executors import MultiThreadedExecutor
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
