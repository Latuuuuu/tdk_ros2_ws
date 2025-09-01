#!/usr/bin/env python3
import os
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from typing import Optional
from interfaces.srv import KeyVisual   # srv 應含: ok,cx,cy,inliers,dx,dy,z

SQUARE_SIZE_M = 0.075  # 主視覺邊長(公尺) 7.5cm

class Table(Node):
    """
    將主視覺置中：
      - 先用模板/白底矩形偵測得到中心 (u,v) 與四角 corners
      - 以主視覺真實尺寸估距 Z_size = fx * 0.07 / w_px  (尺寸估距優先)
      - 反投影 (u,v,Z) 得相機座標 (X,Y,Z)，輸出 (dx,dy)=(-X,-Y)
      - 若尺寸估距不可用，才退回用深度圖估距
    """
    def __init__(self):
        super().__init__('table_node')

        # ---------- 狀態 ----------
        self.bridge = CvBridge()
        self.start = False
        self.last_log_time = 0.0
        self.cv_image = None
        self.location = None          # (u,v) in pixels (color image)
        self.ok = False
        self.inliers = 0
        self.last_corners = None      # (4,2) float32 for size-based Z

        # 內參 / 影像尺寸 / 深度
        self.fx = self.fy = self.cx = self.cy = None
        self.dist = None
        self.color_shape = None   # (h,w)
        self.depth_img = None
        self.depth_shape = None
        self.depth_is_meters = False  # 32FC1(m) or 16UC1(mm)

        # ---------- 參數 ----------
        self.declare_parameter('color_image_topic', '/latuuu_camera/latuuu_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/latuuu_camera/latuuu_camera/color/camera_info')
        self.declare_parameter('aligned_depth_topic', '/latuuu_camera/aligned_depth_to_color/image_raw')

        # 尺寸估距優先區間（m）：在此區間內只用尺寸估距，超出才嘗試深度
        self.declare_parameter('prefer_size_range_min_m', 0.0)
        self.declare_parameter('prefer_size_range_max_m', 0.30)

        # 有效距離範圍（m），防呆
        self.declare_parameter('min_valid_z_m', 0.08)
        self.declare_parameter('max_valid_z_m', 2.50)

        color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        depth_topic       = self.get_parameter('aligned_depth_topic').get_parameter_value().string_value

        self.prefer_min_z = float(self.get_parameter('prefer_size_range_min_m').value)
        self.prefer_max_z = float(self.get_parameter('prefer_size_range_max_m').value)
        self.min_valid_z  = float(self.get_parameter('min_valid_z_m').value)
        self.max_valid_z  = float(self.get_parameter('max_valid_z_m').value)

        # ---------- QoS ----------
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # ---------- 訂閱/發佈 ----------
        self.info_sub  = self.create_subscription(CameraInfo, camera_info_topic, self.info_cb, 10)
        self.img_sub   = self.create_subscription(Image, color_image_topic, self.image_callback, qos)
        # self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_cb, 10)
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

        self.get_logger().info("table_node up (尺寸估距優先)")
    
    def _stop_sub(self):
        if self.img_sub is not None:
            self.destroy_subscription(self.img_sub)
            self.img_sub = None

    # ---------- 路徑處理 ----------
    def _resolve_template_path(self) -> str:
        candidates = []
        try:
            pkg_share = get_package_share_directory('opencv_ros')
            candidates.append(os.path.join(pkg_share, 'asset', 'template.png'))
        except Exception:
            pass
        here = Path(__file__).resolve().parent
        candidates += [str(here / 'asset' / 'template.png'), 'template.png']
        for p in candidates:
            if os.path.exists(p):
                return p
        raise RuntimeError("找不到 template.png, 請放在 share/opencv_ros/asset/ 或與執行檔同層。")

    # ---------- Callbacks ----------
    def info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]; self.fy = msg.k[4]
        self.cx = msg.k[2]; self.cy = msg.k[5]
        self.dist = np.zeros((5,), dtype=np.float64)
        # try:
        #     self.dist = np.array(msg.d, dtype=np.float64).reshape(-1) if len(msg.d) > 0 else None
        # except Exception:
        #     self.dist = None

    def depth_cb(self, msg: Image):
        d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_img = d
        self.depth_shape = d.shape[:2]
        self.depth_is_meters = (d.dtype == np.float32 or d.dtype == np.float64)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_shape = frame.shape[:2]

            vis, center, corners, ok = self.analyze(frame)
            self.cv_image = vis
            self.location = center
            self.ok = ok
            if corners is not None:
                self.last_corners = corners.astype(np.float32)

            if self.start and self.cv_image is not None:
                self.img_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8'))

            now = time.time()
            if not self.start and now - self.last_log_time > 2.0:
                self.get_logger().info('Idle: detection running, but not publishing.')
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    # ---------- Service ----------
    def table_callback(self, request, response):
        try:
            self.start = bool(request.start)

            # 預設填入基本欄位（即使 not ready 也回，方便上游 debug）
            response.ok = False
            response.cx = float(self.location[0]) if self.location is not None else -1.0
            response.cy = float(self.location[1]) if self.location is not None else -1.0
            response.inliers = int(self.inliers) if self.inliers is not None else 0
            response.dx = 0.0
            response.dy = 0.0
            response.z  = -1.0

            if not self.start:
                self.get_logger().info('Table detection stopped.')
                return response

            ready_intrinsics = all(v is not None for v in [self.fx, self.fy, self.cx, self.cy])

            if self.ok and self.location is not None and ready_intrinsics:
                u, v = self.location

                # 先嘗試：PnP（對傾斜最準）
                Z = None
                if self.last_corners is not None:
                    Z_pose = self._estimate_z_from_pose(self.last_corners)
                    if Z_pose is not None and self.min_valid_z <= Z_pose <= self.max_valid_z:
                        Z = Z_pose

                # 若 PnP 無法提供，再用尺寸估距優先
                if Z is None and self.last_corners is not None:
                    w_px, h_px = self._square_edge_px_hw(self.last_corners)
                    w_eff = max(1.0, 0.5 * (w_px + h_px))   # 像素邊長的平均，避免除以 0
                    f_eff = float(np.sqrt(self.fx * self.fy))  # 等效焦距，降低 fx≠fy 的偏差
                    Z_size = (f_eff * SQUARE_SIZE_M) / w_eff
                    if self.min_valid_z <= Z_size <= self.max_valid_z:
                        Z = Z_size

                # 最後才用深度備援/或尺寸回退
                if Z is None:
                    Z = self.depth_at(u, v)
                    if Z is None and self.last_corners is not None:
                        w_px = self._square_edge_px(self.last_corners)
                        if w_px > 5:
                            Z = (self.fx * SQUARE_SIZE_M) / w_px
                            self.get_logger().warn("Depth invalid → fallback to size-based Z.")

                if Z is None or not (self.min_valid_z <= Z <= self.max_valid_z):
                    self.get_logger().warning(f'Z invalid: {Z}')
                    return response

                # 像素→相機座標
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                response.ok = True
                response.z  = float(Z)
                response.dx = float(X*100.0)
                response.dy = float(-Y*100.0)

                self.get_logger().info(f"KV OK: (u,v)=({u:.1f},{v:.1f}), Z={Z:.3f} m, move (dx,dy)=({X:.3f},{-Y:.3f}) m")
            else:
                self.get_logger().warning('Table detection NOT ready (vision/intrinsics).')
            
            self._stop_sub()
            self.start = False
            return response

        except Exception as e:
            self.get_logger().error(f'service error: {e}')
            return response

    # ---------- 視覺核心 ----------
    def analyze(self, frame):
        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ORB 特徵
        kps, desc = self._orb.detectAndCompute(gray, None)  #kps:critical points , desc:descriptors
        if desc is None or len(kps) < 8 or self._tDesc is None: #tDesc:template descriptors
            return vis, None, None, False

        # KNN + Lowe ratio
        matches = self._bf.knnMatch(self._tDesc, desc, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]
        if len(good) < 12:
            return self._fallback_white_rect(frame)

        # 單應性
        src = np.float32([self._tKp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2) #src:template keypoints' coordinates
        dst = np.float32([kps[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)  #dst:frame keypoints' coordinates
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 5.0) #H:homography matrix, turning src to dst
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

    # ---------- 工具 ----------
    def _order_corners(self, corners: np.ndarray) -> np.ndarray:
        """將四角排序為 TL, TR, BR, BL(clockwise),輸入形狀 (4,2)"""
        c = corners.reshape(-1, 2).astype(np.float32)
        s = c.sum(axis=1)              # x+y
        d = c[:, 0] - c[:, 1]          # x-y
        tl = c[np.argmin(s)]
        br = c[np.argmax(s)]
        tr = c[np.argmin(d)]
        bl = c[np.argmax(d)]
        return np.array([tl, tr, br, bl], dtype=np.float32)
    
    def _square_edge_px_hw(self, corners: np.ndarray):
        """回傳四邊形(水平寬)與(垂直高)的平均像素長度"""
        c = corners.reshape(-1, 2).astype(np.float32)
        # 對應四角: [0]=TL, [1]=TR, [2]=BR, [3]=BL（你的流程中順序可能不同，但成對相鄰即可）
        w1 = float(np.linalg.norm(c[0] - c[1]))
        w2 = float(np.linalg.norm(c[2] - c[3]))
        h1 = float(np.linalg.norm(c[1] - c[2]))
        h2 = float(np.linalg.norm(c[3] - c[0]))
        w_px = 0.5 * (w1 + w2)
        h_px = 0.5 * (h1 + h2)
        return w_px, h_px
    
    def _estimate_z_from_pose(self, corners: np.ndarray) -> Optional[float]:
        """用 PnP 在傾斜下估 Z，從多解中挑 Z>0 且重投影誤差最小者"""
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        cimg = self._order_corners(corners).astype(np.float32)  # TL,TR,BR,BL

        half = SQUARE_SIZE_M / 2.0
        obj = np.array([[-half, -half, 0.0],
                        [ half, -half, 0.0],
                        [ half,  half, 0.0],
                        [-half,  half, 0.0]], dtype=np.float32)

        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0,      0,    1]], dtype=np.float64)
        dist = self.dist if self.dist is not None else np.zeros((5,), dtype=np.float64)

        try:
            # 取多解
            retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj, cimg, K, dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not retval or len(tvecs) == 0:
                ok, rvec, tvec = cv2.solvePnP(obj, cimg, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
                if not ok:
                    return None
                rvecs, tvecs = [rvec], [tvec]

            best_Z, best_err = None, 1e9
            for rvec, tvec in zip(rvecs, tvecs):
                # 計算重投影誤差
                proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
                proj = proj.reshape(-1, 2)
                err = float(np.sqrt(np.mean(np.sum((proj - cimg)**2, axis=1))))
                z = float(tvec[2, 0]) if tvec.shape == (3, 1) else float(tvec[2])

                # 優先選擇 Z>0 的解，並取最小誤差
                if z > 0 and err < best_err:
                    best_err, best_Z = err, z

            # 若皆不是正 Z，則挑誤差最小（但標記警告）
            if best_Z is None:
                errs = []
                for rvec, tvec in zip(rvecs, tvecs):
                    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
                    proj = proj.reshape(-1, 2)
                    errs.append(float(np.sqrt(np.mean(np.sum((proj - cimg)**2, axis=1)))))
                idx = int(np.argmin(errs))
                z = float(tvecs[idx][2, 0]) if tvecs[idx].shape == (3, 1) else float(tvecs[idx][2])
                self.get_logger().warn(f'PnP only gave non-positive Z; picking smallest error. Z={z:.6f}, err={errs[idx]:.3f}')
                best_Z = z

            # Debug：若異常小或負，印出資訊幫助定位
            if not np.isfinite(best_Z) or abs(best_Z) < 0.02:
                self.get_logger().warn(f'PnP Z suspicious: Z={best_Z:.6f}, fx={self.fx:.1f}, fy={self.fy:.1f}')
            return best_Z
        except Exception as e:
            self.get_logger().warn(f'PnP failed: {e}')
            return None

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

        # 擴窗搜尋有效深度
        for ksize in (ksize_start, 7, 9, 11):
            r = ksize // 2
            y0, y1 = max(0, v - r), min(h_d, v + r + 1)
            x0, x1 = max(0, u - r), min(w_d, u + r + 1)
            patch = self.depth_img[y0:y1, x0:x1].astype(np.float32)
            vals = patch.flatten()
            vals = vals[vals > 0]
            if vals.size >= max(3, (ksize*ksize)//4):
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