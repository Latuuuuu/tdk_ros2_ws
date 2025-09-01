import os, time, cv2, numpy as np, rclpy
from rclpy.node import Node
from interfaces.srv import Menu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cv2.setNumThreads(1)

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_node')

        # 讓裝置可參數化：整數索引或 /dev/v4l/by-id/...
        self.declare_parameter('video_device', os.getenv('VIDEO_DEVICE', '6'))
        dev = self.get_parameter('video_device').get_parameter_value().string_value
        try:
            self.video_dev = int(dev)  # "6"
        except ValueError:
            self.video_dev = dev       # "/dev/v4l/by-id/xxx"

        self.cap = None
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'coffee_detection', 10)

        self.server = self.create_service(Menu, 'menu', self.menu_callback)

        self.color_idx = -1
        self.table_id  = -1

    # ------------ Camera helpers ------------
    def open_camera(self) -> bool:
        if self.cap and self.cap.isOpened():
            return True
        # 盡量指定 V4L2 後端，較穩
        try:
            self.cap = cv2.VideoCapture(self.video_dev, cv2.CAP_V4L2)
        except Exception:
            self.cap = cv2.VideoCapture(self.video_dev)

        # 可選：降低延遲
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        # 依需求調整解析度 / FPS（不保證一定生效，視驅動）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS,          30)

        ok = self.cap.isOpened()
        if not ok:
            self.get_logger().error(f'Cannot open camera device: {self.video_dev}')
        return ok

    def close_camera(self):
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def publish_image(self, bgr):
        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.pub.publish(msg)

    # ------------- Service -------------
    def menu_callback(self, request, response):
        if not request.start:
            self.get_logger().info('Menu stopped')
            response.result = -1
            self.close_camera()
            return response

        # Start == True ：同步抓幀直到成功或逾時
        if not self.open_camera():
            response.result = -1
            return response

        self.get_logger().info(f'Menu started; capturing from {self.video_dev}')
        deadline = time.time() + 15.0
        self.color_idx, self.table_id = -1, -1

        while time.time() < deadline:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            vis, color_idx, table_id = analyze(frame)
            # 發佈一下偵測疊圖（方便你在 foxglove/rviz 看）
            self.publish_image(vis)

            if color_idx >= 0 and table_id >= 0:
                self.color_idx, self.table_id = color_idx, table_id
                break

            time.sleep(0.03)  # 小睡避免 CPU 滿載

        if self.color_idx >= 0 and self.table_id >= 0:
            result = self.color_idx * 10 + self.table_id
            self.get_logger().info(f'Menu done, result={result}')
            response.result = result
        else:
            self.get_logger().warn('Detection not ready yet (timeout)')
            response.result = -1

        # 若不需要常駐佔用相機，就關掉；要保溫可改成不關
        self.close_camera()
        return response


# ----------------- Image processing -----------------
def is_square(cnt):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
    return len(approx) == 4

def relative_location(cnt, contours, resized):
    if not contours: return -1
    i, cp = 0, [0, 0]
    for c in contours:
        if c is cnt: continue
        if cv2.contourArea(c) < 200: continue
        if not is_square(c): continue
        M = cv2.moments(c)
        if M["m00"] == 0: continue
        i += 1
        cp[0] += int(M["m10"] / M["m00"])
        cp[1] += int(M["m01"] / M["m00"])
    if i == 0: return -1
    cp[0] //= i; cp[1] //= i
    M = cv2.moments(cnt)
    cv2.circle(resized, (int(cp[0]), int(cp[1])), 10, (255, 0, 0), -1)
    if M["m00"] == 0: return -1
    x = int(M["m10"] / M["m00"]) - cp[0]
    y = int(M["m01"] / M["m00"]) - cp[1]
    return (x, y)

def analyze(frame):
    # 回傳 (可視化影像, color_idx, table_id)
    resized = cv2.resize(frame, (int(frame.shape[1]*1.5), int(frame.shape[0]*1.5)))
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    eroded = cv2.erode(thresh, kernal, iterations=1)
    dilated = cv2.dilate(eroded, kernal, iterations=1)

    contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:12]

    color_idx, table_id, found = -1, -1, False
    for cnt in contours:
        if cv2.contourArea(cnt) < 200: continue
        if not is_square(cnt): continue

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = cv2.mean(thresh, mask=mask)[0]

        if mean_val < 200:
            M = cv2.moments(cnt)
            if M["m00"] == 0: continue
            cX = int(M["m10"]/M["m00"]); cY = int(M["m01"]/M["m00"])
            h, w = thresh.shape[:2]
            cX = int(np.clip(cX, 0, w-1)); cY = int(np.clip(cY, 0, h-1))

            center_val = int(thresh[cY, cX])  # 0 or 255
            color_idx = 0 if center_val == 0 else 1
            color = (255, 0, 0) if color_idx == 0 else (0, 0, 255)

            lac = relative_location(cnt, contours, resized)
            if lac == -1: continue
            if   lac[0] >= 0 and lac[1] >= 0: table_id = 2
            elif lac[0] >= 0 and lac[1] <  0: table_id = 3
            elif lac[0] <  0 and lac[1] >= 0: table_id = 1
            else:                              table_id = 4
            found = True
        else:
            color = (0, 255, 0)

        M = cv2.moments(cnt)
        cX, cY = (0, 0) if M["m00"]==0 else (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        cv2.drawContours(resized, [cnt], -1, color, 2)
        cv2.putText(resized, f"{mean_val:>5.1f} {table_id}", (cX-20, cY),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if found: break

    return resized, color_idx, table_id


def main(args=None):
    rclpy.init(args=args)
    node = Coffee()
    try:
        rclpy.spin(node)
    finally:
        node.close_camera()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()