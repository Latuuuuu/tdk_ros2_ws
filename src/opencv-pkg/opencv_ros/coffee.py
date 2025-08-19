import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def is_square(cnt):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
    # if len(approx) < 3: return False
    # if len(approx) > 5: return False
    if len(approx) != 4: return False
    # if peri*peri / cv2.contourArea(cnt) < 15: return False
    # if peri*peri / cv2.contourArea(cnt) > 17: return False
    return True

def relative_location(cnt,contours, resized):
    if not contours:
        return -1
    else:
        i=0
        cp=[0,0]
        for c in contours:
            if c is cnt: continue
            if cv2.contourArea(c) < 200: continue
            if not is_square(c): continue
            else:
                i += 1
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                cp[0] += x
                cp[1] += y
        if i == 0:
            return -1        
        cp[0] = int(cp[0] / i)
        cp[1] = int(cp[1] / i)
        M = cv2.moments(cnt)
        cv2.circle(resized, (int(cp[0]), int(cp[1])), 10, (255, 0, 0), -1)
        if M["m00"] == 0: return -1
        x = int(M["m10"] / M["m00"]) - cp[0]
        y = int(M["m01"] / M["m00"]) - cp[1]
        return (x, y)
    
def analyze(frame):
    resized = cv2.resize(frame, (int(frame.shape[1]*1.5), int(frame.shape[0]*1.5)))
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    # thresh = cv2.adaptiveThreshold(blur, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 11, 3 )
    kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    eroded = cv2.erode(thresh, kernal, iterations=1)
    # kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    dilated = cv2.dilate(eroded, kernal, iterations=1)
    
    contours, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:12]

    for cnt in contours:
        if cv2.contourArea(cnt) < 200: continue
        if not is_square(cnt): continue

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = cv2.mean(thresh, mask=mask)[0]   # 只用單通道
        num = 0
        if mean_val >= 200: color = (0, 255, 0)
        else:
            lac = relative_location(cnt, contours, resized)
            if lac == -1: continue
            elif lac[0] >= 0 and lac[1] >= 0: num = 4
            elif lac[0] >= 0 and lac[1] < 0: num = 3
            elif lac[0] < 0 and lac[1] >= 0: num = 2
            else: num = 1
            if mean_val >= 100: color = (255, 0, 0)
            else: color = (0, 0, 255)
        M = cv2.moments(cnt)
        cX, cY = (0, 0) if M["m00"]==0 else (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        cv2.drawContours(resized, [cnt], -1, color, 2)
        cv2.putText(resized, f"{mean_val:>5.1f} {num}", (cX-20, cY),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return resized

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_node')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            "/latuuu_camera/latuuu_camera/color/image_raw",
            self.image_callback,
            10
        )
        self.img_publisher = self.create_publisher(Image, 'coffee_detection', 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #self.get_logger().info('Received an image!')
            self.publish_processed_image()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def publish_processed_image(self):
        try:
            # templates = [
            #     cv2.imread("/home/opencv/vision-ws/src/opencv_ros/asset/example.png"),
            #     cv2.imread("/home/opencv/vision-ws/src/opencv_ros/asset/example2.png")
            # ]
            # best_match_val = -1
            # best_match_idx = -1
            # best_match_loc = None
            # method = cv2.TM_CCOEFF_NORMED

            # for i, template in enumerate(templates):
            #     result = cv2.matchTemplate(self.cv_image, template, method)
            #     _, max_val, _, max_loc = cv2.minMaxLoc(result)

            #     if max_val > best_match_val:
            #         best_match_val = max_val
            #         best_match_idx = i
            #         best_match_loc = max_loc
            #         best_template_shape = template.shape[:2]

            # 繪製結果
            # h, w = best_template_shape
            # top_left = best_match_loc
            # bottom_right = (top_left[0] + w, top_left[1] + h)
            # cv2.rectangle(self.cv_image, top_left, bottom_right, (0, 255, 0), 2)
            # cv2.putText(self.cv_image, f"Best Match: Template {best_match_idx + 1}", (10, 30),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.cv_image=analyze(self.cv_image)
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.img_publisher.publish(msg)
            #self.get_logger().info('Processed image published!')
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Coffee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()