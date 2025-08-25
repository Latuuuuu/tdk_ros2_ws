import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time 
import numpy as np
from interfaces.srv import Menu

class Coffee(Node):
    def __init__(self):
        super().__init__('coffee_node')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image,"/latuuu_camera/latuuu_camera/color/image_raw",self.image_callback,10)
        self.img_publisher = self.create_publisher(Image, 'coffee_detection', 10)
        self.server = self.create_service(Menu, 'menu', self.menu_callback)
        self.color_idx = -1
        self.table_id = -1
        self.start = False
        self.last_log_time = 0
        self.cv_image = None

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cv_image,self.color_idx,self.table_id=analyze(self.cv_image)
            if self.start:
                self.publish_processed_image()
            else:
                current_time = time.time()
                if current_time - self.last_log_time > 2.0:
                    self.get_logger().info('Waiting for camera to start...')
                    self.last_log_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def publish_processed_image(self):
        try:
            if self.start:
                self.cv_image,self.color_idx,self.table_id=analyze(self.cv_image)
                msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
                self.img_publisher.publish(msg)
            else:
                current_time = time.time()
                if current_time - self.last_log_time > 2.0:
                    self.get_logger().info('Camera is not started, skipping image processing.')
                    self.last_log_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')

    def menu_callback(self, request, response):
        if request.start:
            self.start = True 
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if self.color_idx >= 0 and self.table_id >= 0:
                    result = self.color_idx * 10 + self.table_id
                    self.get_logger().info(f'Menu started, result={result}')
                    response.result = result
                    return response
                time.sleep(0.05)
            self.get_logger().warn('Detection not ready yet (timeout)')
            response.result = -1
            return response
        else:
            self.get_logger().info('Menu stopped')
            response.result = -1
            self.start = False
            return response
    


def main(args=None):
    rclpy.init(args=args)
    node = Coffee()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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

    color_idx = -1
    table_id = -1 
    found = False

    for cnt in contours:
        if cv2.contourArea(cnt) < 200: continue
        if not is_square(cnt): continue

        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = cv2.mean(thresh, mask=mask)[0]   # 只用單通道
        if mean_val >= 200: color = (0, 255, 0)
        else:
            lac = relative_location(cnt, contours, resized)
            if lac == -1: continue
            elif lac[0] >= 0 and lac[1] >= 0: table_id = 2
            elif lac[0] >= 0 and lac[1] < 0: table_id = 3
            elif lac[0] < 0 and lac[1] >= 0: table_id = 1
            else: table_id = 4
            if mean_val >= 100: 
                color_idx = 0
                color = (255, 0, 0)
            else: 
                color = (0, 0, 255)
                color_idx = 1
            found = True
        M = cv2.moments(cnt)
        cX, cY = (0, 0) if M["m00"]==0 else (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        cv2.drawContours(resized, [cnt], -1, color, 2)
        cv2.putText(resized, f"{mean_val:>5.1f} {table_id}", (cX-20, cY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if found:
            break
    return resized,color_idx,table_id