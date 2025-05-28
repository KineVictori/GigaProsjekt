import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detection(Node):

    def __init__(self):
        super().__init__("detection_node")

        # Subscribe to a image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)

        # Publish object position
        self.publisher = self.create_publisher(
            Point,
            "object_position",
            10)
        
        # Publisher for feildetektering
        self.error_publisher = self.create_publisher(
            String,
            "detection_error",
            10
        )

        # Initialize CVBridge
        self.bridge = CvBridge()

        self.resolution = (640, 480)
        self.center_point = (self.resolution[0]//2, self.resolution[1]//2, 1000)

        self.hsv_vals = {
            "Red": {'hmin': 0, 'smin': 107, 'vmin': 227, 'hmax': 179, 'smax': 255, 'vmax': 255},
            "Blue": {'hmin': 99, 'smin': 73, 'vmin': 0, 'hmax': 158, 'smax': 255, 'vmax': 255},
            "Yellow": {'hmin': 18, 'smin': 38, 'vmin': 223, 'hmax': 32, 'smax': 198, 'vmax': 255}
        }
        #self.color_finder = ColorFinder(False)


    def image_callback(self, msg):

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        found_colors = []
        combined_mask = None

        # Prosesser bildet for alle farger i hsvVals og slå sammen maskene
        for name, hsv in self.hsv_vals.items():
            lower = (hsv["hmin"], hsv["smin"], hsv["vmin"])
            upper = (hsv["hmax"], hsv["smax"], hsv["vmax"])
            mask = cv2.inRange(hsv_img, lower, upper)
            
            # Finn konturer på masken
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_SIMPLE)

            # Sjekk om det finnes konturer (ballen funnet)
            if contours:
                largest_contour = max(contours, key = cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                # Terskel for å filterere bort støy
                if area > 100:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cX = M["m10"] / M["m00"]
                        cY = M["m01"] / M["m00"]
                    else:
                        cX, cY = 0, 0 

                    # Beregn avstand i x, y i forhold til scontours[0]['center'][0]enterpunktet
                    x = cX - self.center_point[0]
                    y = (self.resolution[1] - cY) - self.center_point[1]
                    # Estimer z ut fra areal (størrelse) av konturen
                    z = (area - self.center_point[2]) / 1000

                    msg = Point(x=x, y=y, z=z)
                    self.publisher.publish(msg)

                    # Loggfører koordinatene til posisjon
                    self.get_logger().info(
                        f"Detected {name} kube at X: {x:.2f}, Y: {y:.2f}, Z: {x:.2f}"
                    )

                    found_colors.append(name)

        # Sjekk for ikke-detekterte objekter
        missing_object = [color for color in self.hsv_vals.keys() if color not in found_colors]

        if missing_object:
            error_msg = String()
            error_msg.data = f"Missing object: {', '.join(missing_object)}"
            self.error_publisher.publish(error_msg)
            self.get_logger().warn(error_msg.data)

def main(args=None):
    rclpy.init(args=args)
    detection_node = Detection()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
