import cv2
#import cvzone
#from cvzone.ColorModule import ColorFinder
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
        self.subscription  # prevent unused variable warning

        # Publish object position
        self.publisher = self.create_publisher(
            Image,
            'object_position',
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
            "Rød": {'hmin': 160, 'smin': 144, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 0},
            "Blå": {'hmin': 111, 'smin': 0, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 255},
            "Grønn": {'hmin': 31, 'smin': 46, 'vmin': 0, 'hmax': 179, 'smax': 76, 'vmax': 185}
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

        # Prosesser bildet for alle farger i hsvVals og slå sammen maskene
        mask = None
        for name, hsv in self.hsv_vals.items():
            _, currentMask = self.color_finder.update(img, hsv)
            if mask is None:
                mask = currentMask
            else:
                mask = cv2.bitwise_or(mask, currentMask)

        # Finn konturer på masken
        img_contour, contours = cv2.findContours(img, mask, cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_SIMPLE)

        # Sjekk om det finnes konturer (ballen funnet)
        if contours:
            # Beregn avstand i x, y i forhold til senterpunktet
            x = contours[0]['center'][0] - self.center_point[0]
            y = (self.resolution[1] - contours[0]['center'][1]) - self.center_point[1]
            # Estimer z ut fra areal (størrelse) av konturen
            z = (contours[0]['area'] - self.center_point[2]) / 1000

            msg = Point(x=x, y=y, z=z)
            self.publisher.publish(msg)

            found_colors.append(name)

        # Sjekk for ikke-detekterte objekter
        missing_object = [color for color in self.hsv_vals.keys() if color not in found_colors]

        if missing_object:
            error_msg = String()
            error_msg.data = f"Mangler objekt med farge: {", ". join(missing_object)}"
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
