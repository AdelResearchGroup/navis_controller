import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2

class MapViewer(Node):
    def __init__(self):
        super().__init__("navis_map_viewer")
        self._map_topic_name = "navis/map_img"
        self._click_topic_name = "navis/click_pt"
        self._bridge = CvBridge()

        self.create_subscription(ImageMsg, self._map_topic_name, self._map_sub_cb, 10)
        self._click_pub = self.create_publisher(Point, self._click_topic_name, 10)

        self._gui_refresh_timer = self.create_timer(0.03, self._gui_update)

        cv2.namedWindow("map")
        cv2.setMouseCallback("map", self._mouse_cb)

        self._current_image = None

    def _map_sub_cb(self, msg):
        """Callback when the map image is published, setting that as the current image."""
        self._current_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imshow("map", self._current_image)
        cv2.waitKey(1)

    def _gui_update(self):
        """Updates the map image on a timer."""
        if self._current_image is not None:
            cv2.imshow("map", self._current_image)
        cv2.waitKey(1)

    def _mouse_cb(self, event, x, y, flags, param):
        """Called when the user clicks a point on the displayed map and publishes those coordinates."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # self.get_logger().info(f"Clicked at: ({x}, {y})")
            click_pt = Point()
            click_pt.x = float(x)
            click_pt.y = float(y)
            click_pt.z = 0.0
            self._click_pub.publish(click_pt)
    


def main():
    rclpy.init()
    node = MapViewer()

    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
