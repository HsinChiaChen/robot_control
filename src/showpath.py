import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ShowPath:

  def __init__(self):
    rospy.init_node("showpath")
    self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    self.rate = rospy.Rate(20)
    self.listener = tf.TransformListener()

  def show_path(self):
    """
    Publishes a marker to show the path of the robot.
    """
    while not rospy.is_shutdown():
      transform = None
      try:
        (trans, rot) = self.listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      if transform is not None:
        x = round(trans[0], 6)
        y = round(trans[1], 6)

        point = Point()
        point.x = x
        point.y = y
        point.z = 0.1

        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "showpath"
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.id = 0
        marker.type = Marker.POINTS
        marker.scale.x = 0.05
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.points.append(point)

        self.marker_pub.publish(marker)

      self.rate.sleep()

if __name__ == "__main__":
  show_path = ShowPath()
  show_path.show_path()