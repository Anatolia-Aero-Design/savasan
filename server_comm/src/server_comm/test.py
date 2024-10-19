import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Gstreamer pipeline for streaming over UDP
gst_pipeline = 'appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000'

# Open the gstreamer pipeline for writing
out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, 30, (640, 480), True)

def image_callback(image_msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    if frame is not None:
        out.write(frame)  # Stream frame via gstreamer

if __name__ == "__main__":
    rospy.init_node('video_stream_node')
    rospy.Subscriber('/camera_topic', Image, image_callback)
    rospy.spin()
