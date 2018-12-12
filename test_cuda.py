

# subscribed Topic
sub_color = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.handle_color)

def handle_color(self, msg):
    try:
        img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        _, locs = detect(self.img_color, self.chessboard_corners)
    except CvBridgeError as e:
        print(e)