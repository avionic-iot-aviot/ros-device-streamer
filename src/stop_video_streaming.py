#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('/mavros/stop_video_streaming', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    hello_str = '{"janus_feed_id": 1525969061}'
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
