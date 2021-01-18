#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('/mavros/start_video_streaming', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    hello_str = '{\
    "janus_token": "1631121726,janus,janus.plugin.streaming:Xa2Yy/KbEzEfN1ghM252EF8XwDY=",\
    "video_port": 50848,\
    "audio_port": 58154,\
    "janus_feed_id": 1525969061,\
    "janus_feed_pin": 50575,\
    "streaming_server_ip": "192.168.1.5",\
    "streaming_server_port": "8088"\
}'
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
