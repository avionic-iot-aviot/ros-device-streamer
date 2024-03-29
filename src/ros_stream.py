#!/home/pi/catkin_ws/src/ros-device-streamer/venv/bin/python
#/usr/bin/python
import rospy
import json
import subprocess
import os
import signal
import logging
import logging.handlers
import argparse
import requests
import PWM

from std_msgs.msg import String
from datetime import datetime
from enum import Enum

# ROS DEVICE STREAMER
from Packets import Packets
from configparser import ConfigParser
import ifaddr
import socket

import threading
from threading import Timer

DEBUG = True

lock = threading.Lock()
volume_timer=None
volume_multiplier=12

parser = argparse.ArgumentParser(
        description='Notify status change to MQTT broker'
        )

parser.add_argument(
        '-c',
        '--config',
        metavar="config_file",
        action="store",
        type=argparse.FileType('r'),
        required=True,
        # metavar="range_time_criterion",
        help='file that contains config parameter'
)

# parsing parameter
args = parser.parse_args()
config_file = args.config

# loading configurations ...

json_file = config_file.read()
configs = json.loads(json_file)

status_path = configs["device_status_path"]
status_path = os.path.expanduser(status_path)


# loading mac from mac file
mac_path = configs["mac_path"]
mac_path = os.path.expanduser(mac_path)

mac=open(mac_path, 'r').read()
mac=mac.strip().replace(':','').lower()

# auth
username = configs["username"]
password = configs["password"]

# endpoint API
API_SCHEMA = configs["api_schema"]
API_HOST = configs["api_host"]
API_PORT = configs["api_port"]

AUDIO_CARD = configs["audio_card"]

DOMAIN = configs["domain"]
SERIAL = "serial_"+mac

device_id = "rpi_"+mac

topic_name = "gw_"+mac

log_file = configs["log_file"]
log_file = os.path.expanduser(log_file)


endpoint_auth = "/user/auth"
access_token = None

# video streaming path
API_VERSION = "/api/v1.0"
API_URL = API_SCHEMA + "://" + API_HOST + ":" + str(API_PORT) + API_VERSION

feed_id = ""

status = Enum("status", "streaming videoroom idle")

def get_datetime():

    now = datetime.now()
    return now.strftime("%Y-%m-%d %H:%M:%S")

def set_status(s):
    global status_path

    with open(status_path, "w") as f:
        logging.info(
            "[se-status]: set new status => %s" % s
        )
        f.write(s)


def kill_ffmpeg_streaming():

    try:
        logging.info("[kill_ffmpeg_streaming]: kill ffmpeg tool...")
        p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
        out, err = p.communicate()
        for line in out.splitlines():
            if "ffmpeg" in line.decode('utf-8'):
                pid = int(line.split(None, 1)[0])
                print(pid)
                os.kill(pid, signal.SIGKILL)
    except:
        raise
    finally:
        set_status(status.idle.name)


def start_ffmpeg_streaming(server_ip, video_port, audio_port):

    logging.info("[start_ffmpeg_streaming]: starting ffmpeg...")
    cmd_audio_stream = ""

    # FIXME gestire errori
    cmd_video_stream = \
        "/usr/bin/ffmpeg -f v4l2 -s 320x240 -re -i /dev/video0 -pix_fmt yuv420p " \
        "-c:v libx264 -profile:v baseline -level 3.0 -preset ultrafast " \
        "-tune zerolatency " \
        "-movflags +faststart " \
        "-an " \
        "-f rtp rtp://" + str(server_ip) + ":" + str(video_port)

    # check if audio port is set
    if audio_port != 0:
        cmd_audio_stream = "-re -f alsa -ar 44100 -i plughw:CARD="+AUDIO_CARD+",DEV=0 -vn -codec:a opus -strict -2 " \
                           " -f rtp rtp://" + str(server_ip) + ":" + str(audio_port)

    cmd_stream = cmd_video_stream + " " + cmd_audio_stream

    logging.debug(
        "[start_ffmpeg_streaming]: I'm executing this command => '%s'" % cmd_stream)
    proc = subprocess.Popen(
        cmd_stream,
        #cmd_video_stream,
        shell=True
    )
    print(cmd_stream)
    set_status(status.streaming.name)

    return proc


def stop_uv4l():

    logging.info("[stop_uv4l]: stop uv4l...")
    res = requests.get(
        "http://localhost:8090/janus",
        {
            "action": "Stop"
        }
    )

    set_status(status.idle.name)
    return res


def start_uv4l(vr_name, vr_pin, vr_usrnm, vr_secret, vr_server, vr_port):

    logging.info("[start_uv4l]: starting uv4l, I'm communicating with Janus Gateway...")
    res = requests.get(
        "http://localhost:8090/janus",
        {
            "gateway_url": "http://%s:%s" % (vr_server, vr_port),
            "gateway_root": "/janus",
            "room": vr_name,
            "room_pin": "" + str(vr_pin),
            "username": vr_usrnm,
            "proxy_host": "",
            "proxy_port": 80,
            "proxy_password": "",
            "proxy_bypass": "",
            "token": "",
            "publish": 1,
            "subscribe": 1,
            "hw_vcodec": 0,
            "vformat": 60,
            "reconnect": 1,
            "action": "Start"
        }
    )
    set_status(status.videoroom.name)
    return res


def create_videoroom_endpoint():
    pass


def destroy_videoroom_endpoint():
    pass


def create_stream_endpoint(
        audio=False,
        create_endpoint=False,
        feed_video_port=None,
        feed_audio_port=None,
        janus_feed_id=None,
        janus_feed_pin=None,
        janus_token=None,
        streaming_server_ip=None,
        streaming_server_port=None
):
    global feed_id, pid_proc, access_token

    feed_id = janus_feed_id
    feed_pin = janus_feed_pin
    token = janus_token

    if create_endpoint:

        logging.info("[create_stream_endpoint]: creating stream endpoint...")

        api_endpoint = "/streaming_endpoint"
        _params = {
            "serial": SERIAL,
            "audio": audio
        }

        logging.debug("[create_stream_endpoint]: POST request => URL: %s, params: %s..." %
                      (API_URL + api_endpoint, _params))

        # FIXME gestire gli errori
        login()
        headers = {
            "Authorization": "Bearer %s" % access_token,
            "Content-Type": "application/json"
        }
        r = requests.post(
            url=API_URL + api_endpoint,
            json=_params,
            headers=headers,
            verify=False
        )

        # TODO create item on DB
        if r.status_code == 201:
            data = r.json()

            logging.debug("[create_stream_endpoint]: request RESPONSE => URL: %s, data: %s..." %
                          (API_URL + api_endpoint, data))

            token = data['janus_token']
            feed_id = data['janus_feed_id']
            feed_pin = data['janus_feed_pin']
            feed_video_port = data['video_port']
            feed_audio_port = data['audio_port']
            streaming_server_ip = data['streaming_server_ip']
            streaming_server_port = data['streaming_server_port']

            start_ffmpeg_streaming(
                streaming_server_ip,
                feed_video_port,
                feed_audio_port
            )


    else:

        start_ffmpeg_streaming(
            streaming_server_ip,
            feed_video_port,
            feed_audio_port
        )

    # notify all to new live streaming...
    payload = {
                'device': SERIAL,
                'timestamp': get_datetime(),
                'streaming_state': 'started',
                'feed_id': feed_id,
                'context_id': SERIAL,
                'feed_pin': feed_pin,
                'streaming_server_ip': streaming_server_ip,
                'streaming_server_port': streaming_server_port,
                'token': token
            }

    logging.debug(
                "[create_stream_endpoint]: publish MQTT message => topic: %s, payload: %s " % (mqtt_pub_ls_topic, payload)
            )
    # send message to MQTT for notifying new streaming endpoint
    '''
    client_mqtt.publish(
        mqtt_pub_ls_topic,
        payload=json.dumps(payload),
        qos=0,  # QoS
        retain=True  # retain
    )
    '''

def destroy_stream_endpoint():
    global feed_id, access_token

    logging.info("[destroy_stream_endpoint]: destroy stream endpoint...")

    api_endpoint = "/streaming_endpoint"

    login()
    headers = {
        "Authorization": "Bearer %s" % access_token,
        "Content-Type": "application/json"
    }

    r = requests.delete(
        url=API_URL + api_endpoint + "/" + str(feed_id),
        headers=headers,
        verify=False
    )

    kill_ffmpeg_streaming()

    payload = {
                'serial': SERIAL,
                'timestamp': get_datetime(),
                'streaming_state': 'finished',
            }
    logging.debug(
        "[destroy_stream_endpoint]: publish MQTT message => topic: %s, payload: %s" % (mqtt_pub_ls_topic, payload)
    )

    # send message to MQTT for notifying that streaming has been destroyed
    '''
    client_mqtt.publish(
        mqtt_pub_ls_topic,
        payload=json.dumps(payload),
        qos=0,
        retain=True
        )
    '''
    print(r)


def on_mqtt_connect(client, userdata, flags, rc):

        logging.info("[on_mqtt_connect]: Connected with result code " + str(rc))


def on_mqtt_message(client, userdata, msg):

    payload = msg.payload.decode("utf-8")


    # TODO gestire errori
    payload_data = json.loads(payload)

    logging.debug(
        "[on_mqtt_message]: topic => %s, payload => %s" % (msg.topic, payload_data)
    )

    if payload_data['op'] == "start-streaming":
        logging.info(
            "[on_mqtt_message]: 'start-streaming' command has received!"
        )

        create_stream_endpoint(
            payload_data['audio'],
            payload_data['create_endpoint'],
            payload_data['video_port'],
            payload_data['audio_port'],
            payload_data['feed_id'],
            payload_data['feed_pin'],
            payload_data['token'],
            payload_data['streaming_server_ip'],
            payload_data['streaming_server_port']
        )

    if payload_data['op'] == "stop-streaming":
        logging.info(
            "[on_mqtt_message]: 'stop-streaming' command has received!"
        )
        destroy_stream_endpoint()

    if payload_data['op'] == "start-videoroom":
        logging.info(
            "[on_mqtt_message]: 'start-videoroom' command has received!"
        )

        start_uv4l(
            payload_data['videoroom_name'],
            payload_data['videoroom_pin'],
            payload_data['videoroom_usrnm'],
            payload_data['videoroom_secret'],
            payload_data['videoroom_server'],
            payload_data['videoroom_port']
        )

    if payload_data['op'] == "stop-videoroom":
        logging.info(
            "[on_mqtt_message]: 'stop-videoroom' command has received!"
        )
        stop_uv4l()

def start_video_streaming_callback(data):
    print("start video streaming")
    cmd = json.loads(data.data)
    start_ffmpeg_streaming(cmd["streaming_server_ip"], cmd["video_port"], cmd["audio_port"])

def stop_video_streaming_callback(data):
    print("stop video streaming")
    kill_ffmpeg_streaming()


'''
{
    "janus_token": "1634020933,janus,janus.plugin.videoroom:skQjCNX/wDMeMpPYuHZzTtdJ/+k=",
    "videoroom_name": 57908,
    "videoroom_pin": 2347487208,
    "videoroom_secret": "PfMHONjHSFWRQbXTUEYQ",
    "streaming_server_ip": "192.168.1.6",
    "streaming_server_port": "8088"
}
start_uv4l(vr_name, vr_pin, vr_usrnm, vr_secret, vr_server, vr_port):
'''
def start_video_room_callback(data):
    print("Starting video room uv4l")
    cmd = json.loads(data.data)
    print(data)
    start_uv4l(cmd["videoroom_name"], cmd["videoroom_pin"], device_id, cmd["videoroom_secret"], cmd["streaming_server_ip"], cmd["streaming_server_port"])

def stop_video_room_callback(data):
    print("Stopping video room uv4l")
    stop_uv4l()

def rtt_test_callback(data):
    print("Received rtt_test msg")
    print("Sending rtt_resp msg")
    pub_rtt.publish(data)

def servo_open_callback(data):
    cmd = json.loads(data.data)
    PWM.pwm(cmd["number"],'open')
def servo_close_callback(data):
    cmd = json.loads(data.data)
    PWM.pwm(cmd["number"],'close')

def set_volume_callback(data):
    with lock:
        global volume_multiplier
        
        cmd = json.loads(data.data)
        volume_multiplier=int(cmd["volume"])

        if volume_multiplier < 1:
            volume_multiplier = 1
        elif volume_multiplier > 50:
            volume_multiplier = 50

        print("Set volume: "+str(volume_multiplier))

def send_volume():
    with lock:
        pub_volume.publish(str(volume_multiplier))
    set_volume_timer()

def set_volume_timer():
    volume_timer=Timer(1,send_volume)
    volume_timer.start()

# ROS-DEVICE-STREAMER

config = ConfigParser()
config.read('./config.ini')

def GetIp(interface):
    adapters = ifaddr.get_adapters()

    temp = ""
    for adapter in adapters:
        if (adapter.nice_name == interface):
            temp = adapter.ips[0].ip
    
    return temp


if __name__ == '__main__':

    log_path = log_file if DEBUG else "/dev/null"
    root_logger = logging.getLogger()
    handler = logging.FileHandler(log_path)
    formatter = logging.Formatter(
        fmt='%(asctime)s ss[%(process)d] %(levelname)-8s: %(message)s',
        datefmt='%Y-%m-%d %H:%M'
    )
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)
    root_logger.setLevel(logging.DEBUG)
    #start_ffmpeg_streaming('192.168.1.5', 50848, 58154)
    print("Init ros node")
    rospy.init_node(device_id)

    print("Creating a publisher for rtt response...")
    pub_rtt = rospy.Publisher("/"+topic_name+"/rtt_resp", String, queue_size=10)
    print("Creating a publisher for volume...")
    pub_volume = rospy.Publisher("/"+topic_name+"/volume", String, queue_size=10)
    print("Subscribing to video streaming...")
    rospy.Subscriber("/"+topic_name+"/start_video_streaming", String, start_video_streaming_callback)
    rospy.Subscriber("/"+topic_name+"/stop_video_streaming", String, stop_video_streaming_callback)
    print("Subscribing to video room...")
    rospy.Subscriber("/"+topic_name+"/start_video_room", String, start_video_room_callback)
    rospy.Subscriber("/"+topic_name+"/stop_video_room", String, stop_video_room_callback)
    print("Subscribing to rtt test...")
    rospy.Subscriber("/"+topic_name+"/rtt_test", String, rtt_test_callback)
    print("Subscribing to servo open...")
    rospy.Subscriber("/"+topic_name+"/servo/open", String, servo_open_callback)
    print("Subscribing to servo close...")
    rospy.Subscriber("/"+topic_name+"/servo/close", String, servo_close_callback)
    print("Subscribing to set volume...")
    rospy.Subscriber("/"+topic_name+"/set_volume", String, set_volume_callback)

    print("Ok")
    #rospy.spin()

    print('audio-receiver started')
    # Create a UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Get IP
    ip=GetIp(config['GENERAL']['InterfaceRasp'])
    # Bind the socket to the port
    server_address = (ip, int(config['GENERAL']['PortRasp']))
    s.bind(server_address)


    set_volume_timer()

    while True:
        try:
            ##print("####### Node is listening #######")
            data, address = s.recvfrom(8192)
            #if len(data) < 5:
            #    try:
            #        new_volume_multiplier = int(data)
            #        if 1 < new_volume_multiplier < 50:
            #            volume_multiplier = new_volume_multiplier
            #        elif new_volume_multiplier <= 1:
            #            volume_multiplier = 1
            #        else:
            #            volume_multiplier = 50
            #        print("New volume multiplier is: {}".format(volume_multiplier))
            #    except:
            #        print("Received a wrong value for volume_multiplier.")
            #    continue
            packet = Packets.getPacketFromBytes(data)

            if (packet.Destination == ip):
                if(int(packet.Type) == 2):
                    elements = bytearray(packet.Payload)
                    bytes_to_write = bytearray()
                    temp_bytes = bytearray()
                    bytes_effectively_written = 0

                    i = 0
                    # The format of the bytes in the packet is as follow:
                    # 0 s1msb s1lsb 0 s2msb s2lsb 0 .... 0 sNmsb sNlsb
                    while i < len(elements):
                        if elements[i] != 48: # our separator is b'/x30' which is 48 in ASCII (literal '0')
                            temp_bytes.extend(elements[i:i+1])
                            
                        elif elements[i] == 48:
                            if len(temp_bytes) == 2:
                                integer = int.from_bytes(temp_bytes, "big", signed="True")
                                integer *= volume_multiplier
                                try:
                                    new_bytes = integer.to_bytes(2, 'big', signed=True)
                                except:
                                    if integer > 0:
                                        integer = 30000
                                    else:
                                        integer = -30000
                                    new_bytes = integer.to_bytes(2, 'big', signed=True)
                                bytes_to_write.extend(new_bytes)
                                bytes_effectively_written += 2
                            temp_bytes = bytearray()
                        i += 1
                    
                    
                    # This is to recover the last sample "sNmsb sNlsb"
                    if len(temp_bytes) == 2:
                            bytes_to_write.extend(temp_bytes[0:2])
                            bytes_effectively_written += 2        
                    
                    if len(bytes_to_write) > 0:
                        # We don't write if we are not receiving (or 5 times neutral writing have been executed)  
                        #print("Byte-Writer: byte sent ", len(bytes_to_write))
                        f = open("/dev/shm/{}.bin".format(packet.Source), "ab")
                        f.write(bytes_to_write)
                        f.close()
                    
                    #print("audio-receiver: Data Received from: ", packet.Source, ' Size: ', len(elements), ' Effect: ', bytes_effectively_written)
        except Exception as e:
            print(e)
