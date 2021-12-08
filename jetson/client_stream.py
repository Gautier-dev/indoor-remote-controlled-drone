import time
import argparse
import socketio
import cv2
import base64 
from src.drone_control import Drone
from dronekit import VehicleMode 
import time, numpy as np, pathlib 
import paho.mqtt.client as mqtt
import math
import json
 
sio = socketio.Client()
camset = "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=4056, height=3040, format=(string)NV12, framerate=5/1 ! nvvidconv ! video/x-raw, width=800, height=600, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink "
drone = Drone(serial_address="/dev/ttyTHS1", baud=57600)
THRESHOLD_ALT = 0.3
x_current, y_current = 0,0

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("dwm/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    payload = str(msg.payload.decode('UTF-8'))
    json_data = json.loads(payload) 
    if msg.topic == "dwm/destination":
        if (drone.vehicle.location.global_relative_frame.alt < 0.3):  # Take off if landed
            takeoff_alt = 1.5
            drone.vehicle.simple_takeoff(takeoff_alt)
            time.sleep(5)
        x_destination, y_destination = json_data["x"], json_data["y"] # meters
        x = x_destination-x_current
        y = y_destination-y_current
        yaw = drone.vehicle.attitude.yaw
        drone.send_global_velocity(
                        x * np.cos(yaw) - y * np.sin(yaw),
                        x * np.sin(yaw) + y * np.cos(yaw),
                        0,
                        math.sqtr(x**2 + y**2),
                    )
        drone.send_global_velocity(0, 0, 0, 1)
    if msg.topic == "dwm/node/19f2/uplink/location":
        global x_current
        global y_current
        x_current = int(json_data["position"]["x"])
        y_current = int(json_data["position"]["y"])


@sio.event
def connect():
    print('[INFO] Successfully connected to server.')


@sio.event
def connect_error():
    print('[INFO] Failed to connect to server.')


@sio.event
def disconnect():
    print('[INFO] Disconnected from server.')


class CVClient(object):
    def __init__(self, server_addr):
        self.server_addr = server_addr
        self.server_port = 5001
       
        self._last_update_t = time.time()
        self._wait_t = (1/5)

    def setup(self):
        print('[INFO] Connecting to server http://{}:{}...'.format(
            self.server_addr, self.server_port))
        sio.connect(
                'http://{}:{}'.format(self.server_addr, self.server_port),
                transports=['websocket'],
                namespaces=['/cv'])
        time.sleep(1)
        return self

    def _convert_image_to_jpeg(self, image):
        # Encode frame as jpeg
        print(type(image))
        frame = cv2.imencode('.jpg', image)[1].tobytes()
        # Encode frame in base64 representation and remove
        # utf-8 encoding
        frame = base64.b64encode(frame).decode('utf-8')
        return "data:image/jpeg;base64,{}".format(frame)

    def send_data(self, frame, text):
        cur_t = time.time()
        if cur_t - self._last_update_t > self._wait_t:
            self._last_update_t = cur_t
            
            sio.emit(
                    'cv2server',
                    {
                        'image': self._convert_image_to_jpeg(frame),
                        'text': '<br />'.join(text)
                    })

    def check_exit(self):
        pass

    def close(self):
        sio.disconnect()


def main(server_addr):
    streamer = CVClient("192.168.0.185").setup()
        
    video_stream = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)
            # Allow Webcam to warm up
    time.sleep(2.0)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect("192.168.0.149", 1883, 60)
        client.loop_start()
    except:
        print("Connection problems")

    while True:
        _, frame = video_stream.read()
                
        streamer.send_data(frame, "bonjour")

        if cv2.waitKey(1)==ord('q'):
            break
 
    video_stream.release()
    client.loop_stop()
    streamer.close()
    print("Program Ending")


if __name__ == "__main__":

    main()
