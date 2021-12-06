import time
import argparse
import socketio
import cv2
import base64 
from src.drone_control import Drone
from dronekit import VehicleMode 
import time, numpy as np, pathlib 
import paho.mqtt.client as mqtt

 
sio = socketio.Client()
camset = "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=4056, height=3040, format=(string)NV12, framerate=5/1 ! nvvidconv ! video/x-raw, width=800, height=600, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink "
drone = Drone(serial_address="/dev/ttyTHS1", baud=57600)
THRESHOLD_ALT = 0.3

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("gimbal")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    payload = msg.payload.decode('UTF-8')
    print("loftoff")
    if payload = "up":
        drone.vehicle.simple_takeoff(2)

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
    streamer = CVClient(server_addr).setup()
        
    video_stream = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)
            # Allow Webcam to warm up
    time.sleep(2.0)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(server_addr, 1883, 60)
        client.loop_start()
    except:
        print("Connection problems")

    while True:
        _, frame = video_stream.read()
                
        streamer.send_data(frame, "bonjour")

        if cv2.waitKey(1)==ord('q'):
            break
 
    cam.release()
    client.loop_stop()
    streamer.close()
    print("Program Ending")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Drone position and video feed streamer')
    
    parser.add_argument(
            '--server-addr',  type=str, default='localhost',
            help='The IP address or hostname of the SocketIO server.')
  
    
    args = parser.parse_args()
    main(args.server_addr)
