from flask_socketio import SocketIO
from flask import Flask, render_template, request
import json
from flask_mqtt import Mqtt
import eventlet
from flask_bootstrap import Bootstrap

eventlet.monkey_patch()
 
app = Flask(__name__)

app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['MQTT_BROKER_URL'] = '192.168.0.149'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''
app.config['MQTT_PASSWORD'] = ''
app.config['MQTT_KEEPALIVE'] = 5
app.config['MQTT_TLS_ENABLED'] = False

socketio = SocketIO(app)
bootstrap = Bootstrap(app)
mqtt = Mqtt(app)

@app.route('/')
def index():
    """Home page."""
    return render_template('index.html')

@app.route('/up')
def go_up():
    """Move up the gimbal"""
    mqtt.publish("gimbal","up")
    return ("nothing")

@app.route('/down')
def go_down():
    """Move down the gimbal"""
    mqtt.publish("gimbal","down")
    return ("nothing") 

@app.route('/send_destination')
def send_position():
    """Move down the gimbal"""
    json_data = request.args.to_dict()
    print(json_data)
    position = json.dumps({"x" : int(json_data["x"]), "y" : int(json_data["y"]), "z" : int(json_data["z"])})
    mqtt.publish("dwm/destination", position)
    return ("nothing") 

@socketio.on('connect', namespace='/web')
def connect_web():
    print('[INFO] Web client connected: {}'.format(request.sid))


@socketio.on('disconnect', namespace='/web')
def disconnect_web():
    print('[INFO] Web client disconnected: {}'.format(request.sid))


@socketio.on('connect', namespace='/cv')
def connect_cv():
    print('[INFO] CV client connected: {}'.format(request.sid))


@socketio.on('disconnect', namespace='/cv')
def disconnect_cv():
    print('[INFO] CV client disconnected: {}'.format(request.sid))


@socketio.on('cv2server')
def handle_cv_message(message):
    socketio.emit('server2web', message, namespace='/web')

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    mqtt.subscribe('dwm/#')

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
 
    socketio.emit('mqtt', data=data, namespace='/web')


@mqtt.on_log()
def handle_logging(client, userdata, level, buf):
    print(level, buf)

if __name__ == "__main__":
    print('[INFO] Starting server at http://localhost:5001')
    socketio.run(app=app, host='0.0.0.0', port=5001)
