from dronekit import connect
import paho.mqtt.client as mqtt
import math

global client

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/controlling-drone")

    # #-- Connect to the vehicle
    print('Connecting...')
    global vehicle
    # vehicle = connect('localhost:14552', wait_ready=False)
    vehicle = connect('/dev/ttyUSB0', wait_ready=False)
    
    @vehicle.on_attribute("mode")
    def listener(self, attr_name, value):
        client.publish("/drone-status", "mode:" + vehicle.mode.name + "&armed:" + str(vehicle.armed))

    @vehicle.on_attribute("attitude")
    def attitude_change_listener(self, attr_name, value):
        client.publish("/drone-status", "pitch:" + "%.3f" % math.degrees(value.pitch) + "|roll:" + "%.3f" % math.degrees(value.roll) + "|yaw:" + "%.3f" % math.degrees(value.yaw))
        client.publish("/drone-status", "mode:" + vehicle.mode.name + "&armed:" + str(vehicle.armed))

    @vehicle.on_attribute("location.global_relative_frame")
    def global_relative_frame_change_listener(self, attr_name, value):
        client.publish("/drone-status", "lat:" + "%s" % value.lat + ";lng:" + "%s" % value.lon + ";alt:" + "%s" % value.alt)
    
client = mqtt.Client()

client.on_connect = on_connect
client.username_pw_set("bbff39d0d3066758ffe55666762b3c8b150295b848cb6c871b79f2fff36c79fb",
                       "50acea3098359517297e08040dc6bfc371d044190be6527c1ac29e078cbe8313")

client.connect("localhost", 1883, 60)
# client.connect("192.168.2.148", 1883, 60)

client.loop_forever()