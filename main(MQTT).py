#Data is sent to VM on GCP over MQTT client - mosquitto

import RPi.GPIO as GPIO 
import time
import Adafruit_DHT
import json
import paho.mqtt.client as mqtt
from picamera2 import Picamera2
from PIL import Image
from classification import classify_image
import numpy as np

# Pin configuration
CAMERA_IR_SENSOR_PIN = 14  # GPIO pin connected to the camera-use IR sensor's OUT pin
BIN_IR_SENSOR_PIN = 20  # GPIO pin connected to the bin-use IR sensor's OUT pin 
DHT_SENSOR_PIN = 15  # GPIO pin connected to the DHT11 sensor
SERVO_PIN = 21  # GPIO pin connected to the servo signal
RED_LED_PIN = 19  # GPIO pin for red LED
GREEN_LED_PIN = 26  # GPIO pin for green LED

# MQTT Broker configuration
MQTT_BROKER = "34.121.233.234"
MQTT_PORT = 1883
MQTT_TOPIC = "ssrb"

# Initialize MQTT client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

# Initialize Picamera2
camera = Picamera2()
camera.start()

# Set up GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(CAMERA_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(BIN_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)  
GPIO.setup(GREEN_LED_PIN, GPIO.OUT) 

# DHT11 Sensor setup
DHT_SENSOR = Adafruit_DHT.DHT11

# Servo setup
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # Set servo frequency
servo_pwm.start(0)  # Start PWM with 0% duty cycle

# Variables for timing
last_capture_time = 0  # Tracks the last capture time for the object
detection_start_time = None  # Tracks when detection starts for the object
bin_detection_start_time = None  # Tracks when the bin-use IR sensor detects full
DETECTION_DURATION = 1  # Time (seconds) the object must be detected
COOLDOWN_PERIOD = 5  # Time (seconds) to wait before next capture
BIN_FULL_THRESHOLD = 5  # Time (seconds) the bin-use IR sensor must be active to consider the bin full

# Variable for bin full/not full
bin_full_sent = None  # Tracks the last sent bin full status

# Averaging parameters for Temp/Humidity data processing 
num_readings = 10 
temperature_readings = []
humidity_readings = []

def get_average_reading(readings, new_reading):
    if len(readings) >= num_readings:
        readings.pop(0)  # Remove the oldest reading
    readings.append(new_reading)
    return np.mean(readings)  

def set_servo_angle(angle):
    duty_cycle = 2 + (angle / 18)  # Convert angle (0-180) to duty cycle (2-12)
    GPIO.output(SERVO_PIN, True)
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    servo_pwm.ChangeDutyCycle(0)

def read_dht11():
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_SENSOR_PIN)
    if humidity is not None and temperature is not None:
        return humidity, temperature
    else:
        return None, None
    
def handle_bin_detection(bin_sensor_value, current_time):
    global bin_detection_start_time, bin_full_sent
    if bin_sensor_value == 0:  # Bin full detected
        if bin_detection_start_time is None:
            bin_detection_start_time = current_time
        elif current_time - bin_detection_start_time >= BIN_FULL_THRESHOLD:
            if bin_full_sent != 1:  # Send telemetry if not already sent
                client.publish("v1/devices/me/telemetry", json.dumps({"paperFull": 1}), qos=1)
                print("Bin is full! Sent telemetry: paperFull: 1")
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                bin_full_sent = 1
    else:  # Bin not full
        if bin_detection_start_time is not None:
            if current_time - bin_detection_start_time >= BIN_FULL_THRESHOLD and bin_full_sent != 0:
                client.publish("v1/devices/me/telemetry", json.dumps({"paperFull": 0}), qos=1)
                print("Bin is not full! Sent telemetry: paperFull: 0")
                GPIO.output(RED_LED_PIN, GPIO.LOW)
                GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
                bin_full_sent = 0
        bin_detection_start_time = None  # Reset detection start time

def handle_object_detection(camera_sensor_value, current_time):
    """Handles object detection logic."""
    global detection_start_time, last_capture_time
    if camera_sensor_value == 0:  # Object detected
        if detection_start_time is None:
            detection_start_time = current_time  
        elif current_time - detection_start_time >= DETECTION_DURATION:
            if current_time - last_capture_time >= COOLDOWN_PERIOD:
                capture_and_classify_object(current_time)
                detection_start_time = None  # Reset detection timer
    else:
        detection_start_time = None  # Reset detection if no object is detected


def capture_and_classify_object(current_time):
    filename = f"object.jpg"
    print(f"Object detected! Capturing image: {filename}")
    camera.capture_file(filename)
    original_img = Image.open(filename)
    oriented_img = original_img.transpose(method=Image.FLIP_TOP_BOTTOM).transpose(method=Image.FLIP_LEFT_RIGHT) # orient the image correctly
    oriented_img.save(filename)

    classification_result = classify_image(filename)
    print(f"Classification result: {classification_result}")

    # Send telemetry data
    telemetry_data = {
        "paper": 1 if classification_result == "paper" else 0,
        "plasticMetal": 1 if classification_result == "plasticMetal" else 0,
        "glass": 1 if classification_result == "glass" else 0,
        "trash": 1 if classification_result == "trash" else 0
    }
    client.publish(MQTT_TOPIC, json.dumps(telemetry_data), qos=1)
    print(f"Classification data sent to GCP: {telemetry_data}")

    # Control servo based on classification result
    if classification_result == "paper":
        print("Moving servo to position for paper")
        set_servo_angle(160)  
    elif classification_result == "glass":
        print("Moving servo to position for glass")
        set_servo_angle(110)  
    elif classification_result == "plasticMetal":
        print("Moving servo to position for plastic/metal")
        set_servo_angle(70)  
    else:  # Assume trash
        print("Moving servo to position for trash")
        set_servo_angle(20) 

    last_capture_time = current_time

# Component state Initialisation
set_servo_angle(180)
GPIO.output(RED_LED_PIN, GPIO.LOW)
GPIO.output(GREEN_LED_PIN, GPIO.HIGH)

def main():
    print("Monitoring IR sensors and DHT11 sensor...")
    while True:
        camera_sensor_value = GPIO.input(CAMERA_IR_SENSOR_PIN)
        bin_sensor_value = GPIO.input(BIN_IR_SENSOR_PIN)
        current_time = time.time()
        
        handle_bin_detection(bin_sensor_value, current_time) # type: ignore
        handle_object_detection(camera_sensor_value, current_time)
        
        # Read DHT11 sensor
        humidity, temperature = read_dht11()
        if humidity is not None and temperature is not None:
            avg_humidity = get_average_reading(humidity_readings, humidity)
            avg_temperature = get_average_reading(temperature_readings, temperature)
            telemetry_data = {
                "temperature": avg_temperature,
                "humidity": avg_humidity,
            }
            # Send data to GCP
            client.publish(MQTT_TOPIC, json.dumps(telemetry_data), qos=1)
            print(f"Temp/Humidity data sent to GCP: {telemetry_data}")
        else:
            print("Failed to read DHT11 sensor")
        
        time.sleep(1)  # Adjust as needed

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting program...")

    finally:
        client.loop_stop()
        client.disconnect()
        servo_pwm.stop()  # Stop PWM
        GPIO.output(RED_LED_PIN, GPIO.LOW) # Turn off red LED
        GPIO.output(GREEN_LED_PIN, GPIO.LOW) # Turn off green LED
        camera.close()  # Clean up the camera
        GPIO.cleanup()  # Clean up GPIO settings
