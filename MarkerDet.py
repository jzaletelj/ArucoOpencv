import time
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import configparser
import cv2.aruco as aruco
import os



# Constants
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_USERNAME = ""   # Replace with your MQTT username
MQTT_PASSWORD = ""   # Replace with your MQTT password
IDENTITY = "RaspberryPi1"         # Unique ID for this device
TOPIC_STATUS = "/status"
TOPIC_DISPLAY = "/DISPLAY"
TOPIC_COMMAND = "/COMMAND"
TOPIC_MARKERS = "/MARKERS"
DEBUG = True

def load_configuration():
    global MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD, IDENTITY
    global TOPIC_STATUS, TOPIC_DISPLAY, TOPIC_COMMAND, TOPIC_MOTION, TOPIC_ENV, DEBUG

    # Default configuration
    default_config = {
        'MQTT': {
            'broker': 'broker.hivemq.com',
            'port': '1883',
            'username': '',
            'password': ''
        },
        'IDENTITY': {
            'id': 'RaspberryPi1'
        },
        'DEBUG': {
            'debug': True
        },
        'TOPICS': {
            'status': '/status',
            'display': '/DISPLAY',
            'command': '/COMMAND',
            'motion': '/MOTION',
            'env': '/ENV'
        }
    }

    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config.ini')

    config = configparser.ConfigParser()
    
    try:
        if os.path.exists(config_path):
            config.read(config_path)
            print(f"Configuration loaded from {config_path}")
        else:
            print(f"Warning: Configuration file not found at {config_path}, using defaults")
            # Create sections from default config
            for section, values in default_config.items():
                config[section] = values
            
            # Optionally save default config
            with open(config_path, 'w') as configfile:
                config.write(configfile)
                print(f"Created default configuration file at {config_path}")
    except Exception as e:
        print(f"Error handling configuration: {str(e)}, using defaults")
        # Create sections from default config
        for section, values in default_config.items():
            config[section] = values

    try:
        # MQTT Section
        MQTT_BROKER = config.get('MQTT', 'broker', fallback=default_config['MQTT']['broker'])
        MQTT_PORT = config.getint('MQTT', 'port', fallback=int(default_config['MQTT']['port']))
        MQTT_USERNAME = config.get('MQTT', 'username', fallback=default_config['MQTT']['username'])
        MQTT_PASSWORD = config.get('MQTT', 'password', fallback=default_config['MQTT']['password'])

        # Identity Section
        IDENTITY = config.get('IDENTITY', 'id', fallback=default_config['IDENTITY']['id'])

        # Update topics with identity
        TOPIC_STATUS = f"/{IDENTITY}/STATUS"
        TOPIC_DISPLAY = f"/{IDENTITY}/DISPLAY"
        TOPIC_COMMAND = f"/{IDENTITY}/COMMAND"
        TOPIC_MOTION = f"/{IDENTITY}/MOTION"
        TOPIC_ENV = f"/{IDENTITY}/ENV"
        
        DEBUG = config.getboolean('DEBUG', 'debug', fallback=default_config['DEBUG'])

        print(f"Configuration loaded successfully. Using MQTT broker: {MQTT_BROKER}, Identity: {IDENTITY}")
        
    except Exception as e:
        print(f"Error applying configuration: {str(e)}, using defaults")

# Load configuration at startup
load_configuration()

if DEBUG:
    print(f"Debug mode is enabled. MQTT broker: {MQTT_BROKER}, Identity: {IDENTITY}")
    print("OpenCV version: " + cv2.__version__)

# Initialize the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise Exception("Could not open video device")

# Define the dictionary for ARUCO markers
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# Create a detector object for ARUCO markers
parameters = cv2.aruco.DetectorParameters() #ArucoDetector() //DetectorParameters_create()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

print("OK ..")

client = mqtt.Client()

def calculate_marker_pose(corners):
    """
    Calculates the center coordinates and orientation angle of an ARUCO marker.

    Args:
        corners (numpy.ndarray): A numpy array of shape (1, 4, 2) representing the corners of the marker.

    Returns:
        tuple: A tuple containing:
            - center (tuple): The (x, y) coordinates of the marker's center.
            - angle (float): The orientation angle of the marker in degrees,
                             relative to the horizontal axis.
    """
    # Calculate center
    center_x = np.mean(corners[0, :, 0])
    center_y = np.mean(corners[0, :, 1])
    center = (center_x, center_y)

    # Calculate orientation (angle)
    # Use the first two corners to determine the angle
    corner1 = corners[0, 0]
    corner2 = corners[0, 1]
    dx = corner2[0] - corner1[0]
    dy = corner2[1] - corner1[1]
    angle_rad = np.arctan2(dy, dx)
    angle_deg = np.degrees(angle_rad)

    return center, angle_deg



def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")
    
    client.subscribe(TOPIC_DISPLAY)
    client.subscribe(TOPIC_COMMAND)

# Callback function to handle received messages
def on_message(client, userdata, message):
    global start_aruco
    payload = str(message.payload.decode("utf-8"))
    command_data = eval(payload)
    
    if "start_aruco" in command_data and command_data["start_aruco"] == 1:
        start_aruco = True
    else:
        start_aruco = False


# Global variable to control ARUCO detection
start_aruco = True

# Main function
def main():
    
    # Set username and password for MQTT broker
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # Subscribe to the topic
    client.subscribe(TOPIC_COMMAND)

    delay = 0.05

    while True:

        if start_aruco:        
            # Capture frame-by-frame from the camera
            ret, frame = cap.read()
            
            
            if not ret:
                raise Exception("Failed to grab frame")
            
            # Convert the image to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect ARUCO markers in the image
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        
            if ids is not None:
                # Draw detected markers on the frame
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Optionally draw rejected markers
                cv2.aruco.drawDetectedMarkers(frame, rejectedImgPoints, borderColor=(0, 0, 255))
                
                # Print marker IDs and positions
                for i in range(len(ids)):
                    if DEBUG:
                        print(f"Marker ID: {ids[i]}, Position: {corners[i]}")
                    
                # calculate_marker_pose
                center_list = []
                angle_list = []
                for i in range(len(ids)):
                    center, angle = calculate_marker_pose(corners[i])
                    center_list.append(center)
                    angle_list.append(angle)
                    #print(f"Marker ID: {ids[i]}, Center: {center}, Angle: {angle} degrees")

                # Publish detected ARUCO markers as JSON string
                for i in range(len(ids)):
                    marker_data = {
                        "id": ids[i][0],
                        "position": corners[i].tolist(),
                        "center": center_list[i],
                        "angle": angle_list[i]
                    }
                    client.publish(TOPIC_MARKERS, str(marker_data))
        
            # Display the resulting frame
            cv2.imshow('ARUCO Marker Detection', frame)
        
        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if not start_aruco:
            delay = 0.5
        
        time.sleep(delay)

    # When everything done, release the capture and close all windows
    cap.release()
    cv2.destroyAllWindows()

    # Disconnect from the MQTT server
    client.disconnect()
    
    
if __name__ == "__main__":
    client.loop_start()  # Start MQTT loop
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")
