import cv2
import numpy as np
import paho.mqtt.client as mqtt
import configparser
import cv2.aruco as aruco

print(cv2.__version__)

# Constants
MQTT_BROKER = "your_mqtt_broker_address"
MQTT_PORT = 1883
MQTT_USERNAME = "your_username"   # Replace with your MQTT username
MQTT_PASSWORD = "your_password"   # Replace with your MQTT password
IDENTITY = "RaspberryPi1"         # Unique ID for this device
TOPIC_STATUS = "/status"
TOPIC_DISPLAY = "/DISPLAY"
TOPIC_COMMAND = "/COMMAND"
TOPIC_MARKERS = "/MARKERS"

# Read configurations from config.ini
config = configparser.ConfigParser()
config.read('config.ini')

MQTT_BROKER = config['MQTT']['broker']
MQTT_PORT = int(config['MQTT']['port'])
MQTT_USERNAME = config['MQTT']['username']
MQTT_PASSWORD = config['MQTT']['password']

IDENTITY = config['IDENTITY']['id']

# Constants
TOPIC_STATUS = "/"+IDENTITY+"/STATUS"
TOPIC_DISPLAY = "/"+IDENTITY+"/DISPLAY"
TOPIC_COMMAND = "/"+IDENTITY+"/COMMAND"
TOPIC_MOTION = "/"+IDENTITY+"/MOTION"
TOPIC_ENV = "/"+IDENTITY+"/ENV"



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


start_aruco = True

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




def main():


    # Set username and password for MQTT broker
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connect(MQTT_BROKER, MQTT_PORT, 60)



    # Subscribe to the topic
    client.subscribe(TOPIC_COMMAND)


    while True:
        print("A")
        # Capture frame-by-frame from the camera
        ret, frame = cap.read()
        print("B")
        
        if not ret:
            raise Exception("Failed to grab frame")
        
        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ARUCO markers in the image
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        
        if start_aruco:
            if ids is not None:
                # Draw detected markers on the frame
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Optionally draw rejected markers
                cv2.aruco.drawDetectedMarkers(frame, rejectedImgPoints, borderColor=(0, 0, 255))
                
                # Print marker IDs and positions
                for i in range(len(ids)):
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
