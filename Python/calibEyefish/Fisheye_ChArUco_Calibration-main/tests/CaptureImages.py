import cv2
import numpy as np

# You can import CharucoCalibrator or FisheyeCalibrator depending on your needs
from calibrate import CharucoCalibrator  # Adjust the import path accordingly

# Initialize the CharucoCalibrator
aruco_dict = cv2.aruco.DICT_4X4_50  # You can adjust the dictionary if needed
squares_vertically = 5
squares_horizontally = 7
square_length = 0.04  # Adjust according to your board's square length (in meters)
marker_length = 0.02  # Adjust according to your board's marker length (in meters)

calibrator = CharucoCalibrator(
    aruco_dict=aruco_dict, 
    squares_vertically=squares_vertically, 
    squares_horizontally=squares_horizontally, 
    square_length=square_length, 
    marker_length=marker_length
)

# Set your RTSP URL here
rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
cap = cv2.VideoCapture(rtsp_url)  # Initialize capture from the RTSP stream

if not cap.isOpened():
    print("Error: Could not open RTSP stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image from RTSP stream")
        break

    # Show the ChArUco corners on the captured image
    calibrator.show_charuco_corners(frame, window_size=(640, 480), verbose=True)

    # Display the frame with detected ChArUco markers
    cv2.imshow("Capture Window", frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
