import cv2  # Import the OpenCV library for computer vision.
import numpy as np  # Import NumPy for numerical operations.
import pyfirmata  # Import the pyFirmata library for Arduino communication.

# Initialize the video capture from the default camera.
cap = cv2.VideoCapture(0)

# Set the camera resolution to 1280x720 pixels.
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

# Check if the camera couldn't be accessed.
if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# Specify the COM port where the Arduino is connected.
port = "COM7"

# Create an Arduino board object to communicate with the Arduino.
board = pyfirmata.Arduino(port)

# Get a reference to pins 9 and 10 on the Arduino, configured as servo motors.
servo_pinX = board.get_pin('d:9:s')  # Pin 9 Arduino
servo_pinY = board.get_pin('d:10:s')  # Pin 10 Arduino

# Create a face detector using OpenCV's Haar Cascade Classifier.
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize the initial servo positions.
servoPos = [90, 90]

# Start an infinite loop to capture and process video frames.
while True:
    success, img = cap.read()  # Capture a frame from the camera.

    # Check if the frame was successfully captured.
    if not success:
        break

    # Convert the frame to grayscale for face detection.
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame using the Haar Cascade Classifier.
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # If faces are detected, update the servo positions to track the first detected face.
    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        fx, fy = x + w // 2, y + h // 2
        pos = [fx, fy]

        # Convert face coordinates to servo degrees.
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        # Ensure servo angles are within the valid range (0 to 180 degrees).
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))

        # Update servo positions.
        servoPos[0] = servoX
        servoPos[1] = servoY

        # Draw a circle and information on the tracked face.
        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx + 15, fy - 15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # Draw an x-axis line.
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # Draw a y-axis line.
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
    else:
        # Display a message when no face is detected.
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # Draw an x-axis line.
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # Draw a y-axis line.

    # Display the current servo positions.
    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    # Update the servo positions by writing to the Arduino pins.
    servo_pinX.write(servoPos[0])
    servo_pinY.write(servoPos[1])

    # Display the processed image.
    cv2.imshow("Image", img)

    # Wait for a key press and refresh the window at each iteration.
    cv2.waitKey(1)
