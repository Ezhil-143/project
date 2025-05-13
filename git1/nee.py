import cv2
import mediapipe as mp
import serial
import time

# Initialize MediaPipe Hands and drawing utilities
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Start video capture
cap = cv2.VideoCapture(0)

# Initialize serial communication with Arduino
arduino = serial.Serial('COM6', 9600)  # Update with your correct COM port
time.sleep(2)  # Give some time for the serial connection to initialize

# Function to map hand position to servo angles and send commands
def control_robotic_arm(landmarks):
    command = ""
    
    # Base Servo (Horizontal movement based on wrist x-position)
    wrist_x = landmarks[0].x
    base_angle = int(180 * wrist_x)  # Map x-position to 0-180 degrees
    command += f"b{base_angle}"

    # Elbow Servo (Vertical movement based on wrist y-position)
    wrist_y = landmarks[0].y
    elbow_angle = int(90 * wrist_y)  # Map y-position to 0-90 degrees
    command += f"e{elbow_angle}"

    # Turning Arm Servo (Distance between thumb and index)
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    turning_arm_angle = int(260 * calculate_distance(thumb_tip, index_tip))  # Map to 0-180 degrees
    command += f"t{turning_arm_angle}"

    # Picker Servo (Controlled by pinky finger state)
    pinky_tip = landmarks[20]
    pinky_mcp = landmarks[17]  # MCP joint of the pinky
    if pinky_tip.y < pinky_mcp.y:  # If pinky is extended
        picker_angle = 90  # Open picker
    else:
        picker_angle = 45  # Close picker
    command += f"p{picker_angle}"

    # Send the full command to Arduino
    arduino.write((command + "\n").encode())
    print("Sent command:", command)

# Function to calculate distance between two points
def calculate_distance(point1, point2):
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5

with mp_hands.Hands(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
) as hands:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        # Convert the image to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame and detect hands
        result = hands.process(frame_rgb)
        # Draw hand landmarks if detected
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Control robotic arm based on hand landmarks
                control_robotic_arm(hand_landmarks.landmark)

        # Display the frame
        cv2.imshow('Hand Detection', frame)

        # Break the loop when 'q' key is pressed
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

arduino.close()  # Close the serial connection when done
