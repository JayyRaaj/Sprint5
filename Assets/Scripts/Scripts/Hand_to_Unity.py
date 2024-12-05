import cv2
import mediapipe as mp
import socket
import numpy as np

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Connect to Unity
HOST = '127.0.0.1'  # Localhost
PORT = 65432        # Unity listening port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Gesture Detection Functions
def detect_fist(hand_landmarks):
    fingers = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
               mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
               mp_hands.HandLandmark.RING_FINGER_TIP,
               mp_hands.HandLandmark.PINKY_TIP]
    return all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[tip - 2].y for tip in fingers)

def detect_open_hand(hand_landmarks):
    fingers = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
               mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
               mp_hands.HandLandmark.RING_FINGER_TIP,
               mp_hands.HandLandmark.PINKY_TIP]
    return all(hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y for tip in fingers)

def detect_pointing_up(hand_landmarks):
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    # Debugging output
    
    # Check if index is extended and thumb is not extended
    index_extended = index_tip.y < index_pip.y
    thumb_not_extended = thumb_tip.y > thumb_ip.y
    
    other_fingers_closed = all(
        hand_landmarks.landmark[tip].y > hand_landmarks.landmark[tip - 2].y 
        for tip in [mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                   mp_hands.HandLandmark.RING_FINGER_TIP,
                   mp_hands.HandLandmark.PINKY_TIP]
    )
    return index_extended and thumb_not_extended and other_fingers_closed

def detect_thumbs_down(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    # Debugging output
    return thumb_tip.y > thumb_ip.y  # Thumb lowered

def detect_thumbs_up(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
    return thumb_tip.y < thumb_ip.y  # Thumb raised

def detect_victory(hand_landmarks):
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    # Debugging output
    ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    
    # Check if index and middle fingers are extended
    index_middle_extended = (
        index_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y and
        middle_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
    )
    
    # Check if other fingers are closed
    other_fingers_closed = (
        ring_tip.y > hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y and
        pinky_tip.y > hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_FINGER_PIP].y
    )
    
    return index_middle_extended and other_fingers_closed

def detect_i_love_you(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    # Debugging output
    
    # Check if thumb, index, and pinky are extended
    thumb_extended = thumb_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y
    index_extended = index_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y
    pinky_extended = pinky_tip.y < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_FINGER_PIP].y
    
    # Check if middle and ring fingers are folded
    middle_folded = middle_tip.y > hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
    ring_folded = ring_tip.y > hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y
    
    return thumb_extended and index_extended and pinky_extended and middle_folded and ring_folded

# Video Capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip and convert to RGB
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process with MediaPipe Hands
    results = hands.process(rgb_frame)

    # Draw hand landmarks
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Detect gestures
            if detect_fist(hand_landmarks):
                sock.sendall(b"Closed_Fist")
                print("Closed Fist detected!")
            elif detect_open_hand(hand_landmarks):
                sock.sendall(b"Open_Palm")
                print("Open Palm detected!")
            elif detect_pointing_up(hand_landmarks):
                sock.sendall(b"Pointing_Up")
                print("Pointing Up detected!")
            elif detect_thumbs_down(hand_landmarks):
                sock.sendall(b"Thumb_Down")
                print("Thumb Down detected!")
            elif detect_thumbs_up(hand_landmarks):
                sock.sendall(b"Thumb_Up")
                print("Thumb Up detected!")
            elif detect_victory(hand_landmarks):
                sock.sendall(b"Victory")
                print("Victory detected!")
            elif detect_i_love_you(hand_landmarks):
                sock.sendall(b"ILoveYou")
                print("I Love You detected!")
            else:
                sock.sendall(b"0:Unknown")
                print("Unknown gesture detected!")

    # Display the frame
    cv2.imshow('Hand Gesture Recognition', frame)
    
    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
sock.close()
