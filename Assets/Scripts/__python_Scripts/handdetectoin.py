import Leap
import socket
import time
from Leap import Controller, Frame, Hand, Finger

# Connect to Unity
HOST = '127.0.0.1'  # Localhost
PORT = 65432        # Unity listening port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

class GestureListener(Leap.Listener):
    def on_init(self, controller):
        print("Initialized")

    def on_connect(self, controller):
        print("Connected")

    def on_disconnect(self, controller):
        print("Disconnected")

    def on_exit(self, controller):
        print("Exited")

    def detect_fist(self, hand):
        # Check if all fingers are curled
        for finger in hand.fingers:
            if finger.length > 20:  # If any finger is extended
                return False
        return True

    def detect_open_hand(self, hand):
        # Check if all fingers are extended
        extended_fingers = 0
        for finger in hand.fingers:
            if finger.is_extended:
                extended_fingers += 1
        return extended_fingers >= 4

    def detect_pointing_up(self, hand):
        # Check if only index finger is extended
        index_extended = hand.fingers[1].is_extended
        other_fingers_folded = all(not finger.is_extended 
                                 for i, finger in enumerate(hand.fingers) 
                                 if i != 1)
        return index_extended and other_fingers_folded

    def detect_victory(self, hand):
        # Check if index and middle fingers are extended
        index_extended = hand.fingers[1].is_extended
        middle_extended = hand.fingers[2].is_extended
        other_fingers_folded = all(not finger.is_extended 
                                 for i, finger in enumerate(hand.fingers) 
                                 if i not in [1, 2])
        return index_extended and middle_extended and other_fingers_folded

    def detect_thumbs_up(self, hand):
        # Check thumb position and orientation
        thumb = hand.fingers[0]
        palm_normal = hand.palm_normal
        return (thumb.is_extended and 
                palm_normal.y > 0.5 and 
                all(not finger.is_extended for finger in hand.fingers[1:]))

    def detect_thumbs_down(self, hand):
        # Similar to thumbs up but inverted
        thumb = hand.fingers[0]
        palm_normal = hand.palm_normal
        return (thumb.is_extended and 
                palm_normal.y < -0.5 and 
                all(not finger.is_extended for finger in hand.fingers[1:]))

    def detect_i_love_you(self, hand):
        # Check if thumb, index, and pinky are extended
        thumb_extended = hand.fingers[0].is_extended
        index_extended = hand.fingers[1].is_extended
        pinky_extended = hand.fingers[4].is_extended
        others_folded = not hand.fingers[2].is_extended and not hand.fingers[3].is_extended
        return thumb_extended and index_extended and pinky_extended and others_folded

    def on_frame(self, controller):
        frame = controller.frame()
        
        if not frame.hands.is_empty:
            hand = frame.hands[0]  # Get the first hand

            # Detect gestures and send to Unity
            try:
                if self.detect_fist(hand):
                    sock.sendall(b"Closed_Fist")
                    print("Closed Fist detected!")
                elif self.detect_open_hand(hand):
                    sock.sendall(b"Open_Palm")
                    print("Open Palm detected!")
                elif self.detect_pointing_up(hand):
                    sock.sendall(b"Pointing_Up")
                    print("Pointing Up detected!")
                elif self.detect_thumbs_down(hand):
                    sock.sendall(b"Thumb_Down")
                    print("Thumb Down detected!")
                elif self.detect_thumbs_up(hand):
                    sock.sendall(b"Thumb_Up")
                    print("Thumb Up detected!")
                elif self.detect_victory(hand):
                    sock.sendall(b"Victory")
                    print("Victory detected!")
                elif self.detect_i_love_you(hand):
                    sock.sendall(b"ILoveYou")
                    print("I Love You detected!")
                else:
                    sock.sendall(b"0:Unknown")
                    print("Unknown gesture detected!")
            except Exception as e:
                print(f"Error sending data: {e}")

def main():
    # Create a listener and controller
    listener = GestureListener()
    controller = Controller()

    # Have the listener receive events from the controller
    controller.add_listener(listener)

    print("Press Enter to quit...")
    try:
        input()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the listener when done
        controller.remove_listener(listener)
        sock.close()

if __name__ == "__main__":
    main()