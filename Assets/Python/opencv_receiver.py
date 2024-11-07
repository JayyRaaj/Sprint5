import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"

import threading
import struct
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import socket
import traceback
import socket 

INTERRUPT = False
THREAD_LOCK = threading.Lock()
OUTGOING_BUFFER = None
CHUNK_SIZE = 1400

# image defaults
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EOM_MARKER = b'<<EOM>>'

# image for later use
RGB_FRAME = None
DEPTH_FRAME = None

# thread safety lock
THREAD_LOCK = threading.Lock()

def rgb_func(ipaddr='127.0.0.1', port=65400):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            # read until saw at least two EOM markers
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            # get the content between the two EOM markers
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]
            # decode JPEG image
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            # display the image
            # cv2.imshow('RGB Image', frame)
            # cv2.waitKey(1)
            # save the image
            with THREAD_LOCK:
                global RGB_FRAME
                RGB_FRAME = frame
        except:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass
        
def depth_func(ipaddr='127.0.0.1', port=65401):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            # read until saw at least two EOM markers
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            # get the content between the two EOM markers
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]
            # decode EXR ZIP image
            frame = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)
            frame = frame[:, :, 2]
            print(f'Minimum depth: {np.min(frame)}, Maximum depth: {np.max(frame)}')
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            # display the image
            cv2.imshow('Depth Image', frame)
            cv2.waitKey(1)
            # save the image
            with THREAD_LOCK:
                global DEPTH_FRAME
                DEPTH_FRAME = frame
        except:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass

def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, bind_port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            rgb_image = None
            depth_image = None
            
            try:
                with THREAD_LOCK:
                    if RGB_FRAME is not None: rgb_image = RGB_FRAME.copy()
                    if DEPTH_FRAME is not None: depth_image = DEPTH_FRAME.copy()
            except:
                pass
            
# ---------------------------------------------------------------------------- #
#                         ADD YOUR CODE BELOW THIS LINE                        #
# ---------------------------------------------------------------------------- #
            
            # do image processing here
          
           # Ensure both RGB and depth images are available
            if rgb_image is not None and depth_image is not None:
                # Convert RGB image to HSV for color detection
                hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

                # Define the HSV range for red color
                lower_red1 = np.array([0, 120, 70])
                upper_red1 = np.array([10, 255, 255])
                mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

                lower_red2 = np.array([170, 120, 70])
                upper_red2 = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

                # Combine masks
                mask = mask1 | mask2

                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                print(f"Number of red contours found: {len(contours)}")  # Debug statement

                # Proceed if contours are found
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    print(f"Largest contour area: {cv2.contourArea(largest_contour)}")  # Debug

                    # Get bounding box of the largest contour
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    target_x = x + w // 2
                    target_y = y + h // 2

                    print(f"Target detected at pixel (x, y): ({target_x}, {target_y})")  # Debug

                    # Use depth information for 3D positioning
                    depth_value = depth_image[target_y, target_x]
                    print(f"Depth value at target position: {depth_value}")  # Debug

                    # Camera intrinsics
                    fx = 347.0574
                    fy = 520.5861
                    cx = 271
                    cy = 242
                    
                    # Convert to 3D world coordinates
                    Z = depth_value
                    X = (target_x - cx) * Z / fx
                    Y = (target_y - cy) * Z / fy

                    # Prepare the message
                    target_position = target_position = f"[{X}, {Y}, {Z}]".encode('ascii')

                    # Send to Unity
                    udp_socket.sendto(target_position, (ipaddr, destination_port))
                    print("Target 3D position sent to Unity.")  # Debug
                else:
                    print("No target detected.")  # Debug
            else:
                print("Waiting for RGB and Depth frames to be available...")  
            
           
# ---------------------------------------------------------------------------- #
#                         ADD YOUR CODE ABOVE THIS LINE                        #
# ---------------------------------------------------------------------------- #

        except Exception:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass

if __name__ == '__main__':
    # start rgb thread
    rbg_thread = threading.Thread(target=rgb_func)
    rbg_thread.start()
    # start depth thread
    depth_thread = threading.Thread(target=depth_func)
    depth_thread.start()
    # start sending thread
    send_thread = threading.Thread(target=send_thread)
    send_thread.start()
    # wait for threads to finish
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        INTERRUPT = True
        rbg_thread.join()
        send_thread.join()
        exit(0)
        
    

