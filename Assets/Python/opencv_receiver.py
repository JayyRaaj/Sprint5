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
            # decode the image using cv2.imread()
            try:
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            except Exception as e:
                print(f"Error decoding image: {e}")
                continue
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
            # decode the image using cv2.imread()
            try:
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                frame = frame[:, :, 2]
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            except Exception as e:
                print(f"Error decoding depth frame: {e}")
                continue
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
            
            # Process the RGB image to identify a red target
            if rgb_image is not None and depth_image is not None:
                hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 100, 100])
                upper_red = np.array([10, 255, 255])
                mask = cv2.inRange(hsv_image, lower_red, upper_red)
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    # Find the largest red contour
                    largest_contour = max(contours, key=cv2.contourArea)
                    # Calculate the centroid of the red target
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # Draw a green box around the red target
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        cv2.rectangle(rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        # Get the depth value at the centroid
                        depth = depth_image[cy, cx]
                        # Calculate the world position of the target
                        world_x = (cx - IMAGE_WIDTH // 2) * depth / 525
                        world_y = (cy - IMAGE_HEIGHT // 2) * depth / 525
                        world_z = depth
                        # Send the world position to Unity
                        outgoing_message = f"target_x={world_x:.2f},target_y={world_y:.2f},target_z={world_z:.2f}".encode()
                        udp_socket.sendto(outgoing_message, (ipaddr, destination_port))
                        print(f"Sent target world position: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})")
                    else:
                        # No red target found
                        outgoing_message = b"no_target"
                        udp_socket.sendto(outgoing_message, (ipaddr, destination_port))
                        print("No red target found")
                else:
                    # No red target found
                    outgoing_message = b"no_target"
                    udp_socket.sendto(outgoing_message, (ipaddr, destination_port))
                    print("No red target found")
            
# ---------------------------------------------------------------------------- #
#                         ADD YOUR CODE ABOVE THIS LINE                        #
# ---------------------------------------------------------------------------- #

        except Exception as e:
            print(f"Error in send_thread: {e}")
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