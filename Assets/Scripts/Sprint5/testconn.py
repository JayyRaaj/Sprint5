import socket
import time

# UDP socket configuration
UDP_IP = "127.0.0.1"  # Localhost or IP of the machine running Unity
UDP_PORT = 65402      # Port number should match Unity's port

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Sample target position data to send
positions = [
    "[0.158, 0.51, -0.438]",  # Position matching the Unity target position
  
]

# Send data in a loop to test connection
for pos in positions:
    sock.sendto(pos.encode('utf-8'), (UDP_IP, UDP_PORT))
    print(f"Sent position data: {pos}")
    time.sleep(1)  # Delay for readability and testing

# Close the socket after testing
sock.close()
