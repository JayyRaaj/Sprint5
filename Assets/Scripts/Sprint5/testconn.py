import socket

# Define the UDP settings
ip_address = "127.0.0.1"  # Localhost
port = 65409  # Port number that Unity is listening to

# Example target position in [X, Y, Z] format
target_position = "[0.07510197907686234, -0.1510823369026184, 0.457275390625]"

# Convert the message to bytes
message = target_position.encode('ascii')

# Create a UDP socket and send the data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(message, (ip_address, port))
print("Sent target position:", target_position)
