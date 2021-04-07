import socket
import struct
from time import sleep

# Example code from: https://pymotw.com/2/socket/multicast.html#:~:text=Multicast%20messages%20are%20always%20sent,255.255)%20reserved%20for%20multicast%20traffic.

multicast_group = ('224.3.29.71', 10000)

# Create the datagram socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Set a timeout so the socket does not block indefinitely when trying to receive data.
sock.settimeout(0.2)

# Set the time-to-live for messages to 1 so they do not go past the
# local network segment.
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

# Look for responses from all recipients
message = 'sync'
while True:
    # Send data to the multicast group
    print(f'sending {message}')
    sent = sock.sendto(message.encode(), multicast_group)
    sleep(1)


