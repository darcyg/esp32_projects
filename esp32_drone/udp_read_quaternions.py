import socket
import struct

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('192.168.178.68', 3333)
print(f'starting up on {server_address[0]} port {server_address[1]}')
sock.bind(server_address)

print("####### Server is listening #######")
while True:
    data, address = sock.recvfrom(160)
    data_float = struct.unpack('<40f', data)
    print(f'----------------------------------------------------------- {len(data_float)}')
    for i in range(int(len(data_float)/4)):
        print(f'q0: {data_float[i*4]:.6f} \t q1: {data_float[4*i+1]:.6f} \t q2: {data_float[4*i+2]:.6f} \tq3: {data_float[4*i+3]:.6f}')
