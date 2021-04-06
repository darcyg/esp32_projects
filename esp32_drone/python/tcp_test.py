import socket
import struct
import asyncio

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('192.168.178.68', 3333)
print(f'starting up on {server_address[0]} port {server_address[1]}')
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    try:
        print(f'connection from {client_address}')

        # Receive the data in small chunks
        while True:
            data = connection.recv(16)
            # expecting 4 floats (quaternions)
            data_float = struct.unpack('<4f', data)
            print(f'received: {data_float}')
            # if data:
            #     connection.sendall(data)
            #     print('sending data back to the client')
            #     print('Enter throttle: ')
            #     message = input()
            #     print(f'You wrote: {message}')
            #     connection.sendall(message.encode())
            # else:
            #     print(f'no more data from {client_address}')
            #     break

    finally:
        # Clean up the connection
        connection.close()
