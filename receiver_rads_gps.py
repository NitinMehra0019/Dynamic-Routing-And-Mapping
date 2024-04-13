import socket
import struct

RECEIVER_PORT = 17781
BUFFER_SIZE = 1024

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('', RECEIVER_PORT)
sock.bind(server_address)

print("Receiver is listening on port", RECEIVER_PORT)

while True:
    print("\nWaiting to receive message...")
    data, address = sock.recvfrom(BUFFER_SIZE)
    print("Received message from:", address)
    
    # Unpack the received data according to the C++ struct
    message_id, message_length = struct.unpack('II', data[:8])
    payload = payload = data[8:]
    
    print("Header Values : \n")
    print("Message Id : ", message_id)
    print("Message Length : ",message_length)
    # print("CRC 32 : ", crc32)
    # print("Count : ", count)
    print("Payload : ", payload)
    print("-------------------------------------------------------------------------------------------------")
    
    
    # Extract GPS packet from payload
    gps_packet = struct.unpack('dbdbddd', payload[:56])  # Assuming gps_packet_t is 56 bytes
    
    # Print received GPS data
    print("\nReceived GPS Data:")
    print("Latitude:", gps_packet[0], chr(gps_packet[1]))
    print("Longitude:", gps_packet[2], chr(gps_packet[3]))
    print("Dilution of Precision:", gps_packet[4])
    print("Horizontal Dilution of Precision:", gps_packet[5])
    print("Vertical Dilution of Precision:", gps_packet[6])
