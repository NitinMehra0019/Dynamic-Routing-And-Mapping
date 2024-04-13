import socket
import struct

RECEIVER_PORT = 17781
BASE_PACKET_PAYLOAD_SIZE = 10096


class BasePacket:
    def __init__(self, crc32, count,payload):
        self.crc32 = crc32
        self.count = count

        self.payload = payload


class PDUMessage:
    def __init__(self, messageID, messageLength, basePacket):
        self.messageID = messageID
        self.messageLength = messageLength
        self.basePacket = basePacket


class gps_packet:
    def __init__(self, latitude, latitude_indicator, longitude, longitude_indicator, dilution_of_precision, horizontal_dilution_of_precision, vertical_dilution_of_precision):
        self.latitude = latitude
        self.latitude_indicator = latitude_indicator
        self.longitude = longitude
        self.longitude_indicator = longitude_indicator
        self.dilution_of_precision = dilution_of_precision
        self.horizontal_dilution_of_precision = horizontal_dilution_of_precision
        self.vertical_dilution_of_precision = vertical_dilution_of_precision


def create_udp_socket():
    try:
        sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Receiver socket opened")
        return sockfd
    except socket.error as e:
        print("Failed to create receiver socket:", e)
        return None


def bind_socket(sockfd, port):
    try:
        sockfd.bind(("", port))
        print("Socket bound successfully")
        return True
    except socket.error as e:
        print("Failed to bind socket:", e)
        return False


def receive_data(sockfd):
    try:
        data, sender_addr = sockfd.recvfrom(4096)
        
        print(len(data))
        
        # Ensure the received data is at least 24 bytes before unpacking
        # if len(data) < 24:
        #     print("Received data too short:", len(data), "bytes")
        #     return None, None
        
            
        received_message = struct.unpack('=IILQ', data[:20])  # Unpack the fixed part of the message
        print(received_message)
        print(received_message[0])
        print(received_message[1])
        print(received_message[2])
        print(received_message[3])
        # print(received_message[4])

        payload = struct.unpack('dbdbddd', data[20:]) # Unpack the payload
        # print("length of payload : ", len(payload))
        # payload  = data[20:]
        print("After Payload in Reciver end", payload)

        received_message = PDUMessage(received_message[0], received_message[1], BasePacket(received_message[2], received_message[3], payload))
        return received_message, sender_addr
    except socket.error as e:
        print("Failed to receive data:", e)
        return None, None


def get_gps_data(sockfd):
    while True:
        print("data thread")
        received_message, sender_addr = receive_data(sockfd)
        print("Inside get gps data ", received_message)

        if received_message.messageID == 277:
            # recv_packet = gps_packet(*struct.unpack('dbdbddd', received_message.basePacket.payload))
            
            recv_packet = gps_packet(*received_message.basePacket.payload)


            print("Received data from IP:", sender_addr[0])
            print("Latitude : ", recv_packet.latitude)
            print("Longitude : ", recv_packet.longitude)
            # print("Latitude : ", recv_packet.latitude)
            
            print("data received successfully")


def main():
    sockfd = create_udp_socket()
    if not sockfd:
        return

    if not bind_socket(sockfd, RECEIVER_PORT):
        sockfd.close()
        return

    get_gps_data(sockfd)


if __name__ == "__main__":
    main()
