import socket
import struct

RECEIVER_PORT = 17781
BASE_PACKET_PAYLOAD_SIZE = 10096

class BasePacket:
    def __init__(self, crc32, count, version, timestamp, payload):
        self.crc32 = crc32
        self.count = count
        self.version = version
        self.timestamp = timestamp
        self.payload = payload
"""
class PDUMessage:
    def __init__(self, messageID, messageLength, basePacket):
        self.messageID = messageID
        self.messageLength = messageLength
        self.basePacket = basePacket
        """

class gps_packet:
    def __init__(self, latitude, latitude_indicator, longitude, longitude_indicator, dilution_of_precision, horizontal_dilution_of_precision, vertical_dilution_of_precision):
    # def __init__(self, latitude, latitude_indicator, longitude, longitude_indicator):

        self.latitude = latitude
        self.latitude_indicator = latitude_indicator
        self.longitude = longitude
        self.longitude_indicator = longitude_indicator
        self.dilution_of_precision = dilution_of_precision
        self.horizontal_dilution_of_precision = horizontal_dilution_of_precision
        self.vertical_dilution_of_precision = vertical_dilution_of_precision

def createUDPSocket():
    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Receiver socket opened")
    return sockfd

def bindSocket(sockfd, port):
    try:
        sockfd.bind(('0.0.0.0', port))
        return True
    except Exception as e:
        print("Failed to bind socket:", e)
        return False

def receiveData(sockfd):
    try:
        data, senderAddr = sockfd.recvfrom(1024)  # Assuming maximum packet size is 1024
        # receivedMessage = struct.unpack('!IIIIQI{}s'.format(BASE_PACKET_PAYLOAD_SIZE), data)
        messageID, messageLength, crc32, count, version, timestamp, payload = struct.unpack('!IIIIQ{}s'.format(BASE_PACKET_PAYLOAD_SIZE), data[:32 + BASE_PACKET_PAYLOAD_SIZE])

        print(receivedMessage)
        # print(receivedMessage.__sizeof__())
        return receivedMessage, senderAddr
    except Exception as e:
        print("Failed to receive data:", e)
        return None, None

def getGPSData(sockfd):
    while True:
        print("data thread")
        receivedMessage, senderAddr = receiveData(sockfd)
        print(receivedMessage)

        if receivedMessage:
            messageID, messageLength, crc32, count, version, timestamp, payload = receivedMessage
            if messageID == 277:
                # recv_packet = gps_packet(*struct.unpack('!ddccddd', payload[:48]))  # Assuming payload size is 48
                recv_packet = gps_packet(*struct.unpack('!ddccddd', payload[:42]))  # Assuming payload size is 42

                print("Received data from IP:", senderAddr[0])
                print(recv_packet.horizontal_dilution_of_precision)
                print("data recv successfully")

def main():
    sockfd = createUDPSocket()

    if not bindSocket(sockfd, RECEIVER_PORT):
        return 1

    getGPSData(sockfd)

if __name__ == "__main__":
    main()
