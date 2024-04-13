#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>
#define RECEIVER_PORT 17781

constexpr size_t BASE_PACKET_PAYLOAD_SIZE = 10096;

struct BasePacket_t
{
    uint32_t crc32; // the CRC will be computed on everything below
    uint16_t count;
    uint16_t version;
    uint64_t timestamp;
    uint8_t payload[BASE_PACKET_PAYLOAD_SIZE];
};

struct PDUMessage_t
{
    uint32_t messageID;     // big-endian
    uint32_t messageLength; // big-endian (BASE_PACKET_HEADER_SIZE + # actual
                            // bytes stored in "payload" array)
    BasePacket_t basePacket;
};

struct gps_packet_t // structure to send
{
    long double latitude;
    char latitude_indicator;
    long double longitude;
    char longitude_indicator;
    double dilution_of_precision;
    double horizontal_dilution_of_precision;
    double vertical_dilution_of_precision;
};

int createUDPSocket()
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create receiver socket" << std::endl;
    }
    else
    {
        std::cout << "Receiver socket opened" << std::endl;
    }
    return sockfd;
}

bool bindSocket(int sockfd, int port, struct sockaddr_in &srAddr)
{
    // struct sockaddr_in srAddr;
    memset(&srAddr, 0, sizeof(srAddr));
    srAddr.sin_family = AF_INET;
    srAddr.sin_port = htons(port);
    srAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (struct sockaddr *)&srAddr, sizeof(srAddr)) < 0)
    {
        std::cerr << "Failed to bind socket" << std::endl;
        return false;
    }
    return true;
}

bool receiveData(int sockfd, PDUMessage_t &receivedMessage,
                 struct sockaddr_in &senderAddr)
{
    socklen_t senderAddrLen = sizeof(senderAddr);
    // cout << "UART - Ready to receive" << endl;

    int bytesReceived =
        recvfrom(sockfd, &receivedMessage, sizeof(receivedMessage), 0,
                 (struct sockaddr *)&senderAddr, &senderAddrLen);

    if (bytesReceived < 0)
    {
        std::cerr << "Failed to receive data" << std::endl;
        return false;
    }

    // cout << "Data received" << endl;
    return true;
}

void getGPSData(int sockfd)
{
    while (true)
    {
        std::cout << "data thread \n";
        PDUMessage_t receivedMessage;
        struct sockaddr_in senderAddr;

        if (receiveData(sockfd, receivedMessage, senderAddr))
        {

            if (receivedMessage.messageID == 277)
            {
                gps_packet_t recv_packet;

                memcpy(&recv_packet, receivedMessage.basePacket.payload, sizeof(gps_packet_t));

                std::cout << "Received data from IP: " << inet_ntoa(senderAddr.sin_addr) << std::endl;
                std::cout << recv_packet.horizontal_dilution_of_precision << std::endl;
                std::cout << "data recv successfully\n";
            }
        }
    }
}

int main()
{
    int sockfd = createUDPSocket();
    if (sockfd < 0)
    {
        return 1;
    }

    struct sockaddr_in receiverAddr;
    if (!bindSocket(sockfd, RECEIVER_PORT, receiverAddr))
    {
        close(sockfd);
        return 1;
    }

    getGPSData(sockfd);
}