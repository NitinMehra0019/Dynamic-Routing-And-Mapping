// FULL WORKING 
#include "gps.hpp"
#include "gps_communications.hpp"
#include "gps_error_handling.hpp"
#include <cmath>
// #include <csignal>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <sys/fcntl.h> // Used for UART
#include <termios.h>   // Used for UART
#include <unistd.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>

#define RECEIVER_PORT 17781
#define SENDER_PORT 12343

// Global variables for synchronization
std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void milisleep(unsigned milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Structure for storing GPS data
struct Instruction
{
    int type;
    std::array<uint8_t, 96> instruction;
    std::array<uint8_t, 96> verbal_transition_alert_instruction;
    std::array<uint8_t, 96> verbal_succinct_transition_instruction;
    std::array<uint8_t, 96> verbal_pre_transition_instruction;
    std::array<uint8_t, 96> verbal_post_transition_instruction;
    // std::vector<std::string> street_names;
    // std::vector<std::string> begin_street_names;
    double time;
    double length;
    double cost;
    int begin_shape_index;
    int end_shape_index;
    std::array<uint8_t, 96> travel_mode;
    std::array<uint8_t, 96> travel_type;
};

struct ReceivedData
{
    int path_size;
    int instruction_size;
    std::vector<std::pair<double, double>> path;
    double distance;
    std::vector<Instruction> instructions;
    double time;
};

struct gps_packet_t
{
    double latitude;
    char latitude_indicator;
    double longitude;
    char longitude_indicator;
    double dilution_of_precision;
    double horizontal_dilution_of_precision;
    double vertical_dilution_of_precision;
};
struct BasePacket
{
    uint32_t crc32;
    uint16_t count;
    uint16_t version;
    uint64_t timestamp;
    char payload[56000];
};

struct PDUMessage
{
    unsigned int messageID;
    unsigned int messageLength;
    BasePacket basePacket;
};
// Structure for storing received data
std::string uint8Array56ToString(const std::array<uint8_t, 56> &arr)
{
    std::string result;
    for (const auto &value : arr)
    {
        if (value == '\0')
        {
            break; // Null terminator reached
        }
        result += static_cast<char>(value);
    }
    return result;
}
// Define a function to convert an array of uint8_t to a string
std::string uint8ArrayToString(const std::array<uint8_t, 96> &arr)
{
    std::string result;
    for (const auto &value : arr)
    {
        if (value == '\0')
        {
            break; // Null terminator reached
        }
        result += static_cast<char>(value);
    }
    return result;
}

void send_to_Rads_Router(PDUMessage &received_message, unsigned int messageLength)
{

    std::cout << "Inside send_toRads_router :" << std::endl;

    ReceivedData data_received;

    std::cout << "Message size inside SEND_TO_RADS_ROUTER : " << messageLength << std::endl;

    // Extract path size
    memcpy(&data_received.path_size, received_message.basePacket.payload, sizeof(int));
    std::cout << "Path SIZE : " << data_received.path_size << std::endl;

    // Extract instruction size
    memcpy(&data_received.instruction_size, received_message.basePacket.payload + sizeof(int), sizeof(int));
    std::cout << "Instruction SIZE : " << data_received.instruction_size << std::endl;

    // Extract path data
    data_received.path.resize(data_received.path_size);
    memcpy(data_received.path.data(), received_message.basePacket.payload + 2 * sizeof(int), data_received.path_size * sizeof(std::pair<double, double>));

    // Extract distance
    size_t offset = 2 * sizeof(int) + data_received.path_size * sizeof(std::pair<double, double>);
    memcpy(&data_received.distance, received_message.basePacket.payload + offset, sizeof(double));
    offset += sizeof(double);
    std::cout << "Distance : " << data_received.distance << std::endl;

    // Extract instructions data
    data_received.instructions.resize(data_received.instruction_size);
    for (int i = 0; i < data_received.instruction_size; ++i)
    {
        Instruction temp_instruction;
        memcpy(&temp_instruction, received_message.basePacket.payload + offset, sizeof(Instruction));
        data_received.instructions[i] = temp_instruction;
        offset += sizeof(Instruction);
    }

    // Extract time
    memcpy(&data_received.time, received_message.basePacket.payload + offset, sizeof(double));
    int j = 1;
    for (const auto &path : data_received.path)
    {
        std::cout << "path: " << j++ << " - " << path.first << " -- " << path.second << std::endl;
    }
    int i = 1;
    /*
    begin_shape_index: 87
    cost: 26.823
    end_shape_index: 88
    instruction: "Turn left."
    length: 0.038
    time: 26.823
    travel_mode: "pedestrian"
    travel_type: "foot"
    type: 15
    verbal_post_transition_instruction: "Continue for 40 meters."
    verbal_pre_transition_instruction: "Turn left."
    verbal_succinct_transition_instruction: "Turn left."
    verbal_transition_alert_instruction: "Turn left."
    */
    for (const auto &instr : data_received.instructions)
    {

        std::cout << "Instruction - " << i << ": " << std::endl;
        std::cout << "begin_shape_index : " << instr.begin_shape_index << std::endl;
        std::cout << "cost : " << instr.cost << std::endl;
        std::cout << "end_shape_index : " << instr.end_shape_index << std::endl;
        std::cout << "instruction : " << uint8ArrayToString(instr.instruction) << std::endl;
        std::cout << "Length : " << instr.length << std::endl;
        std::cout << "time : " << instr.time << std::endl;
        std::cout << "travel_mode : " << uint8ArrayToString(instr.travel_mode) << std::endl;
        std::cout << "travel_type : " << uint8ArrayToString(instr.travel_type) << std::endl;
        std::cout << "type : " << instr.type << std::endl;
        std::cout << "verbal_post_transition_instruction : " << uint8ArrayToString(instr.verbal_post_transition_instruction) << std::endl;
        std::cout << "verbal_pre_transition_instruction : " << uint8ArrayToString(instr.verbal_pre_transition_instruction) << std::endl;
        std::cout << "verbal_succinct_transition_instruction : " << uint8ArrayToString(instr.verbal_succinct_transition_instruction) << std::endl;
        std::cout << "verbal_transition_alert_instruction : " << uint8ArrayToString(instr.verbal_transition_alert_instruction) << std::endl;

        // std::cout << "Verbal Post Transition Instruction - " << i << " : ";

        // std::cout << uint8ArrayToString(instr.verbal_post_transition_instruction);
        // std::cout << std::endl;
        std::cout << std::endl;

        std::cout << "--------------------------------------------------------- " << std::endl;
        i++;

        // Print other instruction fields as needed
    }

    std::cout << "Sending complete " << std::endl;
}

PDUMessage create_PDUMessage(unsigned int messageID, unsigned int messageLength, ReceivedData parsedData)
{
    PDUMessage pduMessage;
    pduMessage.messageID = messageID;
    pduMessage.messageLength = messageLength;

    // Copy path size and instruction size into payload
    memcpy(pduMessage.basePacket.payload, &parsedData.path_size, sizeof(int));
    memcpy(pduMessage.basePacket.payload + sizeof(int), &parsedData.instruction_size, sizeof(int));

    // Copy path coordinates into payload
    size_t offset = 2 * sizeof(int);
    for (const auto &coord : parsedData.path)
    {
        memcpy(pduMessage.basePacket.payload + offset, &coord.first, sizeof(double));
        memcpy(pduMessage.basePacket.payload + offset + sizeof(double), &coord.second, sizeof(double));
        offset += 2 * sizeof(double);
    }

    // Copy distance into payload
    memcpy(pduMessage.basePacket.payload + offset, &parsedData.distance, sizeof(double));
    offset += sizeof(double);

    // Copy instructions into payload
    for (const auto &instr : parsedData.instructions)
    {
        memcpy(pduMessage.basePacket.payload + offset, &instr, sizeof(Instruction));
        offset += sizeof(Instruction);
    }

    // Copy time into payload
    memcpy(pduMessage.basePacket.payload + offset, &parsedData.time, sizeof(double));

    // Set other fields of base packet
    pduMessage.basePacket.crc32 = 10;
    pduMessage.basePacket.count = 20;
    pduMessage.basePacket.version = 450;
    pduMessage.basePacket.timestamp = 1234;

    send_to_Rads_Router(pduMessage, messageLength);

    return pduMessage;
}

/*
PDUMessage create_PDUMessage(unsigned int messageID, unsigned int messageLength, ReceivedData parsedData)
{
  PDUMessage pduMessage;
  pduMessage.messageID = messageID;
  pduMessage.messageLength = messageLength;
  memcpy(pduMessage.basePacket.payload, &parsedData, messageLength);

  // std::cout<<"Pay load data : "<<pduMessage.basePacket.payload<<std::endl;

  ReceivedData payData;

  memcpy(&payData.path_size, pduMessage.basePacket.payload, sizeof(int));
  memcpy(&payData.instruction_size, pduMessage.basePacket.payload+4, sizeof(int));
  std::cout << "payData.path_size: " << payData.path_size << std::endl;
  std::cout << "payData.instruction_size: " << payData.instruction_size << std::endl;

  // payData.path.resize(payData.path_size);
  // memcpy(&payData.path[0], &pduMessage.basePacket.payload[8], payData.path_size * sizeof(std::pair<double, double>));


  memcpy(&payData.path, (pduMessage.basePacket.payload+8), payData.path_size * 16);
  // memcpy(&payData.distance, &pduMessage.basePacket.payload[payData.path_size + 8], 8);
  // memcpy(&payData.instructions, &pduMessage.basePacket.payload[payData.path_size + 8 + 8], payData.instruction_size);
  // memcpy(&payData.time, &pduMessage.basePacket.payload[payData.path_size + 8 + 8 + payData.instruction_size], 8);

  for (const auto &path : payData.path)
  {


    std::cout << "path: "<< path.first <<" -- " <<path.second<<std::endl;
  }

  BasePacket basePacket;
  pduMessage.basePacket.crc32 = 10;
  pduMessage.basePacket.count = 20;
  pduMessage.basePacket.version = 450;
  pduMessage.basePacket.timestamp= 1234;

  // send_to_Rads_Router(pduMessage, messageLength);

  std::cout<<"After sending function : "<<std::endl;


  return pduMessage;
}
*/

void stringToCharArray(const std::string &str, std::array<uint8_t, 96> &charArray)
{
    memcpy(charArray.data(), str.c_str(), str.size() + 1); // +1 to include null terminator
}

ReceivedData parseJsonData(const std::string &jsonData)
{
    ReceivedData data;
    Json::Value root;
    Json::Reader reader;

    // create_PDUMessage(data, pduMessage);

    // Parse JSON
    if (!reader.parse(jsonData, root))
    {
        std::cerr << "Error parsing JSON data: " << reader.getFormattedErrorMessages() << std::endl;
        return data; // Return empty data in case of parsing error
    }

    try
    {
        // Path Size
        data.path_size = root["path_size"].asInt();

        // Instrcution size
        data.instruction_size = root["instruction_size"].asInt();
        // Extract path coordinates
        const Json::Value &pathArray = root["path"];
        for (const auto &coord : pathArray)
        {
            double lat = coord[0].asDouble();
            double lon = coord[1].asDouble();
            data.path.push_back(std::make_pair(lat, lon));
        }

        // Extract distance
        data.distance = root["distance"].asDouble();

        // Extract instructions
        const Json::Value &instructionArray = root["instruction"];
        for (const auto &instr : instructionArray)
        {
            Instruction instruction;
            instruction.type = instr["type"].asInt();
            // Convert JSON strings to std::array<uint8_t, 100>
            std::string instructionStr = instr["instruction"].asString();
            // std::copy(instructionStr.begin(), instructionStr.end(), instruction.instruction.begin());
            stringToCharArray(instructionStr, instruction.instruction);

            std::string verbalTransitionAlertStr = instr["verbal_transition_alert_instruction"].asString();
            // std::copy(verbalTransitionAlertStr.begin(), verbalTransitionAlertStr.end(), instruction.verbal_transition_alert_instruction.begin());
            stringToCharArray(verbalTransitionAlertStr, instruction.verbal_transition_alert_instruction);

            std::string verbalSuccinctTransitionStr = instr["verbal_succinct_transition_instruction"].asString();
            // std::copy(verbalSuccinctTransitionStr.begin(), verbalSuccinctTransitionStr.end(), instruction.verbal_succinct_transition_instruction.begin());
            stringToCharArray(verbalSuccinctTransitionStr, instruction.verbal_succinct_transition_instruction);

            std::string verbalPreTransitionStr = instr["verbal_pre_transition_instruction"].asString();
            // std::copy(verbalPreTransitionStr.begin(), verbalPreTransitionStr.end(), instruction.verbal_pre_transition_instruction.begin());
            stringToCharArray(verbalPreTransitionStr, instruction.verbal_pre_transition_instruction);

            std::string verbalPostTransitionStr = instr["verbal_post_transition_instruction"].asString();

            // instruction.verbal_post_transition_instruction = stringToArray(instr["verbal_post_transition_instruction"].asString());

            // std::copy(verbalPostTransitionStr.begin(), verbalPostTransitionStr.end(), instruction.verbal_post_transition_instruction.begin());
            // instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            stringToCharArray(verbalPostTransitionStr, instruction.verbal_post_transition_instruction);

            std::string travelModeStr = instr["travel_mode"].asString();
            stringToCharArray(travelModeStr, instruction.travel_mode);

            std::string travelTypeStr = instr["travel_type"].asString();
            stringToCharArray(travelTypeStr, instruction.travel_type);

            // std::memcpy(&instruction.verbal_post_transition_instruction, &verbalPostTransitionStr, sizeof(verbalPostTransitionStr));
            // instruction.verbal_post_transition_instruction[sizeof(verbalPostTransitionStr)] = '\0';

            // for(size_t i = 0; i<verbalPostTransitionStr.size() && i<100; ++i){
            //     instruction.verbal_post_transition_instruction[i] = static_cast<uint8_t>(verbalPostTransitionStr[i]);
            // }
            // std::copy(verbalPostTransitionStr.begin(), verbalPostTransitionStr.end(), instruction.verbal_post_transition_instruction.begin());
            // instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            // Ensure null-termination

            // if (verbalPostTransitionStr.size() < 100)
            // {
            //     instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            // }

            /*
              for (const auto &street : instr["street_names"])
              {
                instruction.street_names.push_back(street.asString());
              }
              for (const auto &begin_street : instr["begin_street_names"])
              {
                instruction.begin_street_names.push_back(begin_street.asString());
              }

              */

            instruction.time = instr["time"].asDouble();
            instruction.length = instr["length"].asDouble();
            instruction.cost = instr["cost"].asDouble();
            instruction.begin_shape_index = instr["begin_shape_index"].asInt();
            instruction.end_shape_index = instr["end_shape_index"].asInt();
            // std::string travelModeStr = instr["travel_mode"].asString();
            // // std::copy(travelModeStr.begin(), travelModeStr.end(), instruction.travel_mode.begin());
            // stringToCharArray(travelModeStr, instruction.travel_mode);
            // std::string travelTypeStr = instr["travel_type"].asString();
            // // std::copy(travelTypeStr.begin(), travelTypeStr.end(), instruction.travel_type.begin());
            // stringToCharArray(travelTypeStr, instruction.travel_type);

            data.instructions.push_back(instruction);
        }

        // Extract time
        data.time = root["time"].asDouble();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error extracting data from JSON: " << e.what() << std::endl;
    }

    return data;
}

void get_gps_data(int sockfd, sockaddr_in &sendAddr)
{

    for (int i = 0; i < 100; ++i)

    {
        gps_packet_t send_packet;
        PDUMessage pduMessage;
        pduMessage.messageID = 277;
        pduMessage.messageLength = sizeof(pduMessage);

        pduMessage.basePacket.count = 10;
        pduMessage.basePacket.crc32 = 20;
        pduMessage.basePacket.timestamp = 1234;
        pduMessage.basePacket.version = 2;

        send_packet.latitude = 10 + i;
        send_packet.latitude_indicator = 'E';
        send_packet.longitude = 20 + i;
        send_packet.longitude_indicator = 'N';
        send_packet.dilution_of_precision = 93.0 - i;
        send_packet.horizontal_dilution_of_precision = 67.9 - i;
        send_packet.vertical_dilution_of_precision = 99.0 - i;
        std::cout << "Message Number : " << i << std::endl;

        std::cout << "Size of gps_packet_t : " << sizeof(gps_packet_t) << std::endl;

        memcpy(pduMessage.basePacket.payload, &send_packet, sizeof(gps_packet_t));

        std::cout << "Size of PDU Message : " << sizeof(pduMessage) << std::endl;

        sendto(sockfd, &pduMessage, sizeof(PDUMessage), 0, (struct sockaddr *)&sendAddr, sizeof(sendAddr));

        // std::this_thread::sleep_for(std::chrono::seconds(1)); // Delay between messages
        milisleep(1000);
    }
}

// Function to send data over UDP
void send_via_udp()
{
    // Wait for receiver to be ready
    std::cout << "Waiting fr receiver side to receive data : " << std::endl;

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, []
            { return ready; });

    // std::cout<<"Waiting for send Data : "<<std::endl;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create UDP socket\n";
        return;
    }

    struct sockaddr_in sendAddr;
    memset(&sendAddr, 0, sizeof(sendAddr));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(RECEIVER_PORT);
    inet_pton(AF_INET, "127.0.0.1", &sendAddr.sin_addr);

    get_gps_data(sockfd, sendAddr);

    close(sockfd);
}

// Function to receive data over UDP
void receive_via_udp()
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create UDP socket\n";
        return;
    }

    struct sockaddr_in receiverAddr;
    // struct sockaddr_in receiverAddr, senderAddr;
    // socklen_t senderSize = sizeof(senderAddr);
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_addr.s_addr = INADDR_ANY;
    receiverAddr.sin_port = htons(SENDER_PORT);

    if (bind(sockfd, (struct sockaddr *)&receiverAddr, sizeof(receiverAddr)) < 0)
    {
        std::cerr << "receiver code Bind failed\n";
        close(sockfd);
        return;
    }

    struct sockaddr_in senderAddr;
    socklen_t senderSize = sizeof(senderAddr);
    char buffer[46024]; // Increase buffer size to accommodate larger JSON data
    // char buffer[45008];

    while (true)
    {
        std::cout << "Waiting for data... " << std::endl;
        ssize_t bytesReceived = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&senderAddr, &senderSize);
        if (bytesReceived < 0)
        {
            std::cerr << "Error in recvfrom()\n";
            // break;
        }
        // PDUMessage pduMessage;
        // pduMessage.messageID = 1;
        // pduMessage.messageLength = sizeof(BasePacket);

        // BasePacket basePacket;
        // pduMessage.basePacket.crc32 = 0;
        // pduMessage.basePacket.count = htons(10);
        // pduMessage.basePacket.version = htons(1);
        // pduMessage.basePacket.timestamp = 1234;
        // //bas
        std::cout << "Bytes Recived from Receive function : " << bytesReceived << std::endl;
        std::string receivedData(buffer, bytesReceived);

        // send_to_Rads_Router(route_Message, receivedData);

        // std::cout<< "Raw Received Data : " << receivedData;
        //  std::cout << "Received data: " << receivedData << std::endl;

        // std::cout << "Received data: Latitude=" << receivedData.latitude << ", Longitude=" << receivedData.longitude << std::endl;

        // Parse received JSON data

        ReceivedData parsedData = parseJsonData(receivedData);

        PDUMessage route_Message = create_PDUMessage(10, bytesReceived, parsedData);
        // send_to_Rads_Router(route_Message, bytesReceived);

        std::cout << "After sending the data to RADS_ROUTER: " << std::endl;

        // create_PDUMessage(10, 300, parsedData);

        // pduMessage.basePacket.payload = parsedData;
        //  create_PDUMessage(parsedData, pd)
        // std::cout<<"Path Size : "<<parsedData.path_size<<std::endl;
        //  std::cout << "Instruction Size : " << parsedData.instruction_size << std::endl;

        // std::cout << "Path coordinates:" << std::endl;

        for (const auto &coord : parsedData.path)
        {
            // std::cout << "(" << coord.first << ", " << coord.second << ")" << std::endl;
        }
        // std::cout << "Distance: " << parsedData.distance << std::endl;

        // std::cout << "Instructions:" << std::endl;
        int i = 0;
        for (const auto &instr : parsedData.instructions)
        {
            /*
            std::cout << instr.begin_shape_index << std::endl;
            std::cout << instr.cost << std::endl;
            std::cout << instr.end_shape_index << std::endl;
            std::cout << instr.instruction.data() << std::endl;
            */

            // std::cout << "Type: " << instr.type << std::endl;
            // std::cout << "Instruction: "<<i++;
            // for (const auto &value : instr.instruction)
            // {
            //     std::cout << static_cast<char>(value);
            // }
            // std::cout << instr.instruction.data();
            // std::cout << std::endl;

            // std::cout << "Verbal Post Transition Instruction: ";

            // for (const auto &value : instr.verbal_post_transition_instruction)

            // {
            //     if(value != '\0'){
            //         std::cout << static_cast<char>(value);
            //     }
            // }

            // std::cout << instr.verbal_post_transition_instruction.data();
            // std::cout << std::endl;

            // Print other instruction fields as needed
        }
        // std::cout << "Time: " << parsedData.time << std::endl;

        // Notify main thread that data has been received
        {
            std::lock_guard<std::mutex> lock(mtx);
            ready = true;
        }
        cv.notify_one();
    }
    milisleep(1000);

    close(sockfd);
}

int main()
{
    std::cout << "Sender thread" << std::endl;

    std::thread sender(send_via_udp);
    std::cout << "Receiver thread" << std::endl;

    while (true)
    {
        std::thread receiver(receive_via_udp);
        receiver.join();
        // Pause the sender thread for some time
        // std::this_thread::sleep_for(std::chrono::seconds(5)); // Adjust the duration as needed
    }

    // Wait for receiver thread to receive data
    {
        std::lock_guard<std::mutex> lock(mtx);
        ready = true;
    }
    cv.notify_one();

    // Do something with the received data

    sender.join();

    return 0;
}

/*
#include "gps.hpp"
#include "gps_communications.hpp"
#include "gps_error_handling.hpp"
#include <cmath>
// #include <csignal>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <sys/fcntl.h> // Used for UART
#include <termios.h>   // Used for UART
#include <unistd.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>

#define RECEIVER_PORT 17781
#define SENDER_PORT 12343

// Global variables for synchronization
std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void milisleep(unsigned milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Structure for storing GPS data
struct Instruction
{
    int type;
    std::array<uint8_t, 96> instruction;
    std::array<uint8_t, 96> verbal_transition_alert_instruction;
    std::array<uint8_t, 96> verbal_succinct_transition_instruction;
    std::array<uint8_t, 96> verbal_pre_transition_instruction;
    std::array<uint8_t, 96> verbal_post_transition_instruction;
    // std::vector<std::string> street_names;
    // std::vector<std::string> begin_street_names;
    double time;
    double length;
    double cost;
    int begin_shape_index;
    int end_shape_index;
    std::array<uint8_t, 56> travel_mode;
    std::array<uint8_t, 56> travel_type;
};

struct ReceivedData
{
    int path_size;
    int instruction_size;
    std::vector<std::pair<double, double>> path;
    double distance;
    std::vector<Instruction> instructions;
    double time;
};

struct gps_packet_t
{
    double latitude;
    char latitude_indicator;
    double longitude;
    char longitude_indicator;
    double dilution_of_precision;
    double horizontal_dilution_of_precision;
    double vertical_dilution_of_precision;
};
struct BasePacket
{
    uint32_t crc32;
    uint16_t count;
    uint16_t version;
    uint64_t timestamp;
    char payload[100];
};

struct PDUMessage
{
    unsigned int messageID;
    unsigned int messageLength;
    BasePacket basePacket;
};
// Structure for storing received data

PDUMessage create_PDUMessage(unsigned int messageID, unsigned int messageLength, ReceivedData parsedData)
{
    PDUMessage pduMessage;
    pduMessage.messageID = 1;
    pduMessage.messageLength = messageLength;
    memcpy(pduMessage.basePacket.payload, &parsedData, sizeof(parsedData));

    BasePacket basePacket;
    pduMessage.basePacket.crc32 = 10;
    pduMessage.basePacket.count = 20;
    pduMessage.basePacket.version = 450;
    pduMessage.basePacket.timestamp = 1234;

    return pduMessage;
}

void stringToCharArray(const std::string &str, std::array<uint8_t, 96> &charArray)
{
    memcpy(charArray.data(), str.c_str(), str.size() + 1); // +1 to include null terminator
}

ReceivedData parseJsonData(const std::string &jsonData)
{
    ReceivedData data;
    Json::Value root;
    Json::Reader reader;

    // create_PDUMessage(data, pduMessage);

    // Parse JSON
    if (!reader.parse(jsonData, root))
    {
        std::cerr << "Error parsing JSON data: " << reader.getFormattedErrorMessages() << std::endl;
        return data; // Return empty data in case of parsing error
    }

    try
    {
        // Path Size
        data.path_size = root["path_size"].asInt();

        // Instrcution size
        data.instruction_size = root["instruction_size"].asInt();
        // Extract path coordinates
        const Json::Value &pathArray = root["path"];
        for (const auto &coord : pathArray)
        {
            double lat = coord[0].asDouble();
            double lon = coord[1].asDouble();
            data.path.push_back(std::make_pair(lat, lon));
        }

        // Extract distance
        data.distance = root["distance"].asDouble();

        // Extract instructions
        const Json::Value &instructionArray = root["instruction"];
        for (const auto &instr : instructionArray)
        {
            Instruction instruction;
            instruction.type = instr["type"].asInt();
            // Convert JSON strings to std::array<uint8_t, 100>
            std::string instructionStr = instr["instruction"].asString();
            // std::copy(instructionStr.begin(), instructionStr.end(), instruction.instruction.begin());
            stringToCharArray(instructionStr, instruction.instruction);

            std::string verbalTransitionAlertStr = instr["verbal_transition_alert_instruction"].asString();
            // std::copy(verbalTransitionAlertStr.begin(), verbalTransitionAlertStr.end(), instruction.verbal_transition_alert_instruction.begin());
            stringToCharArray(verbalTransitionAlertStr, instruction.verbal_transition_alert_instruction);

            std::string verbalSuccinctTransitionStr = instr["verbal_succinct_transition_instruction"].asString();
            // std::copy(verbalSuccinctTransitionStr.begin(), verbalSuccinctTransitionStr.end(), instruction.verbal_succinct_transition_instruction.begin());
            stringToCharArray(verbalSuccinctTransitionStr, instruction.verbal_succinct_transition_instruction);

            std::string verbalPreTransitionStr = instr["verbal_pre_transition_instruction"].asString();
            // std::copy(verbalPreTransitionStr.begin(), verbalPreTransitionStr.end(), instruction.verbal_pre_transition_instruction.begin());
            stringToCharArray(verbalPreTransitionStr, instruction.verbal_pre_transition_instruction);

            std::string verbalPostTransitionStr = instr["verbal_post_transition_instruction"].asString();

            // instruction.verbal_post_transition_instruction = stringToArray(instr["verbal_post_transition_instruction"].asString());

            // std::copy(verbalPostTransitionStr.begin(), verbalPostTransitionStr.end(), instruction.verbal_post_transition_instruction.begin());
            // instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            stringToCharArray(verbalPostTransitionStr, instruction.verbal_post_transition_instruction);

            // std::memcpy(&instruction.verbal_post_transition_instruction, &verbalPostTransitionStr, sizeof(verbalPostTransitionStr));
            // instruction.verbal_post_transition_instruction[sizeof(verbalPostTransitionStr)] = '\0';

            // for(size_t i = 0; i<verbalPostTransitionStr.size() && i<100; ++i){
            //     instruction.verbal_post_transition_instruction[i] = static_cast<uint8_t>(verbalPostTransitionStr[i]);
            // }
            // std::copy(verbalPostTransitionStr.begin(), verbalPostTransitionStr.end(), instruction.verbal_post_transition_instruction.begin());
            // instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            // Ensure null-termination

            // if (verbalPostTransitionStr.size() < 100)
            // {
            //     instruction.verbal_post_transition_instruction[verbalPostTransitionStr.size()] = '\0';
            // }

            /*
              for (const auto &street : instr["street_names"])
              {
                instruction.street_names.push_back(street.asString());
              }
              for (const auto &begin_street : instr["begin_street_names"])
              {
                instruction.begin_street_names.push_back(begin_street.asString());
              }

              */


             /*/

            instruction.time = instr["time"].asDouble();
            instruction.length = instr["length"].asDouble();
            instruction.cost = instr["cost"].asDouble();
            instruction.begin_shape_index = instr["begin_shape_index"].asInt();
            instruction.end_shape_index = instr["end_shape_index"].asInt();
            // std::string travelModeStr = instr["travel_mode"].asString();
            // // std::copy(travelModeStr.begin(), travelModeStr.end(), instruction.travel_mode.begin());
            // stringToCharArray(travelModeStr, instruction.travel_mode);
            // std::string travelTypeStr = instr["travel_type"].asString();
            // // std::copy(travelTypeStr.begin(), travelTypeStr.end(), instruction.travel_type.begin());
            // stringToCharArray(travelTypeStr, instruction.travel_type);

            data.instructions.push_back(instruction);
        }

        // Extract time
        data.time = root["time"].asDouble();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error extracting data from JSON: " << e.what() << std::endl;
    }

    return data;
}

void send_to_Rads_Router(PDUMessage route_Message)
{
    std::cout << "Inside send_toRads_router :" << std::endl;
    ReceivedData data_received;
    memcpy(&data_received, route_Message.basePacket.payload, sizeof(data_received));
    std::cout << "Data from RADS Router : " << data_received.instruction_size << std::endl;
}

void get_gps_data(int sockfd, sockaddr_in &sendAddr)
{

    for (int i = 0; i < 100; ++i)

    {
        gps_packet_t send_packet;
        PDUMessage pduMessage;
        pduMessage.messageID = 277;
        pduMessage.messageLength = sizeof(pduMessage);

        pduMessage.basePacket.count = 10;
        pduMessage.basePacket.crc32 = 20;
        pduMessage.basePacket.timestamp = 1234;
        pduMessage.basePacket.version = 2;

        send_packet.latitude = 10 + i;
        send_packet.latitude_indicator = 'E';
        send_packet.longitude = 20 + i;
        send_packet.longitude_indicator = 'N';
        send_packet.dilution_of_precision = 93.0 - i;
        send_packet.horizontal_dilution_of_precision = 67.9 - i;
        send_packet.vertical_dilution_of_precision = 99.0 - i;
        std::cout << "Message Number : " << i << std::endl;

        std::cout << "Size of gps_packet_t : " << sizeof(gps_packet_t) << std::endl;

        memcpy(pduMessage.basePacket.payload, &send_packet, sizeof(gps_packet_t));

        std::cout << "Size of PDU Message : " << sizeof(pduMessage) << std::endl;

        sendto(sockfd, &pduMessage, sizeof(PDUMessage), 0, (struct sockaddr *)&sendAddr, sizeof(sendAddr));

        // std::this_thread::sleep_for(std::chrono::seconds(1)); // Delay between messages
        milisleep(1000);
    }
}

// Function to send data over UDP
void send_via_udp()
{
    // Wait for receiver to be ready
    std::cout << "Waiting fr receiver side to receive data : " << std::endl;

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, []
            { return ready; });

    // std::cout<<"Waiting for send Data : "<<std::endl;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create UDP socket\n";
        return;
    }

    struct sockaddr_in sendAddr;
    memset(&sendAddr, 0, sizeof(sendAddr));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(RECEIVER_PORT);
    inet_pton(AF_INET, "127.0.0.1", &sendAddr.sin_addr);

    get_gps_data(sockfd, sendAddr);

    close(sockfd);
}

// Function to receive data over UDP
void receive_via_udp()
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create UDP socket\n";
        return;
    }

    struct sockaddr_in receiverAddr;
    // struct sockaddr_in receiverAddr, senderAddr;
    // socklen_t senderSize = sizeof(senderAddr);
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_addr.s_addr = INADDR_ANY;
    receiverAddr.sin_port = htons(SENDER_PORT);

    if (bind(sockfd, (struct sockaddr *)&receiverAddr, sizeof(receiverAddr)) < 0)
    {
        std::cerr << "receiver code Bind failed\n";
        close(sockfd);
        return;
    }

    struct sockaddr_in senderAddr;
    socklen_t senderSize = sizeof(senderAddr);
    char buffer[45008]; // Increase buffer size to accommodate larger JSON data
    // char buffer[45008];

    while (true)
    {
        std::cout << "Waiting for data... " << std::endl;
        ssize_t bytesReceived = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&senderAddr, &senderSize);
        if (bytesReceived < 0)
        {
            std::cerr << "Error in recvfrom()\n";
            // break;
        }
        // PDUMessage pduMessage;
        // pduMessage.messageID = 1;
        // pduMessage.messageLength = sizeof(BasePacket);

        // BasePacket basePacket;
        // pduMessage.basePacket.crc32 = 0;
        // pduMessage.basePacket.count = htons(10);
        // pduMessage.basePacket.version = htons(1);
        // pduMessage.basePacket.timestamp = 1234;
        // //bas

        std::string receivedData(buffer, bytesReceived);

        // send_to_Rads_Router(route_Message, receivedData);

        // std::cout<< "Raw Received Data : " << receivedData;
        //  std::cout << "Received data: " << receivedData << std::endl;

        // std::cout << "Received data: Latitude=" << receivedData.latitude << ", Longitude=" << receivedData.longitude << std::endl;

        // Parse received JSON data

        ReceivedData parsedData = parseJsonData(receivedData);

        // PDUMessage route_Message = create_PDUMessage(10, 300, parsedData);
        // send_to_Rads_Router(route_Message);

        // create_PDUMessage(10, 300, parsedData);

        // pduMessage.basePacket.payload = parsedData;
        //  create_PDUMessage(parsedData, pd)
        std::cout << "Path Size : " << parsedData.path_size << std::endl;
        std::cout << "Instruction Size : " << parsedData.instruction_size << std::endl;

        std::cout << "Path coordinates:" << std::endl;

        for (const auto &coord : parsedData.path)
        {
            std::cout << "(" << coord.first << ", " << coord.second << ")" << std::endl;
        }
        std::cout << "Distance: " << parsedData.distance << std::endl;

        std::cout << "Instructions:" << std::endl;
        int i = 0;
        for (const auto &instr : parsedData.instructions)
        {
            /*
            std::cout << instr.begin_shape_index << std::endl;
            std::cout << instr.cost << std::endl;
            std::cout << instr.end_shape_index << std::endl;
            std::cout << instr.instruction.data() << std::endl;
            */
/*/
            std::cout << "Type: " << instr.type << std::endl;
            std::cout << "Instruction: " << i++;
            // for (const auto &value : instr.instruction)
            // {
            //     std::cout << static_cast<char>(value);
            // }
            std::cout << instr.instruction.data();
            std::cout << std::endl;

            std::cout << "Verbal Post Transition Instruction: ";

            // for (const auto &value : instr.verbal_post_transition_instruction)

            // {
            //     if(value != '\0'){
            //         std::cout << static_cast<char>(value);
            //     }
            // }

            std::cout << instr.verbal_post_transition_instruction.data();
            std::cout << std::endl;

            // Print other instruction fields as needed
        }
        std::cout << "Time: " << parsedData.time << std::endl;

        // Notify main thread that data has been received
        {
            std::lock_guard<std::mutex> lock(mtx);
            ready = true;
        }
        cv.notify_one();
    }
    milisleep(1000);

    close(sockfd);
}

int main()
{
    std::cout << "Sender thread" << std::endl;

    std::thread sender(send_via_udp);
    std::cout << "Receiver thread" << std::endl;

    while (true)
    {
        std::thread receiver(receive_via_udp);
        receiver.join();
    }

    // Wait for receiver thread to receive data
    {
        std::lock_guard<std::mutex> lock(mtx);
        ready = true;
    }
    cv.notify_one();

    // Do something with the received data

    sender.join();

    return 0;
}
*/