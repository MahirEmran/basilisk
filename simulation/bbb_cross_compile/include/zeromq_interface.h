#ifndef ZEROMQ_INTERFACE_H
#define ZEROMQ_INTERFACE_H

#include <string>
#include <cstdint>
#include <functional>

/**
 * @brief ZeroMQ interface for BBB-Basilisk communication
 *
 * This class provides a high-level interface for ZeroMQ-based
 * communication between the BeagleBone Black and Basilisk simulation.
 */
class ZeroMQInterface {
public:
    // Callback types for message handling
    using MessageCallback = std::function<void(const uint8_t*, int)>;

    /**
     * @brief Constructor
     */
    ZeroMQInterface();

    /**
     * @brief Destructor
     */
    ~ZeroMQInterface();

    /**
     * @brief Initialize as a publisher (BBB -> Basilisk)
     * @param bindAddress Address to bind to (e.g., "tcp://*:5555")
     * @return true if initialization successful, false otherwise
     */
    bool initPublisher(const std::string& bindAddress);

    /**
     * @brief Initialize as a subscriber (Basilisk -> BBB)
     * @param connectAddress Address to connect to (e.g., "tcp://192.168.1.100:5556")
     * @param topic Topic to subscribe to (empty string for all messages)
     * @return true if initialization successful, false otherwise
     */
    bool initSubscriber(const std::string& connectAddress, const std::string& topic = "");

    /**
     * @brief Initialize as a request-reply client
     * @param connectAddress Address to connect to (e.g., "tcp://192.168.1.100:5557")
     * @return true if initialization successful, false otherwise
     */
    bool initRequestClient(const std::string& connectAddress);

    /**
     * @brief Initialize as a request-reply server
     * @param bindAddress Address to bind to (e.g., "tcp://*:5557")
     * @return true if initialization successful, false otherwise
     */
    bool initRequestServer(const std::string& bindAddress);

    /**
     * @brief Send a message (for publisher or request client)
     * @param data Message data
     * @param size Message size in bytes
     * @return true if send successful, false otherwise
     */
    bool sendMessage(const uint8_t* data, int size);

    /**
     * @brief Send a message with topic (for publisher)
     * @param topic Message topic
     * @param data Message data
     * @param size Message size in bytes
     * @return true if send successful, false otherwise
     */
    bool sendMessage(const std::string& topic, const uint8_t* data, int size);

    /**
     * @brief Receive a message (blocking)
     * @param buffer Output buffer for received message
     * @param bufferSize Size of output buffer
     * @param timeoutMs Timeout in milliseconds (0 for non-blocking, -1 for infinite)
     * @return Number of bytes received, or -1 on error/timeout
     */
    int receiveMessage(uint8_t* buffer, int bufferSize, int timeoutMs = -1);

    /**
     * @brief Set callback for received messages (for subscriber or request server)
     * @param callback Function to call when a message is received
     */
    void setMessageCallback(MessageCallback callback);

    /**
     * @brief Process incoming messages (non-blocking)
     * @return Number of messages processed
     */
    int processMessages();

    /**
     * @brief Check if interface is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Get statistics about the interface
     * @param messagesSent Output parameter for messages sent
     * @param messagesReceived Output parameter for messages received
     * @param bytesSent Output parameter for bytes sent
     * @param bytesReceived Output parameter for bytes received
     */
    void getStatistics(uint64_t& messagesSent, uint64_t& messagesReceived,
                      uint64_t& bytesSent, uint64_t& bytesReceived) const;

    /**
     * @brief Reset statistics
     */
    void resetStatistics();

    /**
     * @brief Cleanup and close the interface
     */
    void cleanup();

private:
    void* context_;           // ZeroMQ context
    void* socket_;           // ZeroMQ socket
    bool initialized_;       // Initialization status
    int socketType_;         // Socket type (PUB, SUB, REQ, REP)

    // Statistics
    uint64_t messagesSent_;
    uint64_t messagesReceived_;
    uint64_t bytesSent_;
    uint64_t bytesReceived_;

    // Callback for message handling
    MessageCallback messageCallback_;

    // Helper methods
    bool createSocket(int socketType);
    void closeSocket();
};

#endif // ZEROMQ_INTERFACE_H