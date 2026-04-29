#include "zeromq_interface.h"
#include <zmq.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

ZeroMQInterface::ZeroMQInterface()
    : context_(nullptr)
    , socket_(nullptr)
    , initialized_(false)
    , socketType_(0)
    , messagesSent_(0)
    , messagesReceived_(0)
    , bytesSent_(0)
    , bytesReceived_(0)
    , messageCallback_(nullptr) {
}

ZeroMQInterface::~ZeroMQInterface() {
    cleanup();
}

bool ZeroMQInterface::initPublisher(const std::string& bindAddress) {
    if (initialized_) {
        std::cerr << "ZeroMQInterface already initialized" << std::endl;
        return false;
    }

    if (!createSocket(ZMQ_PUB)) {
        return false;
    }

    // Set socket options for better performance
    int sndhwm = 1000;  // Send high water mark
    zmq_setsockopt(socket_, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));

    // Bind to address
    if (zmq_bind(socket_, bindAddress.c_str()) != 0) {
        std::cerr << "Failed to bind to address: " << bindAddress << std::endl;
        closeSocket();
        return false;
    }

    initialized_ = true;
    std::cout << "ZeroMQ publisher initialized on: " << bindAddress << std::endl;
    return true;
}

bool ZeroMQInterface::initSubscriber(const std::string& connectAddress, const std::string& topic) {
    if (initialized_) {
        std::cerr << "ZeroMQInterface already initialized" << std::endl;
        return false;
    }

    if (!createSocket(ZMQ_SUB)) {
        return false;
    }

    // Set socket options for better performance
    int rcvhwm = 1000;  // Receive high water mark
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));

    // Set timeout for non-blocking receive
    int timeout = 100;  // 100ms
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

    // Subscribe to topic (empty string for all messages)
    if (!topic.empty()) {
        zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, topic.c_str(), topic.length());
    } else {
        zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    }

    // Connect to address
    if (zmq_connect(socket_, connectAddress.c_str()) != 0) {
        std::cerr << "Failed to connect to address: " << connectAddress << std::endl;
        closeSocket();
        return false;
    }

    initialized_ = true;
    std::cout << "ZeroMQ subscriber connected to: " << connectAddress << std::endl;
    return true;
}

bool ZeroMQInterface::initRequestClient(const std::string& connectAddress) {
    if (initialized_) {
        std::cerr << "ZeroMQInterface already initialized" << std::endl;
        return false;
    }

    if (!createSocket(ZMQ_REQ)) {
        return false;
    }

    // Set socket options
    int timeout = 5000;  // 5 second timeout
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(socket_, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));

    // Connect to address
    if (zmq_connect(socket_, connectAddress.c_str()) != 0) {
        std::cerr << "Failed to connect to address: " << connectAddress << std::endl;
        closeSocket();
        return false;
    }

    initialized_ = true;
    std::cout << "ZeroMQ request client connected to: " << connectAddress << std::endl;
    return true;
}

bool ZeroMQInterface::initRequestServer(const std::string& bindAddress) {
    if (initialized_) {
        std::cerr << "ZeroMQInterface already initialized" << std::endl;
        return false;
    }

    if (!createSocket(ZMQ_REP)) {
        return false;
    }

    // Bind to address
    if (zmq_bind(socket_, bindAddress.c_str()) != 0) {
        std::cerr << "Failed to bind to address: " << bindAddress << std::endl;
        closeSocket();
        return false;
    }

    initialized_ = true;
    std::cout << "ZeroMQ request server initialized on: " << bindAddress << std::endl;
    return true;
}

bool ZeroMQInterface::sendMessage(const uint8_t* data, int size) {
    if (!initialized_ || !data || size <= 0) {
        return false;
    }

    zmq_msg_t message;
    zmq_msg_init_size(&message, size);
    memcpy(zmq_msg_data(&message), data, size);

    int result = zmq_msg_send(&message, socket_, 0);
    zmq_msg_close(&message);

    if (result < 0) {
        std::cerr << "Failed to send message: " << zmq_strerror(zmq_errno()) << std::endl;
        return false;
    }

    messagesSent_++;
    bytesSent_ += size;
    return true;
}

bool ZeroMQInterface::sendMessage(const std::string& topic, const uint8_t* data, int size) {
    if (!initialized_ || socketType_ != ZMQ_PUB) {
        std::cerr << "sendMessage with topic only works for publisher sockets" << std::endl;
        return false;
    }

    // Send topic first
    zmq_msg_t topic_msg;
    zmq_msg_init_size(&topic_msg, topic.length());
    memcpy(zmq_msg_data(&topic_msg), topic.c_str(), topic.length());

    int result = zmq_msg_send(&topic_msg, socket_, ZMQ_SNDMORE);
    zmq_msg_close(&topic_msg);

    if (result < 0) {
        std::cerr << "Failed to send topic: " << zmq_strerror(zmq_errno()) << std::endl;
        return false;
    }

    // Send data
    return sendMessage(data, size);
}

int ZeroMQInterface::receiveMessage(uint8_t* buffer, int bufferSize, int timeoutMs) {
    if (!initialized_ || !buffer || bufferSize <= 0) {
        return -1;
    }

    // Set timeout if specified
    if (timeoutMs >= 0) {
        zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeoutMs, sizeof(timeoutMs));
    }

    zmq_msg_t message;
    zmq_msg_init(&message);

    int result = zmq_msg_recv(&message, socket_, 0);

    if (result < 0) {
        zmq_msg_close(&message);
        if (zmq_errno() == EAGAIN) {
            return 0;  // Timeout
        }
        std::cerr << "Failed to receive message: " << zmq_strerror(zmq_errno()) << std::endl;
        return -1;
    }

    int size = zmq_msg_size(&message);
    if (size > bufferSize) {
        std::cerr << "Received message too large for buffer" << std::endl;
        zmq_msg_close(&message);
        return -1;
    }

    memcpy(buffer, zmq_msg_data(&message), size);
    zmq_msg_close(&message);

    messagesReceived_++;
    bytesReceived_ += size;
    return size;
}

void ZeroMQInterface::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

int ZeroMQInterface::processMessages() {
    if (!initialized_ || !messageCallback_) {
        return 0;
    }

    int processed = 0;
    uint8_t buffer[4096];  // Buffer for received messages

    while (true) {
        int size = receiveMessage(buffer, sizeof(buffer), 0);  // Non-blocking
        if (size <= 0) {
            break;  // No more messages
        }

        messageCallback_(buffer, size);
        processed++;
    }

    return processed;
}

bool ZeroMQInterface::isInitialized() const {
    return initialized_;
}

void ZeroMQInterface::getStatistics(uint64_t& messagesSent, uint64_t& messagesReceived,
                                   uint64_t& bytesSent, uint64_t& bytesReceived) const {
    messagesSent = messagesSent_;
    messagesReceived = messagesReceived_;
    bytesSent = bytesSent_;
    bytesReceived = bytesReceived_;
}

void ZeroMQInterface::resetStatistics() {
    messagesSent_ = 0;
    messagesReceived_ = 0;
    bytesSent_ = 0;
    bytesReceived_ = 0;
}

void ZeroMQInterface::cleanup() {
    if (initialized_) {
        closeSocket();
        initialized_ = false;
    }
}

bool ZeroMQInterface::createSocket(int socketType) {
    // Create context
    context_ = zmq_ctx_new();
    if (!context_) {
        std::cerr << "Failed to create ZeroMQ context" << std::endl;
        return false;
    }

    // Create socket
    socket_ = zmq_socket(context_, socketType);
    if (!socket_) {
        std::cerr << "Failed to create ZeroMQ socket" << std::endl;
        zmq_ctx_term(context_);
        context_ = nullptr;
        return false;
    }

    socketType_ = socketType;
    return true;
}

void ZeroMQInterface::closeSocket() {
    if (socket_) {
        zmq_close(socket_);
        socket_ = nullptr;
    }

    if (context_) {
        zmq_ctx_term(context_);
        context_ = nullptr;
    }
}