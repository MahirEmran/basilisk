#include "bbb_gateway.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <unistd.h>

namespace {
constexpr uint32_t kTruthNavMagic = 0x42424242;
constexpr uint32_t kCommandMagic = 0x43434343;
constexpr uint16_t kProtocolVersion = 1;
constexpr uint16_t kPacketTypeTruthNav = 0x01;
constexpr uint16_t kPacketTypeFcCommand = 0x04;
constexpr size_t kHeaderSize = 16;
constexpr size_t kCrcSize = 4;
constexpr size_t kTruthNavBodySize = 112;
constexpr size_t kCommandTypeSize = 8;
constexpr uint64_t kCommandValidityWindowNanos = 1000000000ULL;  // [ns]

uint32_t crc32_compute(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t idx = 0; idx < length; ++idx) {
        crc ^= data[idx];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = static_cast<uint32_t>(-(static_cast<int>(crc & 1u)));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}
}  // namespace

BBBGateway::BBBGateway()
    : activeGuidance_(nullptr)
    , zmqPublisher_(nullptr)
    , zmqSubscriber_(nullptr)
    , uartInterface_(nullptr)
    , spiInterface_(nullptr)
    , operationMode_(MODE_NETWORK_ONLY)
    , updateRateHz_(100.0)
    , status_(STATUS_STOPPED)
    , running_(false)
    , startTime_(0)
    , loopCount_(0)
    , missedDeadlines_(0)
    , totalLoopTime_(0.0)
    , commandSequence_(0)
    , lastTruthTimestamp_(0)
    , lastTruthSequence_(0)
    , hasTruthNav_(false)
    , uartBaudRate_(115200)
    , spiSpeedHz_(1000000) {
}

BBBGateway::~BBBGateway() {
    stop();
    cleanup();
}

bool BBBGateway::initialize(const std::string& configFilePath) {
    std::cout << "Initializing BBB Gateway..." << std::endl;

    configFilePath_ = configFilePath;
    status_ = STATUS_INITIALIZING;

    // Load configuration
    if (!loadConfiguration(configFilePath)) {
        std::cerr << "Failed to load configuration from: " << configFilePath << std::endl;
        status_ = STATUS_ERROR;
        return false;
    }

    // Initialize components based on operation mode
    if (!initializeActiveGuidance()) {
        std::cerr << "Failed to initialize active guidance" << std::endl;
        status_ = STATUS_ERROR;
        return false;
    }

    // Initialize network if needed
    if (operationMode_ == MODE_NETWORK_ONLY ||
        operationMode_ == MODE_NETWORK_UART ||
        operationMode_ == MODE_NETWORK_SPI ||
        operationMode_ == MODE_ALL_INTERFACES) {
        if (!initializeNetwork()) {
            std::cerr << "Failed to initialize network" << std::endl;
            status_ = STATUS_ERROR;
            return false;
        }
    }

    // Initialize UART if needed
    if (operationMode_ == MODE_UART_ONLY ||
        operationMode_ == MODE_NETWORK_UART ||
        operationMode_ == MODE_ALL_INTERFACES) {
        if (!initializeUART()) {
            std::cerr << "Failed to initialize UART" << std::endl;
            status_ = STATUS_ERROR;
            return false;
        }
    }

    // Initialize SPI if needed
    if (operationMode_ == MODE_SPI_ONLY ||
        operationMode_ == MODE_NETWORK_SPI ||
        operationMode_ == MODE_ALL_INTERFACES) {
        if (!initializeSPI()) {
            std::cerr << "Failed to initialize SPI" << std::endl;
            status_ = STATUS_ERROR;
            return false;
        }
    }

    status_ = STATUS_STOPPED;
    std::cout << "BBB Gateway initialized successfully" << std::endl;
    return true;
}

bool BBBGateway::start() {
    if (status_ != STATUS_STOPPED) {
        std::cerr << "Gateway is not in stopped state" << std::endl;
        return false;
    }

    std::cout << "Starting BBB Gateway..." << std::endl;

    status_ = STATUS_RUNNING;
    running_ = true;
    startTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // Start gateway thread
    gatewayThread_ = std::thread(&BBBGateway::gatewayLoop, this);

    std::cout << "BBB Gateway started" << std::endl;
    return true;
}

void BBBGateway::stop() {
    if (status_ != STATUS_RUNNING) {
        return;
    }

    std::cout << "Stopping BBB Gateway..." << std::endl;

    status_ = STATUS_SHUTTING_DOWN;
    running_ = false;

    // Wait for gateway thread to finish
    if (gatewayThread_.joinable()) {
        gatewayThread_.join();
    }

    status_ = STATUS_STOPPED;
    std::cout << "BBB Gateway stopped" << std::endl;
}

BBBGateway::GatewayStatus BBBGateway::getStatus() const {
    return status_;
}

void BBBGateway::setOperationMode(OperationMode mode) {
    operationMode_ = mode;
}

BBBGateway::OperationMode BBBGateway::getOperationMode() const {
    return operationMode_;
}

void BBBGateway::setUpdateRate(double rateHz) {
    updateRateHz_ = rateHz;
}

double BBBGateway::getUpdateRate() const {
    return updateRateHz_;
}

void BBBGateway::getStatistics(uint64_t& uptimeSeconds, uint64_t& loopCount,
                              uint32_t& missedDeadlines, double& averageLoopTime) const {
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    uptimeSeconds = (currentTime - startTime_) / 1000000000ULL;
    loopCount = loopCount_;
    missedDeadlines = missedDeadlines_;
    averageLoopTime = (loopCount_ > 0) ? (totalLoopTime_ / loopCount_) : 0.0;
}

void BBBGateway::resetStatistics() {
    loopCount_ = 0;
    missedDeadlines_ = 0;
    totalLoopTime_ = 0.0;
    startTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool BBBGateway::loadConfiguration(const std::string& configFilePath) {
    std::cout << "Loading configuration from: " << configFilePath << std::endl;

    // For now, use default configuration
    // In a real implementation, this would parse a YAML or JSON config file

    // Set default network configuration
    zmqPublisherAddress_ = "tcp://*:5555";
    zmqSubscriberAddress_ = "tcp://192.168.6.1:5566";
    zmqRequestAddress_ = "tcp://192.168.6.1:5567";

    // Set default UART configuration
    uartDevicePath_ = "/dev/ttyO0";
    uartBaudRate_ = 115200;

    // Set default SPI configuration
    spiDevicePath_ = "/dev/spidev0.0";
    spiSpeedHz_ = 1000000;

    // Set default operation mode
    operationMode_ = MODE_NETWORK_ONLY;

    // Set default update rate
    updateRateHz_ = 100.0;

    std::cout << "Configuration loaded successfully" << std::endl;
    return true;
}

bool BBBGateway::initializeNetwork() {
    std::cout << "Initializing network interface..." << std::endl;

    // Create separate interfaces for publisher and subscriber
    zmqPublisher_ = std::make_unique<ZeroMQInterface>();
    zmqSubscriber_ = std::make_unique<ZeroMQInterface>();

    // Initialize as publisher (BBB -> Basilisk)
    if (!zmqPublisher_->initPublisher(zmqPublisherAddress_)) {
        std::cerr << "Failed to initialize ZeroMQ publisher" << std::endl;
        return false;
    }

    // Initialize as subscriber (Basilisk -> BBB)
    if (!zmqSubscriber_->initSubscriber(zmqSubscriberAddress_)) {
        std::cerr << "Failed to initialize ZeroMQ subscriber" << std::endl;
        return false;
    }

    // Set message callback on subscriber
    zmqSubscriber_->setMessageCallback([this](const uint8_t* data, int size) {
        this->handleNetworkMessage(data, size);
    });

    std::cout << "Network interface initialized" << std::endl;
    return true;
}

bool BBBGateway::initializeUART() {
    std::cout << "Initializing UART interface..." << std::endl;

    uartInterface_ = std::make_unique<UARTInterface>();

    if (!uartInterface_->initialize(uartDevicePath_, uartBaudRate_)) {
        std::cerr << "Failed to initialize UART" << std::endl;
        return false;
    }

    // Set data callback
    uartInterface_->setDataCallback([this](const uint8_t* data, int size) {
        this->handleUARTMessage(data, size);
    });

    std::cout << "UART interface initialized" << std::endl;
    return true;
}

bool BBBGateway::initializeSPI() {
    std::cout << "Initializing SPI interface..." << std::endl;

    spiInterface_ = std::make_unique<SPIInterface>();

    if (!spiInterface_->initialize(spiDevicePath_, spiSpeedHz_)) {
        std::cerr << "Failed to initialize SPI" << std::endl;
        return false;
    }

    std::cout << "SPI interface initialized" << std::endl;
    return true;
}

bool BBBGateway::initializeActiveGuidance() {
    std::cout << "Initializing active guidance module..." << std::endl;

    activeGuidance_ = std::make_unique<ActiveGuidanceBBB>();

    if (!activeGuidance_->initializeHardware()) {
        std::cerr << "Failed to initialize active guidance hardware" << std::endl;
        return false;
    }

    // Reset the active guidance module
    activeGuidance_->Reset(0);

    std::cout << "Active guidance module initialized" << std::endl;
    return true;
}

void BBBGateway::gatewayLoop() {
    std::cout << "Gateway loop started at " << updateRateHz_ << " Hz" << std::endl;

    const double loopPeriodUs = 1000000.0 / updateRateHz_;  // Loop period in microseconds

    while (running_) {
        auto loopStart = std::chrono::steady_clock::now();

        // Get current time in nanoseconds
        uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // Process messages from all interfaces
        processNetworkMessages();
        processUARTMessages();
        processSPIMessages();

        // Update active guidance
        updateActiveGuidance();

        // Send state to Basilisk
        sendStateToBasilisk();

        // Receive commands from Basilisk
        receiveCommandsFromBasilisk();

        // Calculate loop time
        auto loopEnd = std::chrono::steady_clock::now();
        double loopTimeUs = std::chrono::duration<double, std::micro>(loopEnd - loopStart).count();

        // Update statistics
        updateStatistics(loopTimeUs);

        // Check for missed deadlines
        if (loopTimeUs > loopPeriodUs * 1.5) {  // Allow 50% tolerance
            missedDeadlines_++;
        }

        // Sleep for remaining time
        double remainingTimeUs = loopPeriodUs - loopTimeUs;
        if (remainingTimeUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(remainingTimeUs)));
        }

        loopCount_++;
    }

    std::cout << "Gateway loop ended" << std::endl;
}

void BBBGateway::processNetworkMessages() {
    if (zmqSubscriber_ && zmqSubscriber_->isInitialized()) {
        zmqSubscriber_->processMessages();
    }
}

void BBBGateway::processUARTMessages() {
    if (uartInterface_ && uartInterface_->isInitialized()) {
        uartInterface_->processData();
    }
}

void BBBGateway::processSPIMessages() {
    // SPI is typically request/response, so we don't process incoming messages here
    // SPI communication is handled on-demand
}

void BBBGateway::updateActiveGuidance() {
    if (activeGuidance_) {
        uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        activeGuidance_->updateStateBBB(currentTime);
    }
}

void BBBGateway::sendStateToBasilisk() {
    if (!zmqPublisher_ || !zmqPublisher_->isInitialized() || !activeGuidance_) {
        return;
    }

    uint8_t commandPacket[128];
    int packetSize = 0;
    if (!buildFcCommandPacket(commandPacket, sizeof(commandPacket), packetSize)) {
        return;
    }

    if (packetSize > 0) {
        zmqPublisher_->sendMessage(commandPacket, packetSize);
    }
}

void BBBGateway::receiveCommandsFromBasilisk() {
    // Commands are received via the message callback in handleNetworkMessage
}

void BBBGateway::handleNetworkMessage(const uint8_t* data, int size) {
    std::cout << "[BBB RX] Received network message: " << size << " bytes" << std::endl;

    if (!activeGuidance_) {
        return;
    }

    if (parseTruthNavPacket(data, size)) {
        return;
    }

    // Parse command packet
    if (parseCommandPacket(data, size)) {
        std::cout << "Received and processed command from Basilisk" << std::endl;
    }
}

void BBBGateway::handleUARTMessage(const uint8_t* data, int size) {
    std::cout << "Received UART message: " << size << " bytes" << std::endl;
    // Process UART message as needed
}

void BBBGateway::handleSPIMessage(const uint8_t* data, int size) {
    std::cout << "Received SPI message: " << size << " bytes" << std::endl;
    // Process SPI message as needed
}

bool BBBGateway::parseTruthNavPacket(const uint8_t* data, int size) {
    if (!data || size < static_cast<int>(kHeaderSize + kCrcSize)) {
        return false;
    }

    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t packetType = 0;
    uint32_t length = 0;
    uint32_t sequence = 0;

    memcpy(&magic, data + 0, sizeof(magic));
    memcpy(&version, data + 4, sizeof(version));
    memcpy(&packetType, data + 6, sizeof(packetType));
    memcpy(&length, data + 8, sizeof(length));
    memcpy(&sequence, data + 12, sizeof(sequence));

    if (magic != kTruthNavMagic || version != kProtocolVersion || packetType != kPacketTypeTruthNav) {
        return false;
    }

    if (length < kTruthNavBodySize) {
        return false;
    }

    const size_t totalSize = kHeaderSize + static_cast<size_t>(length) + kCrcSize;
    if (size < static_cast<int>(totalSize)) {
        return false;
    }

    uint32_t crcReceived = 0;
    memcpy(&crcReceived, data + kHeaderSize + length, sizeof(crcReceived));
    const uint32_t crcComputed = crc32_compute(data, kHeaderSize + length);

    std::cout << "[BBB RX] TruthNav: magic=" << magic << " version=" << version
              << " type=" << packetType << " length=" << length
              << " seq=" << sequence << " crc_recv=" << crcReceived
              << " crc_calc=" << crcComputed << std::endl;

    if (crcReceived != crcComputed) {
        std::cerr << "TruthNav CRC mismatch" << std::endl;
        return true;
    }

    uint64_t timestamp = 0;
    memcpy(&timestamp, data + kHeaderSize, sizeof(timestamp));

    lastTruthTimestamp_ = timestamp;
    lastTruthSequence_ = sequence;
    hasTruthNav_ = true;
    return true;
}

bool BBBGateway::parseCommandPacket(const uint8_t* data, int size) {
    if (!activeGuidance_) {
        return false;
    }

    return activeGuidance_->applyCommandPacket(data, size);
}

bool BBBGateway::buildFcCommandPacket(uint8_t* buffer, int bufferSize, int& bytesWritten) {
    if (!buffer || bufferSize <= 0) {
        return false;
    }

    const char commandTypeRaw[kCommandTypeSize] = {'S', 'T', 'A', 'T', 'U', 'S', 0, 0};
    const uint16_t commandDataLen = 0;

    const uint32_t bodyLength = static_cast<uint32_t>(8 + kCommandTypeSize + 2 + commandDataLen + 8);
    const size_t totalLength = kHeaderSize + bodyLength + kCrcSize;
    if (bufferSize < static_cast<int>(totalLength)) {
        return false;
    }

    const uint32_t magic = kCommandMagic;
    const uint16_t version = kProtocolVersion;
    const uint16_t packetType = kPacketTypeFcCommand;
    const uint32_t sequence = commandSequence_++;

    const uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    size_t offset = 0;
    memcpy(buffer + offset, &magic, sizeof(magic));
    offset += sizeof(magic);
    memcpy(buffer + offset, &version, sizeof(version));
    offset += sizeof(version);
    memcpy(buffer + offset, &packetType, sizeof(packetType));
    offset += sizeof(packetType);
    memcpy(buffer + offset, &bodyLength, sizeof(bodyLength));
    offset += sizeof(bodyLength);
    memcpy(buffer + offset, &sequence, sizeof(sequence));
    offset += sizeof(sequence);

    memcpy(buffer + offset, &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(buffer + offset, commandTypeRaw, kCommandTypeSize);
    offset += kCommandTypeSize;
    memcpy(buffer + offset, &commandDataLen, sizeof(commandDataLen));
    offset += sizeof(commandDataLen);
    memcpy(buffer + offset, &kCommandValidityWindowNanos, sizeof(kCommandValidityWindowNanos));
    offset += sizeof(kCommandValidityWindowNanos);

    const uint32_t crc = crc32_compute(buffer, kHeaderSize + bodyLength);
    memcpy(buffer + offset, &crc, sizeof(crc));
    offset += sizeof(crc);

    bytesWritten = static_cast<int>(offset);
    return true;
}

bool BBBGateway::createStatePacket(uint8_t* buffer, int bufferSize, int& bytesWritten) {
    if (!activeGuidance_) {
        return false;
    }

    bytesWritten = activeGuidance_->getStatePacket(buffer, bufferSize);
    return bytesWritten > 0;
}

void BBBGateway::updateStatistics(double loopTimeUs) {
    totalLoopTime_ += loopTimeUs;
}

void BBBGateway::cleanup() {
    std::cout << "Cleaning up BBB Gateway..." << std::endl;

    if (zmqPublisher_) {
        zmqPublisher_->cleanup();
        zmqPublisher_.reset();
    }

    if (zmqSubscriber_) {
        zmqSubscriber_->cleanup();
        zmqSubscriber_.reset();
    }

    if (uartInterface_) {
        uartInterface_->close();
        uartInterface_.reset();
    }

    if (spiInterface_) {
        spiInterface_->close();
        spiInterface_.reset();
    }

    if (activeGuidance_) {
        activeGuidance_->cleanupHardware();
        activeGuidance_.reset();
    }

    std::cout << "BBB Gateway cleaned up" << std::endl;
}