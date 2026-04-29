#include "activeGuidanceBBB.h"
#include <cstring>
#include <iostream>
#include <chrono>

ActiveGuidanceBBB::ActiveGuidanceBBB()
    : networkIpAddress_("192.168.6.1")
    , networkPort_(5555)
    , uartDevicePath_("/dev/ttyO0")
    , uartBaudRate_(115200)
    , spiDevicePath_("/dev/spidev0.0")
    , spiSpeedHz_(1000000)
    , networkInitialized_(false)
    , uartInitialized_(false)
    , spiInitialized_(false)
    , lastUpdateTime_(0)
    , updateCount_(0)
    , missedDeadlines_(0)
    , healthStatus_(0)
    , lastHealthCheck_(0) {
}

ActiveGuidanceBBB::~ActiveGuidanceBBB() {
    cleanupHardware();
}

bool ActiveGuidanceBBB::initializeHardware() {
    std::cout << "Initializing BBB hardware interfaces..." << std::endl;

    // Note: Actual hardware initialization would be done by the gateway
    // This is just a placeholder for the interface setup
    networkInitialized_ = true;
    uartInitialized_ = true;
    spiInitialized_ = true;

    std::cout << "BBB hardware interfaces initialized" << std::endl;
    return true;
}

void ActiveGuidanceBBB::cleanupHardware() {
    std::cout << "Cleaning up BBB hardware interfaces..." << std::endl;

    networkInitialized_ = false;
    uartInitialized_ = false;
    spiInitialized_ = false;

    std::cout << "BBB hardware interfaces cleaned up" << std::endl;
}

void ActiveGuidanceBBB::updateStateBBB(uint64_t currentSimNanos) {
    // Check for timing deadlines
    checkTimingDeadlines();

    // Update the base active guidance state
    UpdateState(currentSimNanos);

    // Update timing statistics
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    if (lastUpdateTime_ > 0) {
        uint64_t deltaTime = currentTime - lastUpdateTime_;
        // Check if we missed any deadlines (assuming 100Hz update rate = 10ms)
        const uint64_t expectedDelta = 10000000ULL;  // 10ms in nanoseconds
        if (deltaTime > expectedDelta * 2) {  // Allow 2x tolerance
            missedDeadlines_++;
        }
    }

    lastUpdateTime_ = currentTime;
    updateCount_++;

    // Update health status periodically
    if (currentTime - lastHealthCheck_ > 1000000000ULL) {  // Every 1 second
        updateHealthStatus();
        lastHealthCheck_ = currentTime;
    }
}

int ActiveGuidanceBBB::getStatePacket(uint8_t* buffer, int bufferSize) {
    if (!buffer || bufferSize < 64) {
        return -1;
    }

    // Packet structure:
    // [0-3]:   Magic number (0x42424242)
    // [4-7]:   Version (1)
    // [8-15]:  Timestamp (uint64_t)
    // [16-23]: State string (8 bytes, null-terminated)
    // [24-31]: Active ground station (8 bytes, null-terminated)
    // [32-35]: Health status (uint32_t)
    // [36-39]: Update count (uint32_t)
    // [40-43]: Missed deadlines (uint32_t)
    // [44-63]: Reserved (20 bytes)

    const uint32_t magicNumber = 0x42424242;
    const uint32_t version = 1;
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    std::string stateStr = getState();
    std::string stationStr = getActiveGroundStation();

    // Write header
    memcpy(buffer + 0, &magicNumber, sizeof(magicNumber));
    memcpy(buffer + 4, &version, sizeof(version));
    memcpy(buffer + 8, &currentTime, sizeof(currentTime));

    // Write state string (truncated to 8 bytes)
    memset(buffer + 16, 0, 8);
    strncpy(reinterpret_cast<char*>(buffer + 16), stateStr.c_str(), 7);

    // Write ground station string (truncated to 8 bytes)
    memset(buffer + 24, 0, 8);
    strncpy(reinterpret_cast<char*>(buffer + 24), stationStr.c_str(), 7);

    // Write statistics
    memcpy(buffer + 32, &healthStatus_, sizeof(healthStatus_));
    memcpy(buffer + 36, &updateCount_, sizeof(updateCount_));
    memcpy(buffer + 40, &missedDeadlines_, sizeof(missedDeadlines_));

    // Clear reserved area
    memset(buffer + 44, 0, 20);

    return 64;  // Total packet size
}

bool ActiveGuidanceBBB::applyCommandPacket(const uint8_t* buffer, int bufferSize) {
    if (!buffer || bufferSize < 32) {
        return false;
    }

    // Packet structure:
    // [0-3]:   Magic number (0x43434343)
    // [4-7]:   Version (1)
    // [8-15]:  Timestamp (uint64_t)
    // [16-23]: Command type (8 bytes, null-terminated)
    // [24-31]: Command data (8 bytes)

    const uint32_t magicNumber = 0x43434343;
    uint32_t receivedMagic;
    memcpy(&receivedMagic, buffer + 0, sizeof(receivedMagic));

    if (receivedMagic != magicNumber) {
        std::cerr << "Invalid command packet magic number" << std::endl;
        return false;
    }

    uint32_t version;
    memcpy(&version, buffer + 4, sizeof(version));

    if (version != 1) {
        std::cerr << "Unsupported command packet version: " << version << std::endl;
        return false;
    }

    // Extract command type
    char commandType[9];
    memset(commandType, 0, sizeof(commandType));
    memcpy(commandType, buffer + 16, 8);

    // Process command
    std::string cmdStr(commandType);
    if (cmdStr == "SETMODE") {
        // Set mode command
        char modeStr[9];
        memset(modeStr, 0, sizeof(modeStr));
        memcpy(modeStr, buffer + 24, 8);
        setModeString(modeStr);
        std::cout << "Set mode to: " << modeStr << std::endl;
        return true;
    } else if (cmdStr == "RESET") {
        // Reset command
        Reset(0);
        std::cout << "Reset active guidance" << std::endl;
        return true;
    } else if (cmdStr == "STATUS") {
        // Status request (no action needed, handled by state packet)
        return true;
    }

    std::cerr << "Unknown command type: " << cmdStr << std::endl;
    return false;
}

uint32_t ActiveGuidanceBBB::getHealthStatus() const {
    return healthStatus_;
}

void ActiveGuidanceBBB::setNetworkConfig(const std::string& ipAddress, int port) {
    networkIpAddress_ = ipAddress;
    networkPort_ = port;
    std::cout << "Network config set: " << ipAddress << ":" << port << std::endl;
}

void ActiveGuidanceBBB::setUARTConfig(const std::string& devicePath, int baudRate) {
    uartDevicePath_ = devicePath;
    uartBaudRate_ = baudRate;
    std::cout << "UART config set: " << devicePath << " at " << baudRate << " baud" << std::endl;
}

void ActiveGuidanceBBB::setSPIConfig(const std::string& devicePath, uint32_t speedHz) {
    spiDevicePath_ = devicePath;
    spiSpeedHz_ = speedHz;
    std::cout << "SPI config set: " << devicePath << " at " << speedHz << " Hz" << std::endl;
}

void ActiveGuidanceBBB::updateHealthStatus() {
    // Build health status word
    // Bit 0: Network interface OK
    // Bit 1: UART interface OK
    // Bit 2: SPI interface OK
    // Bit 3: No missed deadlines in last second
    // Bit 4-7: Reserved
    // Bit 8-15: Error count (limited to 255)
    // Bit 16-31: Reserved

    healthStatus_ = 0;

    if (networkInitialized_) {
        healthStatus_ |= (1 << 0);
    }
    if (uartInitialized_) {
        healthStatus_ |= (1 << 1);
    }
    if (spiInitialized_) {
        healthStatus_ |= (1 << 2);
    }

    // Check for missed deadlines in the last second
    uint32_t recentMissedDeadlines = missedDeadlines_;
    if (recentMissedDeadlines == 0) {
        healthStatus_ |= (1 << 3);
    }

    // Add error count (limited to 255)
    uint8_t errorCount = (recentMissedDeadlines > 255) ? 255 : recentMissedDeadlines;
    healthStatus_ |= (errorCount << 8);
}

void ActiveGuidanceBBB::checkTimingDeadlines() {
    // This is called during updateStateBBB to check for timing issues
    // The actual deadline checking is done in updateStateBBB
}