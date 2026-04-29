#ifndef ACTIVE_GUIDANCE_BBB_H
#define ACTIVE_GUIDANCE_BBB_H

#include "activeGuidanceStandalone.h"
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief BBB-specific active guidance module
 *
 * This class extends the base ActiveGuidanceStandalone class with BBB-specific
 * functionality for hardware interfaces and real-time operation.
 */
class ActiveGuidanceBBB : public ActiveGuidanceStandalone {
public:
    ActiveGuidanceBBB();
    ~ActiveGuidanceBBB();

    /**
     * @brief Initialize BBB-specific hardware interfaces
     * @return true if initialization successful, false otherwise
     */
    bool initializeHardware();

    /**
     * @brief Cleanup BBB-specific hardware interfaces
     */
    void cleanupHardware();

    /**
     * @brief Update guidance state with BBB-specific timing
     * @param currentSimNanos Current simulation time in nanoseconds
     */
    void updateStateBBB(uint64_t currentSimNanos);

    /**
     * @brief Get the current guidance state as a binary packet
     * @param buffer Output buffer for the packet
     * @param bufferSize Size of the output buffer
     * @return Number of bytes written to buffer
     */
    int getStatePacket(uint8_t* buffer, int bufferSize);

    /**
     * @brief Parse and apply external command packet
     * @param buffer Input buffer containing the command packet
     * @param bufferSize Size of the input buffer
     * @return true if packet was valid and applied, false otherwise
     */
    bool applyCommandPacket(const uint8_t* buffer, int bufferSize);

    /**
     * @brief Get system health status
     * @return Health status code
     */
    uint32_t getHealthStatus() const;

    /**
     * @brief Set network configuration
     * @param ipAddress IP address for network communication
     * @param port Port number for network communication
     */
    void setNetworkConfig(const std::string& ipAddress, int port);

    /**
     * @brief Set UART configuration
     * @param devicePath UART device path (e.g., "/dev/ttyO0")
     * @param baudRate Baud rate for UART communication
     */
    void setUARTConfig(const std::string& devicePath, int baudRate);

    /**
     * @brief Set SPI configuration
     * @param devicePath SPI device path (e.g., "/dev/spidev0.0")
     * @param speedHz SPI clock speed in Hz
     */
    void setSPIConfig(const std::string& devicePath, uint32_t speedHz);

private:
    // Network configuration
    std::string networkIpAddress_;
    int networkPort_;

    // UART configuration
    std::string uartDevicePath_;
    int uartBaudRate_;

    // SPI configuration
    std::string spiDevicePath_;
    uint32_t spiSpeedHz_;

    // Hardware interface status
    bool networkInitialized_;
    bool uartInitialized_;
    bool spiInitialized_;

    // Timing and performance monitoring
    uint64_t lastUpdateTime_;
    uint32_t updateCount_;
    uint32_t missedDeadlines_;

    // Health monitoring
    uint32_t healthStatus_;
    uint64_t lastHealthCheck_;

    /**
     * @brief Update health status based on current system state
     */
    void updateHealthStatus();

    /**
     * @brief Check for missed timing deadlines
     */
    void checkTimingDeadlines();
};

#endif // ACTIVE_GUIDANCE_BBB_H