#ifndef BBB_GATEWAY_H
#define BBB_GATEWAY_H

#include "activeGuidanceBBB.h"
#include "zeromq_interface.h"
#include "uart_interface.h"
#include "spi_interface.h"
#include <string>
#include <cstdint>
#include <memory>
#include <atomic>
#include <thread>

/**
 * @brief BBB Gateway process for HIL communication
 *
 * This class implements the main gateway process that runs on the
 * BeagleBone Black, handling communication between Basilisk and
 * the active guidance module via various interfaces.
 */
class BBBGateway {
public:
    // Gateway operation modes
    enum OperationMode {
        MODE_NETWORK_ONLY,      // Network communication only
        MODE_UART_ONLY,         // UART communication only
        MODE_SPI_ONLY,          // SPI communication only
        MODE_NETWORK_UART,      // Network + UART
        MODE_NETWORK_SPI,       // Network + SPI
        MODE_ALL_INTERFACES     // All interfaces
    };

    // Gateway status
    enum GatewayStatus {
        STATUS_STOPPED,
        STATUS_INITIALIZING,
        STATUS_RUNNING,
        STATUS_ERROR,
        STATUS_SHUTTING_DOWN
    };

    /**
     * @brief Constructor
     */
    BBBGateway();

    /**
     * @brief Destructor
     */
    ~BBBGateway();

    /**
     * @brief Initialize the gateway
     * @param configFilePath Path to configuration file
     * @return true if initialization successful, false otherwise
     */
    bool initialize(const std::string& configFilePath);

    /**
     * @brief Start the gateway
     * @return true if start successful, false otherwise
     */
    bool start();

    /**
     * @brief Stop the gateway
     */
    void stop();

    /**
     * @brief Get current gateway status
     * @return Current status
     */
    GatewayStatus getStatus() const;

    /**
     * @brief Set operation mode
     * @param mode Operation mode to set
     */
    void setOperationMode(OperationMode mode);

    /**
     * @brief Get operation mode
     * @return Current operation mode
     */
    OperationMode getOperationMode() const;

    /**
     * @brief Set update rate
     * @param rateHz Update rate in Hz
     */
    void setUpdateRate(double rateHz);

    /**
     * @brief Get update rate
     * @return Current update rate in Hz
     */
    double getUpdateRate() const;

    /**
     * @brief Get gateway statistics
     * @param uptimeSeconds Output parameter for uptime in seconds
     * @param loopCount Output parameter for number of loops executed
     * @param missedDeadlines Output parameter for missed deadlines
     * @param averageLoopTime Output parameter for average loop time in microseconds
     */
    void getStatistics(uint64_t& uptimeSeconds, uint64_t& loopCount,
                      uint32_t& missedDeadlines, double& averageLoopTime) const;

    /**
     * @brief Reset statistics
     */
    void resetStatistics();

private:
    // Components
    std::unique_ptr<ActiveGuidanceBBB> activeGuidance_;
    std::unique_ptr<ZeroMQInterface> zmqPublisher_;   // For publishing to Basilisk
    std::unique_ptr<ZeroMQInterface> zmqSubscriber_; // For subscribing from Basilisk
    std::unique_ptr<UARTInterface> uartInterface_;
    std::unique_ptr<SPIInterface> spiInterface_;

    // Configuration
    OperationMode operationMode_;
    double updateRateHz_;
    std::string configFilePath_;

    // Runtime state
    std::atomic<GatewayStatus> status_;
    std::atomic<bool> running_;
    std::thread gatewayThread_;

    // Timing and statistics
    uint64_t startTime_;
    uint64_t loopCount_;
    uint32_t missedDeadlines_;
    double totalLoopTime_;

    uint32_t commandSequence_;
    uint64_t lastTruthTimestamp_;
    uint32_t lastTruthSequence_;
    bool hasTruthNav_;

    // Network configuration
    std::string zmqPublisherAddress_;
    std::string zmqSubscriberAddress_;
    std::string zmqRequestAddress_;

    // UART configuration
    std::string uartDevicePath_;
    int uartBaudRate_;

    // SPI configuration
    std::string spiDevicePath_;
    uint32_t spiSpeedHz_;

    // Private methods
    bool loadConfiguration(const std::string& configFilePath);
    bool initializeNetwork();
    bool initializeUART();
    bool initializeSPI();
    bool initializeActiveGuidance();

    void gatewayLoop();
    void processNetworkMessages();
    void processUARTMessages();
    void processSPIMessages();
    void updateActiveGuidance();

    void sendStateToBasilisk();
    void receiveCommandsFromBasilisk();

    void handleNetworkMessage(const uint8_t* data, int size);
    void handleUARTMessage(const uint8_t* data, int size);
    void handleSPIMessage(const uint8_t* data, int size);

    bool parseTruthNavPacket(const uint8_t* data, int size);
    bool parseCommandPacket(const uint8_t* data, int size);
    bool buildFcCommandPacket(uint8_t* buffer, int bufferSize, int& bytesWritten);
    bool createStatePacket(uint8_t* buffer, int bufferSize, int& bytesWritten);

    void updateStatistics(double loopTimeUs);
    void cleanup();
};

#endif // BBB_GATEWAY_H